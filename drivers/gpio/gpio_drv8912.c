/*
 * Copyright (c) 2026 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_drv8912_gpio

/**
 * @file Driver for the TI DRV8912-Q1 12-channel half-bridge driver
 *
 * A half-bridge is a pair of switches, one tying the output to the VM supply
 * and one tying it to ground. The device holds twelve of them behind a SPI
 * interface, and each one is exposed here as a GPIO pin. A pin driven high
 * closes the high-side switch, a pin driven low closes the low-side switch,
 * and a disconnected pin opens both and leaves the output floating, which is
 * also the state the device powers up in.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_drv8912, CONFIG_GPIO_LOG_LEVEL);

/* The device answers on the SPI bus and nowhere else, so the bus comes up first. */
#if CONFIG_SPI_INIT_PRIORITY >= CONFIG_GPIO_DRV8912_INIT_PRIORITY
#error SPI_INIT_PRIORITY must be lower than GPIO_DRV8912_INIT_PRIORITY
#endif

/*
 * Register addresses. The device implements more than these three, the rest
 * carrying the per channel diagnostics and the PWM generators, neither of
 * which this driver uses.
 */
#define DRV8912_REG_IC_STAT     0x00U
#define DRV8912_REG_CONFIG_CTRL 0x07U
#define DRV8912_REG_OP_CTRL_1   0x08U

/*
 * IC status register fields. The power-on reset flag only reads back as one
 * once it has been acknowledged, which makes it a proof that the device took
 * a write rather than a fault to worry about.
 */
#define DRV8912_IC_STAT_NPOR   BIT(0)
#define DRV8912_IC_STAT_FAULTS GENMASK(6, 1)

/*
 * Configuration register fields. Acknowledging the faults is a command, its
 * bit clears itself once the device has acted on it, while the overvoltage
 * threshold is a setting that stays until it is written again.
 */
#define DRV8912_CONFIG_CLR_FLT BIT(0)
#define DRV8912_CONFIG_EXT_OVP BIT(1)

/* Overvoltage protection threshold, in millivolts, that a reset selects. */
#define DRV8912_OVP_STANDARD_MV 21000

/*
 * A SPI frame is sixteen bits: one leading zero, a read flag, a six bit
 * register address and a byte of data. The device answers with its fault
 * summary followed by the content of the accessed register.
 */
#define DRV8912_FRAME_READ BIT(14)
#define DRV8912_FRAME_ADDR GENMASK(13, 8)
#define DRV8912_FRAME_DATA GENMASK(7, 0)

/* The two leading bits of every answer are hardwired to one. */
#define DRV8912_ANSWER_MARK GENMASK(15, 14)

#define DRV8912_NUM_HALF_BRIDGES 12U
/* One operation control register holds four consecutive half-bridges. */
#define DRV8912_BRIDGES_PER_REG  4U
#define DRV8912_NUM_OP_CTRL_REGS (DRV8912_NUM_HALF_BRIDGES / DRV8912_BRIDGES_PER_REG)

/* Time the device needs to answer on the SPI bus after nSLEEP goes high. */
#define DRV8912_WAKE_TIME_US 200U
/* Minimum gap the device requires between two successive SPI writes. */
#define DRV8912_WRITE_GAP_US 3U

struct drv8912_config {
	/* Must come first, the GPIO subsystem casts dev->config to it. */
	struct gpio_driver_config common;
	struct spi_dt_spec bus;
	struct gpio_dt_spec nsleep;
	struct gpio_dt_spec nfault;
	/*
	 * What to write into the configuration register. The fault
	 * acknowledgment bit is added on top when it is needed, as it is a
	 * command and not part of the configuration.
	 */
	uint8_t config_ctrl;
};

struct drv8912_data {
	/* Must come first, the GPIO subsystem casts dev->data to it. */
	struct gpio_driver_data common;
	struct k_mutex lock;
	/*
	 * The state that was asked for, held as two masks because a
	 * half-bridge has three states where a GPIO level only has two. Bit N
	 * of driven says whether half-bridge N drives its output at all, and
	 * bit N of high says which rail it drives it to. A disconnected pin
	 * keeps its bit in high, so it remembers the level it would take were
	 * it driven again.
	 */
	gpio_port_pins_t driven;
	gpio_port_value_t high;
	/*
	 * Copy of what was last written to each operation control register.
	 * One register carries four half-bridges while the bus only moves
	 * whole bytes, so changing a single pin means writing the three others
	 * back unchanged, which is only possible by remembering them. The copy
	 * also tells a write that would change something from one that would
	 * not, and the second kind is simply skipped.
	 */
	uint8_t op_ctrl[DRV8912_NUM_OP_CTRL_REGS];
};

static int drv8912_transfer(const struct device *dev, uint16_t frame, uint8_t *report)
{
	const struct drv8912_config *config = dev->config;
	uint8_t tx[2];
	uint8_t rx[2];
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	uint16_t answer;
	int ret;

	/* A SPI transfer can block, so none of this may run in an interrupt. */
	__ASSERT(!k_is_in_isr(), "attempt to access SPI from ISR");

	sys_put_be16(frame, tx);

	ret = spi_transceive_dt(&config->bus, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI transfer failed (%d)", ret);
		return ret;
	}

	answer = sys_get_be16(rx);

	/*
	 * Nothing else can produce those two leading bits, so an answer without
	 * them means that no device is driving the bus.
	 */
	if ((answer & DRV8912_ANSWER_MARK) != DRV8912_ANSWER_MARK) {
		LOG_ERR("no answer from the device (0x%04x)", answer);
		return -EIO;
	}

	/*
	 * The six remaining bits of the answer are a fault summary. They are
	 * not reported yet, faults are read from the status registers instead.
	 */
	if (report != NULL) {
		*report = FIELD_GET(DRV8912_FRAME_DATA, answer);
	}

	return 0;
}

static int drv8912_write_reg(const struct device *dev, uint8_t addr, uint8_t value)
{
	uint16_t frame =
		FIELD_PREP(DRV8912_FRAME_ADDR, addr) | FIELD_PREP(DRV8912_FRAME_DATA, value);
	int ret;

	ret = drv8912_transfer(dev, frame, NULL);
	if (ret < 0) {
		return ret;
	}

	/* The device drops writes that follow the previous one too closely. */
	k_busy_wait(DRV8912_WRITE_GAP_US);

	return 0;
}

static int drv8912_read_reg(const struct device *dev, uint8_t addr, uint8_t *value)
{
	uint16_t frame = DRV8912_FRAME_READ | FIELD_PREP(DRV8912_FRAME_ADDR, addr);

	return drv8912_transfer(dev, frame, value);
}

/*
 * Build the value of one operation control register. The register holds four
 * half-bridges, two bits each, the low-side enable first and the high-side
 * enable second. Leaving both bits clear floats the output.
 */
static uint8_t drv8912_op_ctrl_value(gpio_port_pins_t driven, gpio_port_value_t high, uint8_t index)
{
	uint8_t value = 0;

	for (uint8_t i = 0; i < DRV8912_BRIDGES_PER_REG; i++) {
		uint8_t bridge = index * DRV8912_BRIDGES_PER_REG + i;

		if ((driven & BIT(bridge)) == 0) {
			continue;
		}

		if ((high & BIT(bridge)) != 0) {
			value |= BIT(2 * i + 1);
		} else {
			value |= BIT(2 * i);
		}
	}

	return value;
}

/*
 * Push the requested state to the device, skipping the registers that would
 * not change. The caller must hold the lock.
 *
 * A failed write leaves the logical state untouched while the registers
 * already written keep their new value. The next successful call notices the
 * difference and writes them back, so the device always ends up matching the
 * logical state.
 */
static int drv8912_apply(const struct device *dev, gpio_port_pins_t driven, gpio_port_value_t high)
{
	struct drv8912_data *data = dev->data;

	for (uint8_t i = 0; i < DRV8912_NUM_OP_CTRL_REGS; i++) {
		uint8_t value = drv8912_op_ctrl_value(driven, high, i);
		int ret;

		if (value == data->op_ctrl[i]) {
			continue;
		}

		ret = drv8912_write_reg(dev, DRV8912_REG_OP_CTRL_1 + i, value);
		if (ret < 0) {
			return ret;
		}

		data->op_ctrl[i] = value;
	}

	data->driven = driven;
	data->high = high;

	return 0;
}

static int drv8912_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct drv8912_data *data = dev->data;
	gpio_port_pins_t driven;
	gpio_port_value_t high;
	int ret;

	/* The outputs cannot be sensed, only driven. */
	if ((flags & GPIO_INPUT) != 0) {
		return -ENOTSUP;
	}

	/* The half-bridges are push-pull, and nothing biases the outputs. */
	if ((flags & (GPIO_SINGLE_ENDED | GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	driven = data->driven;
	high = data->high;

	if ((flags & GPIO_OUTPUT) == 0) {
		/* Neither an input nor an output: float the half-bridge. */
		driven &= ~BIT(pin);
	} else {
		driven |= BIT(pin);

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			high |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			high &= ~BIT(pin);
		}
	}

	ret = drv8912_apply(dev, driven, high);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int drv8912_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct drv8912_data *data = dev->data;

	/*
	 * The device offers no way to sense the outputs, so report the levels
	 * that were last asked for. A floating half-bridge reports the level it
	 * would take once driven again.
	 */
	k_mutex_lock(&data->lock, K_FOREVER);
	*value = data->high;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int drv8912_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				       gpio_port_value_t value)
{
	struct drv8912_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = drv8912_apply(dev, data->driven, (data->high & ~mask) | (value & mask));
	k_mutex_unlock(&data->lock);

	return ret;
}

static int drv8912_port_set_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return drv8912_port_set_masked_raw(dev, mask, mask);
}

static int drv8912_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return drv8912_port_set_masked_raw(dev, mask, 0);
}

static int drv8912_port_toggle_bits(const struct device *dev, gpio_port_pins_t mask)
{
	struct drv8912_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = drv8912_apply(dev, data->driven, data->high ^ mask);
	k_mutex_unlock(&data->lock);

	return ret;
}

static DEVICE_API(gpio, drv8912_api) = {
	.pin_configure = drv8912_pin_configure,
	.port_get_raw = drv8912_port_get_raw,
	.port_set_masked_raw = drv8912_port_set_masked_raw,
	.port_set_bits_raw = drv8912_port_set_bits_raw,
	.port_clear_bits_raw = drv8912_port_clear_bits_raw,
	.port_toggle_bits = drv8912_port_toggle_bits,
};

static int drv8912_init(const struct device *dev)
{
	const struct drv8912_config *config = dev->config;
	struct drv8912_data *data = dev->data;
	uint8_t status;
	int ret;

	k_mutex_init(&data->lock);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	if (config->nsleep.port != NULL) {
		if (!gpio_is_ready_dt(&config->nsleep)) {
			LOG_ERR("GPIO port %s not ready", config->nsleep.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->nsleep, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("unable to configure nSLEEP pin %u (%d)", config->nsleep.pin, ret);
			return ret;
		}

		k_busy_wait(DRV8912_WAKE_TIME_US);
	}

	if (config->nfault.port != NULL) {
		if (!gpio_is_ready_dt(&config->nfault)) {
			LOG_ERR("GPIO port %s not ready", config->nfault.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->nfault, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("unable to configure nFAULT pin %u (%d)", config->nfault.pin, ret);
			return ret;
		}
	}

	/*
	 * Resetting this SoC does not reset the device, which keeps driving
	 * whatever it was last asked to drive. Float every output before
	 * anything else, so that the loads are released and the shadow
	 * registers describe the device for sure.
	 */
	for (uint8_t i = 0; i < DRV8912_NUM_OP_CTRL_REGS; i++) {
		ret = drv8912_write_reg(dev, DRV8912_REG_OP_CTRL_1 + i, 0);
		if (ret < 0) {
			return ret;
		}

		data->op_ctrl[i] = 0;
	}

	/*
	 * Configure the device before acknowledging anything. A supply above
	 * the standard overvoltage threshold keeps every half-bridge disabled,
	 * so clearing that fault first would only see it latch again.
	 */
	ret = drv8912_write_reg(dev, DRV8912_REG_CONFIG_CTRL, config->config_ctrl);
	if (ret < 0) {
		return ret;
	}

	/* The acknowledgment bit clears itself, the configuration stays. */
	ret = drv8912_write_reg(dev, DRV8912_REG_CONFIG_CTRL,
				config->config_ctrl | DRV8912_CONFIG_CLR_FLT);
	if (ret < 0) {
		return ret;
	}

	ret = drv8912_read_reg(dev, DRV8912_REG_IC_STAT, &status);
	if (ret < 0) {
		return ret;
	}

	/* The power-on reset flag only reads back as one once acknowledged. */
	if ((status & DRV8912_IC_STAT_NPOR) == 0) {
		LOG_ERR("the device did not leave its power-on reset state");
		return -EIO;
	}

	if ((status & DRV8912_IC_STAT_FAULTS) != 0) {
		LOG_WRN("faults latched right after init, IC_STAT is 0x%02x", status);
	}

	if (config->nfault.port != NULL && gpio_pin_get_dt(&config->nfault) == 1) {
		LOG_WRN("nFAULT is asserted");
	}

	return 0;
}

/*
 * The clock idles low and the device samples on its falling edge, shifting its
 * own answer out on the rising one. In the terms the SPI API uses, that is a
 * clock polarity of zero and a clock phase of one, better known as SPI mode 1.
 */
#define DRV8912_SPI_OPERATION                                                                      \
	((uint16_t)(SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_WORD_SET(8)))

/*
 * The binding only accepts the two thresholds the device offers, so anything
 * above the standard one is asking for the extended protection.
 */
#define DRV8912_CONFIG_CTRL_INIT(inst)                                                             \
	(DT_INST_PROP(inst, overvoltage_protection_millivolt) > DRV8912_OVP_STANDARD_MV            \
		 ? DRV8912_CONFIG_EXT_OVP                                                          \
		 : 0)

#define DRV8912_INIT(inst)                                                                         \
	static struct drv8912_data drv8912_data_##inst;                                            \
                                                                                                   \
	static const struct drv8912_config drv8912_config_##inst = {                               \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.bus = SPI_DT_SPEC_INST_GET(inst, DRV8912_SPI_OPERATION),                          \
		.nsleep = GPIO_DT_SPEC_INST_GET_OR(inst, nsleep_gpios, {0}),                       \
		.nfault = GPIO_DT_SPEC_INST_GET_OR(inst, nfault_gpios, {0}),                       \
		.config_ctrl = DRV8912_CONFIG_CTRL_INIT(inst),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, drv8912_init, NULL, &drv8912_data_##inst,                      \
			      &drv8912_config_##inst, POST_KERNEL,                                 \
			      CONFIG_GPIO_DRV8912_INIT_PRIORITY, &drv8912_api);

DT_INST_FOREACH_STATUS_OKAY(DRV8912_INIT)
