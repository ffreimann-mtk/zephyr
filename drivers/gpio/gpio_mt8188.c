/*
 * Copyright (c) 2025 MediaTek
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mediatek_mt8188_gpio

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include "gpio_mtk_common.h"

/* Register offsets in order of offset values. */
#define GPIO_OFFSET_DIR_0      0x0000
#define GPIO_OFFSET_DIR_0_SET  0x0004
#define GPIO_OFFSET_DIR_0_CLR  0x0008
#define GPIO_OFFSET_DIR_1      0x0010
#define GPIO_OFFSET_DIR_1_SET  0x0014
#define GPIO_OFFSET_DIR_1_CLR  0x0018
#define GPIO_OFFSET_DIR_2      0x0020
#define GPIO_OFFSET_DIR_2_SET  0x0024
#define GPIO_OFFSET_DIR_2_CLR  0x0028
#define GPIO_OFFSET_DIR_3      0x0030
#define GPIO_OFFSET_DIR_3_SET  0x0034
#define GPIO_OFFSET_DIR_3_CLR  0x0038
#define GPIO_OFFSET_DIR_4      0x0040
#define GPIO_OFFSET_DIR_4_SET  0x0044
#define GPIO_OFFSET_DIR_4_CLR  0x0048
#define GPIO_OFFSET_DIR_5      0x0050
#define GPIO_OFFSET_DIR_5_SET  0x0054
#define GPIO_OFFSET_DIR_5_CLR  0x0058
#define GPIO_OFFSET_DOUT_0     0x0100
#define GPIO_OFFSET_DOUT_0_SET 0x0104
#define GPIO_OFFSET_DOUT_0_CLR 0x0108
#define GPIO_OFFSET_DOUT_1     0x0110
#define GPIO_OFFSET_DOUT_1_SET 0x0114
#define GPIO_OFFSET_DOUT_1_CLR 0x0118
#define GPIO_OFFSET_DOUT_2     0x0120
#define GPIO_OFFSET_DOUT_2_SET 0x0124
#define GPIO_OFFSET_DOUT_2_CLR 0x0128
#define GPIO_OFFSET_DOUT_3     0x0130
#define GPIO_OFFSET_DOUT_3_SET 0x0134
#define GPIO_OFFSET_DOUT_3_CLR 0x0138
#define GPIO_OFFSET_DOUT_4     0x0140
#define GPIO_OFFSET_DOUT_4_SET 0x0144
#define GPIO_OFFSET_DOUT_4_CLR 0x0148
#define GPIO_OFFSET_DOUT_5     0x0150
#define GPIO_OFFSET_DOUT_5_SET 0x0154
#define GPIO_OFFSET_DOUT_5_CLR 0x0158
#define GPIO_OFFSET_DIN_0      0x0200
#define GPIO_OFFSET_DIN_1      0x0210
#define GPIO_OFFSET_DIN_2      0x0220
#define GPIO_OFFSET_DIN_3      0x0230
#define GPIO_OFFSET_DIN_4      0x0240
#define GPIO_OFFSET_DIN_5      0x0250

#define GPIO_OFFSET_DIN_DELTA      (GPIO_OFFSET_DIN_1 - GPIO_OFFSET_DIN_0)
#define GPIO_OFFSET_DOUT_SET_DELTA (GPIO_OFFSET_DOUT_1_SET - GPIO_OFFSET_DOUT_0_SET)
#define GPIO_OFFSET_DOUT_CLR_DELTA (GPIO_OFFSET_DOUT_1_CLR - GPIO_OFFSET_DOUT_0_CLR)
#define GPIO_OFFSET_DIR_SET_DELTA  (GPIO_OFFSET_DIR_1_SET - GPIO_OFFSET_DIR_0_SET)
#define GPIO_OFFSET_DIR_CLR_DELTA  (GPIO_OFFSET_DIR_1_CLR - GPIO_OFFSET_DIR_0_CLR)

static uint32_t reg_offset(const struct device *dev, reg_type_t reg_type);

static uint32_t reg_offset(const struct device *dev, reg_type_t reg_type)
{
	const gpio_mtk_config_t *gpio_config = dev->config;

	switch (reg_type) {
	case REG_TYPE_DIN:
		return (uint32_t)(GPIO_OFFSET_DIN_0 + (gpio_config->idx * GPIO_OFFSET_DIN_DELTA));
		break;

	case REG_TYPE_DOUT_SET:
		return (uint32_t)(GPIO_OFFSET_DOUT_0_SET +
				  (gpio_config->idx * GPIO_OFFSET_DOUT_SET_DELTA));
		break;

	case REG_TYPE_DOUT_CLR:
		return (uint32_t)(GPIO_OFFSET_DOUT_0_CLR +
				  (gpio_config->idx * GPIO_OFFSET_DOUT_CLR_DELTA));
		break;

	case REG_TYPE_DIR_SET:
		return (uint32_t)(GPIO_OFFSET_DIR_0_SET +
				  (gpio_config->idx * GPIO_OFFSET_DIR_SET_DELTA));
		break;

	case REG_TYPE_DIR_CLR:
		return (uint32_t)(GPIO_OFFSET_DIR_0_CLR +
				  (gpio_config->idx * GPIO_OFFSET_DIR_CLR_DELTA));
		break;

	default:
		break;
	}

	return INV_REG_OFFSET;
}

static DEVICE_API(gpio, gpio_mtk_driver_api) = {
	.pin_configure = gpio_mtk_pin_configure,
	.port_get_raw = gpio_mtk_port_get_raw,
	.port_set_masked_raw = gpio_mtk_port_set_masked_raw,
	.port_set_bits_raw = gpio_mtk_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mtk_port_clr_bits_raw,
	.port_toggle_bits = gpio_mtk_port_toggle_bits,
	.pin_interrupt_configure = gpio_mtk_pin_interrupt_configure,
	.manage_callback = gpio_mtk_manage_callback,
};

#define GPIO_DECLARE_CFG(n)                                                                        \
	static gpio_mtk_data_t gpio_mtk_##n##_data;                                                \
                                                                                                   \
	static const gpio_mtk_config_t gpio_mtk_##n##_config = {                                   \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},                   \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_INST_PARENT(n)),                           \
		.eint_dev = &DEVICE_DT_NAME_GET(DT_INST_PROP(n, interrupt_parent)),                \
		.idx = DT_INST_REG_ADDR(n),                                                        \
		.num_gpio_pins = DT_INST_PROP(n, ngpios),                                          \
		.gpio_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_PROP(n, ngpios)),          \
		.reg_offset = reg_offset,                                                          \
	};

#define GPIO_INIT(n)                                                                               \
	GPIO_DECLARE_CFG(n)                                                                        \
	DEVICE_DT_INST_DEFINE(n, &gpio_mtk_init, NULL, &gpio_mtk_##n##_data,                       \
			      &gpio_mtk_##n##_config, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &gpio_mtk_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INIT)
