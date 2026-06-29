/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 capture sample: UL8 ← eTDM_IN1, 16 channel, 48 kHz, 32-bit.
 * Prints one sample per channel per second.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "mt8390-afe.h"

#define SAMPLE_RATE     48000
#define CHANNELS        16
#define WORD_BYTES      4   /* 32-bit */
#define BUF_MS          100

#define BUF_FRAMES      (SAMPLE_RATE * BUF_MS / 1000)
#define BUF_SAMPLES     (BUF_FRAMES * CHANNELS)
#define BUF_BYTES       (BUF_SAMPLES * WORD_BYTES)

static int32_t dma_buf[BUF_SAMPLES] __aligned(64);

int main(void)
{
	const struct device *dev;
	const struct mt8390_afe_driver_api *api;
	struct mt8390_afe_cfg cfg = {
		.rate          = SAMPLE_RATE,
		.channels      = CHANNELS,
		.word_size     = 32,
		.fmt           = MT8390_ETDM_FMT_DSPB,
		.data_mode     = MT8390_ETDM_DATA_ONE_PIN,
		.mclk_freq     = 256 * SAMPLE_RATE,   /* 12.288 MHz, codec MCLK */
		.mclk_dir      = MT8390_MCLK_DIR_OUT,
		.period_frames = SAMPLE_RATE / 100, /* 10 ms period */
	};
	int ret;

	dev = DEVICE_DT_GET(DT_NODELABEL(afe));
	if (!device_is_ready(dev)) {
		printk("AFE device not ready\n");
		return -ENODEV;
	}
	api = dev->api;

	ret = api->configure(dev, MT8390_UL8, &cfg);
	if (ret) {
		printk("configure failed: %d\n", ret);
		return ret;
	}

	ret = api->route(dev, MT8390_ROUTE_SRC_ETDM_IN1, MT8390_ROUTE_DST_UL8);
	if (ret) {
		printk("route failed: %d\n", ret);
		return ret;
	}

	/* Hardware requires physical addresses */
	uintptr_t buf_phys = k_mem_phys_addr(dma_buf);

	ret = api->set_buf(dev, MT8390_UL8,
			   (uint32_t)buf_phys, BUF_BYTES);
	if (ret) {
		printk("set_buf failed: %d\n", ret);
		return ret;
	}

	ret = api->start(dev, MT8390_UL8);
	if (ret) {
		printk("start failed: %d\n", ret);
		return ret;
	}

	printk("MT8390 UL8 capture started: 48kHz 16ch 32-bit from eTDM_IN1\n");

	while (1) {
		k_msleep(1000);

		/* get_cur returns a physical address */
		uint32_t ptr = api->get_cur(dev, MT8390_UL8);
		uint32_t offset = (ptr > (uint32_t)buf_phys) ?
				  (ptr - (uint32_t)buf_phys) : 0;
		uint32_t frame = (offset / (CHANNELS * WORD_BYTES));

		/* Print one sample per channel from that frame */
		uint32_t sample_idx = (frame > 0 ? frame - 1 : 0) * CHANNELS;

		for (int ch = 0; ch < CHANNELS; ch++) {
			printk("UL8 ch%d: 0x%08x  ", ch,
			       (uint32_t)dma_buf[sample_idx + ch]);
		}
		printk("\n");
	}

	return 0;
}
