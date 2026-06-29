/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 APMIXEDSYS PLL clock driver.
 * Ported from Linux clk-mt8188-apmixedsys.c + clk-pll.c.
 *
 * Supports all 15 PLLs defined in the Linux driver:
 *   ETHPLL, MSDCPLL, TVDPLL1/2, MMPLL, MAINPLL, IMGPLL, UNIVPLL,
 *   ADSPPLL, APLL1, APLL2, APLL3, APLL4, APLL5, MFGPLL
 *
 * PLL register layout (per PLL, offsets relative to APMIXEDSYS base):
 *   base_reg (CON0):  pll_en_bit = enable, en_mask = RST_BAR (if HAVE_RST_BAR)
 *   pwr_reg:          bit 0 = PWR_ON, bit 1 = ISO_EN (set=isolated, clear=active)
 *   pd_reg:           bits [pd_shift+2 : pd_shift] = post-divider select
 *   pcw_reg:          bits [pcwbits-1 : 0] = PCW fixed-point frequency word
 *   pcw_chg_reg:      bit 31 = PCW_CHG (trigger frequency update, self-clearing)
 *   tuner_en_reg:     bit tuner_en_bit = tuner enable (APLL1–5 only)
 *
 * enable sequence (from Linux mtk_pll_prepare in clk-pll.c):
 *   1. PWR_ON in pwr_reg,      udelay(1)
 *   2. clear ISO_EN in pwr_reg, udelay(1)
 *   3. set pll_en_bit in base_reg (CON0)
 *   4. set en_mask in CON0     (only PLLs with HAVE_RST_BAR)
 *   5. enable tuner            (only APLL1–5)
 *   6. udelay(20)              (wait for PLL lock)
 *   7. set rst_bar_mask in CON0 (only PLLs with HAVE_RST_BAR)
 *
 * disable sequence (from Linux mtk_pll_unprepare):
 *   1. clear rst_bar_mask      (HAVE_RST_BAR only)
 *   2. disable tuner           (APLL1–5 only)
 *   3. clear en_mask           (HAVE_RST_BAR only)
 *   4. clear pll_en_bit
 *   5. set ISO_EN
 *   6. clear PWR_ON
 */

#define DT_DRV_COMPAT mediatek_mt8390_apmixedsys

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

#include "clock_control_mtk_mt8390_apmixed.h"

/* -------------------------------------------------------------------------
 * PLL register constants (from clk-pll.c)
 * -------------------------------------------------------------------------
 */
#define CON0_PWR_ON    BIT(0)   /* pwr_reg: power on */
#define CON0_ISO_EN    BIT(1)   /* pwr_reg: isolation enable (set=isolated) */
#define PCW_CHG_MASK   BIT(31)  /* pcw_chg_reg: trigger PCW update */
#define PLL_EN_BIT_NUM 9        /* CON0 bit for PLL enable (all MT8188 PLLs) */
#define HAVE_RST_BAR   BIT(0)   /* flags: PLL has RST_BAR in CON0 */

/* -------------------------------------------------------------------------
 * PLL descriptor — mirrors Linux struct mtk_pll_data fields used here
 * -------------------------------------------------------------------------
 */
struct mtk_pll_data {
	uint32_t base_reg;      /* CON0 — pll_en_bit lives here */
	uint32_t pwr_reg;       /* power / isolation register */
	uint32_t pd_reg;        /* post-divider register */
	uint8_t  pd_shift;      /* post-divider field LSB in pd_reg */
	uint32_t pcw_reg;       /* PCW register */
	uint32_t pcw_chg_reg;   /* PCW change trigger register */
	uint32_t tuner_en_reg;  /* tuner enable register (0 = no tuner) */
	uint8_t  tuner_en_bit;  /* tuner enable bit in tuner_en_reg */
	uint32_t en_mask;       /* extra bits to set in CON0 at enable (RST_BAR) */
	uint32_t rst_bar_mask;  /* RST_BAR mask in CON0 */
	uint8_t  flags;         /* HAVE_RST_BAR */
	uint8_t  pcwbits;       /* total PCW field width (22 or 32) */
};

/*
 * PLL table — indexed by CLK_APMIXED_* (see mtk_mt8188_clock.h).
 * All register values from Linux clk-mt8188-apmixedsys.c plls[] table:
 *   PLL(id, name, reg, pwr_reg, en_mask, flags, rst_bar_mask, pcwbits,
 *       pd_reg, pd_shift, tuner_reg, tuner_en_reg, tuner_en_bit,
 *       pcw_reg, pcw_shift, pcw_chg_reg, en_reg, pll_en_bit)
 *
 * Mapping to our struct:
 *   base_reg      = reg  (= CON0)
 *   pwr_reg       = pwr_reg
 *   pd_reg        = pd_reg,  pd_shift = pd_shift
 *   pcw_reg       = pcw_reg
 *   pcw_chg_reg   = pcw_chg_reg  (0 → use pd_reg+4 = CON1, as in Linux)
 *   tuner_en_reg  = tuner_en_reg, tuner_en_bit = tuner_en_bit
 *   en_mask       = en_mask
 *   rst_bar_mask  = rst_bar_mask
 *   flags         = HAVE_RST_BAR if flags has it
 *
 * For PLLs where pcw_chg_reg=0 in Linux, Linux falls back to pll->pcw_chg_addr
 * = base_addr + REG_CON1 = base_reg + 4. We replicate that below.
 */
static const struct mtk_pll_data pll_table[] = {
	/*
	 * CLK_APMIXED_ETHPLL = 0
	 * PLL(0, "ethpll", 0x044C, 0x0458, 0,
	 *     0, 0, 22, 0x0450, 24, 0, 0, 0, 0x0450, 0, 0, 0, 9)
	 * pcw_chg_reg=0 → CON1 = 0x044C + 4 = 0x0450 (same as pd_reg, bit31)
	 */
	[0] = {
		.base_reg     = 0x044C,
		.pwr_reg      = 0x0458,
		.pd_reg       = 0x0450,
		.pd_shift     = 24,
		.pcw_reg      = 0x0450,
		.pcw_chg_reg  = 0x0450,  /* CON1 */
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_MSDCPLL = 1
	 * PLL(1, "msdcpll", 0x0514, 0x0520, 0,
	 *     0, 0, 22, 0x0518, 24, 0, 0, 0, 0x0518, 0, 0, 0, 9)
	 */
	[1] = {
		.base_reg     = 0x0514,
		.pwr_reg      = 0x0520,
		.pd_reg       = 0x0518,
		.pd_shift     = 24,
		.pcw_reg      = 0x0518,
		.pcw_chg_reg  = 0x0518,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_TVDPLL1 = 2
	 * PLL(2, "tvdpll1", 0x0524, 0x0530, 0,
	 *     0, 0, 22, 0x0528, 24, 0, 0, 0, 0x0528, 0, 0, 0, 9)
	 */
	[2] = {
		.base_reg     = 0x0524,
		.pwr_reg      = 0x0530,
		.pd_reg       = 0x0528,
		.pd_shift     = 24,
		.pcw_reg      = 0x0528,
		.pcw_chg_reg  = 0x0528,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_TVDPLL2 = 3
	 * PLL(3, "tvdpll2", 0x0534, 0x0540, 0,
	 *     0, 0, 22, 0x0538, 24, 0, 0, 0, 0x0538, 0, 0, 0, 9)
	 */
	[3] = {
		.base_reg     = 0x0534,
		.pwr_reg      = 0x0540,
		.pd_reg       = 0x0538,
		.pd_shift     = 24,
		.pcw_reg      = 0x0538,
		.pcw_chg_reg  = 0x0538,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_MMPLL = 4
	 * PLL(4, "mmpll", 0x0544, 0x0550, 0xff000000,
	 *     HAVE_RST_BAR, BIT(23), 22, 0x0548, 24, 0, 0, 0, 0x0548, 0, 0, 0, 9)
	 */
	[4] = {
		.base_reg     = 0x0544,
		.pwr_reg      = 0x0550,
		.en_mask      = 0xff000000,
		.flags        = HAVE_RST_BAR,
		.rst_bar_mask = BIT(23),
		.pd_reg       = 0x0548,
		.pd_shift     = 24,
		.pcw_reg      = 0x0548,
		.pcw_chg_reg  = 0x0548,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_MAINPLL = 5
	 * PLL(5, "mainpll", 0x045C, 0x0468, 0xff000000,
	 *     HAVE_RST_BAR, BIT(23), 22, 0x0460, 24, 0, 0, 0, 0x0460, 0, 0, 0, 9)
	 */
	[5] = {
		.base_reg     = 0x045C,
		.pwr_reg      = 0x0468,
		.en_mask      = 0xff000000,
		.flags        = HAVE_RST_BAR,
		.rst_bar_mask = BIT(23),
		.pd_reg       = 0x0460,
		.pd_shift     = 24,
		.pcw_reg      = 0x0460,
		.pcw_chg_reg  = 0x0460,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_IMGPLL = 6
	 * PLL(6, "imgpll", 0x0554, 0x0560, 0,
	 *     0, 0, 22, 0x0558, 24, 0, 0, 0, 0x0558, 0, 0, 0, 9)
	 */
	[6] = {
		.base_reg     = 0x0554,
		.pwr_reg      = 0x0560,
		.pd_reg       = 0x0558,
		.pd_shift     = 24,
		.pcw_reg      = 0x0558,
		.pcw_chg_reg  = 0x0558,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_UNIVPLL = 7
	 * PLL(7, "univpll", 0x0504, 0x0510, 0xff000000,
	 *     HAVE_RST_BAR, BIT(23), 22, 0x0508, 24, 0, 0, 0, 0x0508, 0, 0, 0, 9)
	 */
	[7] = {
		.base_reg     = 0x0504,
		.pwr_reg      = 0x0510,
		.en_mask      = 0xff000000,
		.flags        = HAVE_RST_BAR,
		.rst_bar_mask = BIT(23),
		.pd_reg       = 0x0508,
		.pd_shift     = 24,
		.pcw_reg      = 0x0508,
		.pcw_chg_reg  = 0x0508,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_ADSPPLL = 8
	 * PLL(8, "adsppll", 0x042C, 0x0438, 0,
	 *     0, 0, 22, 0x0430, 24, 0, 0, 0, 0x0430, 0, 0, 0, 9)
	 */
	[8] = {
		.base_reg     = 0x042C,
		.pwr_reg      = 0x0438,
		.pd_reg       = 0x0430,
		.pd_shift     = 24,
		.pcw_reg      = 0x0430,
		.pcw_chg_reg  = 0x0430,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/*
	 * CLK_APMIXED_APLL1 = 9
	 * PLL(9, "apll1", 0x0304, 0x0314, 0,
	 *     0, 0, 32, 0x0308, 24, 0x0034, 0x0000, 12, 0x030C, 0, 0, 0, 9)
	 * tuner_en_reg=0x0000 (CON0 of APMIXEDSYS base), tuner_en_bit=12
	 * pcw_chg_reg=0 → CON1 = 0x0304 + 4 = 0x0308
	 */
	[9] = {
		.base_reg     = 0x0304,
		.pwr_reg      = 0x0314,
		.pd_reg       = 0x0308,
		.pd_shift     = 24,
		.pcw_reg      = 0x030C,
		.pcw_chg_reg  = 0x0308,  /* CON1 */
		.tuner_en_reg = 0x0000,  /* APMIXEDSYS base + 0 = AP_PLL_CON0 */
		.tuner_en_bit = 12,
		.pcwbits      = 32,
	},
	/*
	 * CLK_APMIXED_APLL2 = 10
	 * PLL(10, "apll2", 0x0318, 0x0328, 0,
	 *     0, 0, 32, 0x031C, 24, 0x0038, 0x0000, 13, 0x0320, 0, 0, 0, 9)
	 */
	[10] = {
		.base_reg     = 0x0318,
		.pwr_reg      = 0x0328,
		.pd_reg       = 0x031C,
		.pd_shift     = 24,
		.pcw_reg      = 0x0320,
		.pcw_chg_reg  = 0x031C,  /* CON1 */
		.tuner_en_reg = 0x0000,
		.tuner_en_bit = 13,
		.pcwbits      = 32,
	},
	/*
	 * CLK_APMIXED_APLL3 = 11
	 * PLL(11, "apll3", 0x032C, 0x033C, 0,
	 *     0, 0, 32, 0x0330, 24, 0x003C, 0x0000, 14, 0x0334, 0, 0, 0, 9)
	 */
	[11] = {
		.base_reg     = 0x032C,
		.pwr_reg      = 0x033C,
		.pd_reg       = 0x0330,
		.pd_shift     = 24,
		.pcw_reg      = 0x0334,
		.pcw_chg_reg  = 0x0330,  /* CON1 */
		.tuner_en_reg = 0x0000,
		.tuner_en_bit = 14,
		.pcwbits      = 32,
	},
	/*
	 * CLK_APMIXED_APLL4 = 12
	 * PLL(12, "apll4", 0x0404, 0x0414, 0,
	 *     0, 0, 32, 0x0408, 24, 0x0040, 0x0000, 15, 0x040C, 0, 0, 0, 9)
	 */
	[12] = {
		.base_reg     = 0x0404,
		.pwr_reg      = 0x0414,
		.pd_reg       = 0x0408,
		.pd_shift     = 24,
		.pcw_reg      = 0x040C,
		.pcw_chg_reg  = 0x0408,  /* CON1 */
		.tuner_en_reg = 0x0000,
		.tuner_en_bit = 15,
		.pcwbits      = 32,
	},
	/*
	 * CLK_APMIXED_APLL5 = 13
	 * PLL(13, "apll5", 0x0418, 0x0428, 0,
	 *     0, 0, 32, 0x041C, 24, 0x0044, 0x0000, 16, 0x0420, 0, 0, 0, 9)
	 */
	[13] = {
		.base_reg     = 0x0418,
		.pwr_reg      = 0x0428,
		.pd_reg       = 0x041C,
		.pd_shift     = 24,
		.pcw_reg      = 0x0420,
		.pcw_chg_reg  = 0x041C,  /* CON1 */
		.tuner_en_reg = 0x0000,
		.tuner_en_bit = 16,
		.pcwbits      = 32,
	},
	/*
	 * CLK_APMIXED_MFGPLL = 14
	 * PLL(14, "mfgpll", 0x0340, 0x034C, 0,
	 *     0, 0, 22, 0x0344, 24, 0, 0, 0, 0x0344, 0, 0, 0, 9)
	 */
	[14] = {
		.base_reg     = 0x0340,
		.pwr_reg      = 0x034C,
		.pd_reg       = 0x0344,
		.pd_shift     = 24,
		.pcw_reg      = 0x0344,
		.pcw_chg_reg  = 0x0344,
		.tuner_en_reg = 0,
		.pcwbits      = 22,
	},
	/* CLK_APMIXED_PLL_SSUSB26M_EN = 15 — gate clock, not a PLL; unsupported */
};

/* -------------------------------------------------------------------------
 * Device structs
 * -------------------------------------------------------------------------
 */
struct mtk_apmixed_data {
	DEVICE_MMIO_RAM;
};

struct mtk_apmixed_config {
	DEVICE_MMIO_ROM;
};

/* -------------------------------------------------------------------------
 * Tuner helpers
 * Linux: __mtk_pll_tuner_enable / __mtk_pll_tuner_disable
 * APLL tuner_en_reg = 0x0000 = AP_PLL_CON0 relative to APMIXEDSYS base.
 * -------------------------------------------------------------------------
 */
/*
 * tuner_en_bit == 0 means "no tuner" — non-audio PLLs have tuner_en_reg=0
 * and tuner_en_bit=0 in their table entries.
 * APLL1–5 have tuner_en_reg=0x0000 (AP_PLL_CON0) with bits 12–16, so we
 * distinguish them by tuner_en_bit != 0.
 */
static void pll_tuner_enable(uintptr_t base, const struct mtk_pll_data *d)
{
	if (d->tuner_en_bit == 0) {
		return;
	}
	sys_write32(sys_read32(base + d->tuner_en_reg) | BIT(d->tuner_en_bit),
		    base + d->tuner_en_reg);
}

static void pll_tuner_disable(uintptr_t base, const struct mtk_pll_data *d)
{
	if (d->tuner_en_bit == 0) {
		return;
	}
	sys_write32(sys_read32(base + d->tuner_en_reg) & ~BIT(d->tuner_en_bit),
		    base + d->tuner_en_reg);
}

/* -------------------------------------------------------------------------
 * clock_on — Linux mtk_pll_prepare()
 * -------------------------------------------------------------------------
 */
static int mtk_apmixed_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_pll_data *d;
	uint32_t r;

	if (idx >= ARRAY_SIZE(pll_table)) {
		return -EINVAL;
	}
	d = &pll_table[idx];
	if (d->base_reg == 0) {
		return -ENOTSUP;
	}

	/* Step 1: Power on, wait 1 µs */
	sys_write32(sys_read32(base + d->pwr_reg) | CON0_PWR_ON,
		    base + d->pwr_reg);
	k_busy_wait(1);

	/* Step 2: De-isolate, wait 1 µs */
	sys_write32(sys_read32(base + d->pwr_reg) & ~CON0_ISO_EN,
		    base + d->pwr_reg);
	k_busy_wait(1);

	/* Step 3: Enable PLL (pll_en_bit = 9 for all MT8188 PLLs) */
	sys_write32(sys_read32(base + d->base_reg) | BIT(PLL_EN_BIT_NUM),
		    base + d->base_reg);

	/* Step 4: Set en_mask (RST_BAR PLLs: MMPLL, MAINPLL, UNIVPLL) */
	if (d->en_mask) {
		r = sys_read32(base + d->base_reg) | d->en_mask;
		sys_write32(r, base + d->base_reg);
	}

	/* Step 5: Enable tuner (APLL1–5) */
	pll_tuner_enable(base, d);

	/* Step 6: Wait for PLL lock (~20 µs) */
	k_busy_wait(20);

	/* Step 7: Set RST_BAR (HAVE_RST_BAR PLLs) */
	if (d->flags & HAVE_RST_BAR) {
		r = sys_read32(base + d->base_reg) | d->rst_bar_mask;
		sys_write32(r, base + d->base_reg);
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * clock_off — Linux mtk_pll_unprepare()
 * -------------------------------------------------------------------------
 */
static int mtk_apmixed_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_pll_data *d;
	uint32_t r;

	if (idx >= ARRAY_SIZE(pll_table)) {
		return -EINVAL;
	}
	d = &pll_table[idx];
	if (d->base_reg == 0) {
		return -ENOTSUP;
	}

	/* Step 1: Clear RST_BAR */
	if (d->flags & HAVE_RST_BAR) {
		r = sys_read32(base + d->base_reg) & ~d->rst_bar_mask;
		sys_write32(r, base + d->base_reg);
	}

	/* Step 2: Disable tuner */
	pll_tuner_disable(base, d);

	/* Step 3: Clear en_mask */
	if (d->en_mask) {
		r = sys_read32(base + d->base_reg) & ~d->en_mask;
		sys_write32(r, base + d->base_reg);
	}

	/* Step 4: Disable PLL */
	sys_write32(sys_read32(base + d->base_reg) & ~BIT(PLL_EN_BIT_NUM),
		    base + d->base_reg);

	/* Step 5: Isolate */
	sys_write32(sys_read32(base + d->pwr_reg) | CON0_ISO_EN,
		    base + d->pwr_reg);

	/* Step 6: Power off */
	sys_write32(sys_read32(base + d->pwr_reg) & ~CON0_PWR_ON,
		    base + d->pwr_reg);

	return 0;
}

/* -------------------------------------------------------------------------
 * get_status — read CON0 pll_en_bit
 * -------------------------------------------------------------------------
 */
static enum clock_control_status mtk_apmixed_clock_status(const struct device *dev,
							   clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_pll_data *d;

	if (idx >= ARRAY_SIZE(pll_table)) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	d = &pll_table[idx];
	if (d->base_reg == 0) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return (sys_read32(base + d->base_reg) & BIT(PLL_EN_BIT_NUM))
		? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int mtk_apmixed_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(clock_control, mtk_apmixed_api) = {
	.on         = mtk_apmixed_clock_on,
	.off        = mtk_apmixed_clock_off,
	.get_status = mtk_apmixed_clock_status,
};

static struct mtk_apmixed_data apmixed_data;

static const struct mtk_apmixed_config apmixed_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
};

DEVICE_DT_INST_DEFINE(0, &mtk_apmixed_init, NULL,
		      &apmixed_data, &apmixed_cfg,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &mtk_apmixed_api);
