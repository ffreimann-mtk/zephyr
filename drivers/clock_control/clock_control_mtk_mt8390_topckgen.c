/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 TOPCKGEN mux+gate clock driver.
 * Implements only the audio-relevant CLK_TOP_* clocks. All other IDs
 * return -ENOTSUP.
 *
 * Register data ported from Linux clk-mt8188-topckgen.c.
 * MUX_GATE_CLR_SET_UPD(id, name, parents, sta_ofs, clr_ofs, set_ofs,
 *                       shift, width, gate_bit, upd_ofs, upd_bit)
 *
 * In the Linux driver, gate enable = write 0 to gate_bit in clr_reg,
 * gate disable = write 1 to gate_bit in clr_reg (inverted gate).
 * Here we map: clock_on → clear gate bit (enable), clock_off → set it (disable).
 */

#define DT_DRV_COMPAT mediatek_mt8390_topckgen

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

#include "clock_control_mtk_topckgen_common.h"

/* CLK_TOP_* IDs are defined in mtk_mt8188_clock.h — include for the values */
#include <zephyr/dt-bindings/clock/mtk_mt8188_clock.h>

/*
 * mux_val: parent index to select for audio operation.
 * For i2so1/i2so2/i2si1/i2si2: parent index 1 = apll1 (48kHz family)
 * For a1sys_hp: parent index 1 = apll1_d4
 * For a2sys:    parent index 1 = apll2_d4
 * For aud_intbus: parent index 1 = mainpll_d4_d4
 * For audio_h:    parent index 2 = apll1
 * For apll1/apll2 mux: parent index 1 = apll1_d4 / apll2_d4
 * For asm_h/asm_l: parent index 3 = mainpll_d5_d2
 * For audio_local_bus: parent index 2 = mainpll_d4_d4
 * For aud_iec: parent index 1 = apll1
 * For a3sys/a4sys: parent index 1 = apll3_d4 (deferred — stubs)
 */

/*
 * MUX_GATE entry (standard mux+gate in same register word)
 * gate is inverted: bit set in sta_reg = gate DISABLED.
 * clock_on:  write bit to mux_clr_reg (clears gate-disable bit → enables clock)
 * clock_off: write bit to mux_set_reg (sets gate-disable bit → disables clock)
 *
 * Linux macro: MUX_GATE_CLR_SET_UPD(id, name, parents,
 *   mux_ofs, SET_ofs, CLR_ofs, shift, width, gate_bit, upd_ofs, upd_bit)
 * Note: Linux arg order is (sta, SET, CLR) — SET before CLR.
 * The topckgen table calls look like (0x074, 0x078, 0x07C, ...) where
 *   0x078 = SET register, 0x07C = CLR register.
 *
 * Our macro parameter names match the struct fields:
 *   _sta = sta/read, _set = set register, _clr = clr register
 */
#define TOPCK_MUX_GATE(_sta, _set, _clr, _shift, _width, _gbit, _mval, _ureg, _ubit) \
	{ .mux_sta_reg  = (_sta),                                                      \
	  .mux_clr_reg  = (_clr),                                                      \
	  .mux_set_reg  = (_set),                                                      \
	  .mux_shift    = (_shift),                                                    \
	  .mux_width    = (_width),                                                    \
	  .mux_val      = (_mval),                                                     \
	  .gate_bit     = (_gbit),                                                     \
	  .dynamic_parent = false,                                                     \
	  .upd_reg      = (_ureg),                                                     \
	  .upd_bit      = (_ubit),                                                     \
	  .div_gate_reg = 0,                                                           \
	  .div_gate_bit = 0 }

/*
 * Like TOPCK_MUX_GATE but the parent is selected at runtime via the .configure
 * callback (not by clock_on). Used for clocks whose parent varies by sample
 * rate: A1SYS_HP (apll1_d4) and the I2SIx/I2SOx MCLK muxes. clock_on enables
 * only the gate, leaving the configured parent untouched. _mval is the
 * power-on default parent (clk26m) for documentation; it is not written.
 */
#define TOPCK_MUX_GATE_DYN(_sta, _set, _clr, _shift, _width, _gbit, _mval, _ureg, _ubit) \
	{ .mux_sta_reg  = (_sta),                                                      \
	  .mux_clr_reg  = (_clr),                                                      \
	  .mux_set_reg  = (_set),                                                      \
	  .mux_shift    = (_shift),                                                    \
	  .mux_width    = (_width),                                                    \
	  .mux_val      = (_mval),                                                     \
	  .gate_bit     = (_gbit),                                                     \
	  .dynamic_parent = true,                                                      \
	  .upd_reg      = (_ureg),                                                     \
	  .upd_bit      = (_ubit),                                                     \
	  .div_gate_reg = 0,                                                           \
	  .div_gate_bit = 0 }

/*
 * DIV_GATE entry: gate is in a separate register (div_gate_reg/bit).
 * No mux to configure (mux_width = 0).
 */
#define TOPCK_DIV_GATE(_dreg, _dbit) \
	{ .mux_sta_reg  = 0,          \
	  .mux_clr_reg  = 0,          \
	  .mux_set_reg  = 0,          \
	  .mux_shift    = 0,          \
	  .mux_width    = 0,          \
	  .mux_val      = 0,          \
	  .gate_bit     = 0,          \
	  .div_gate_reg = (_dreg),    \
	  .div_gate_bit = (_dbit) }

/*
 * Clock table indexed by CLK_TOP_* ID (0 .. CLK_TOP_NR_CLK-1 = 204).
 * All-zero entries (default C zero-initialization) are treated as stubs
 * and return -ENOTSUP.  Only audio-relevant clocks are populated.
 *
 * Register values from Linux clk-mt8188-topckgen.c
 * MUX_GATE_CLR_SET_UPD args: (sta_ofs, clr_ofs, set_ofs, shift, width, gate_bit, ...)
 */
/*
 * upd_reg/upd_bit values from Linux clk-mt8188-topckgen.c top_mtk_muxes[]:
 *   MUX_GATE_CLR_SET_UPD(id, name, parents, sta, clr, set,
 *                        shift, width, gate_bit, upd_ofs, upd_bit)
 */
static const struct mtk_topck_clk topck_clk_table[CLK_TOP_NR_CLK] = {
	/* CLK_TOP_AUD_INTBUS = 31 — upd_ofs=0x04 upd_bit=31 */
	[CLK_TOP_AUD_INTBUS] = TOPCK_MUX_GATE(0x074, 0x078, 0x07C, 24, 4, 31, 1, 0x04, 31),

	/* CLK_TOP_AUDIO_H = 32 — upd_ofs=0x08 upd_bit=0 */
	[CLK_TOP_AUDIO_H] = TOPCK_MUX_GATE(0x080, 0x084, 0x088, 0, 4, 7, 2, 0x08, 0),

	/* CLK_TOP_AUDIO_LOCAL_BUS = 69 — upd_ofs=0x0C upd_bit=5 */
	/* set default parent clock to mainpll_d5_d2 — 4 */
	[CLK_TOP_AUDIO_LOCAL_BUS] = TOPCK_MUX_GATE(0x0EC, 0x0F0, 0x0F4, 8, 4, 15, 4, 0x0C, 5),

	/* CLK_TOP_ASM_H = 70 — upd_ofs=0x0C upd_bit=6 */
	[CLK_TOP_ASM_H] = TOPCK_MUX_GATE(0x0EC, 0x0F0, 0x0F4, 16, 4, 23, 3, 0x0C, 6),

	/* CLK_TOP_ASM_L = 71 — upd_ofs=0x0C upd_bit=7 */
	[CLK_TOP_ASM_L] = TOPCK_MUX_GATE(0x0EC, 0x0F0, 0x0F4, 24, 4, 31, 3, 0x0C, 7),

	/* CLK_TOP_APLL1 = 72 — upd_ofs=0x0C upd_bit=8 */
	[CLK_TOP_APLL1] = TOPCK_MUX_GATE(0x0F8, 0x0FC, 0x100, 0, 4, 7, 1, 0x0C, 8),

	/* CLK_TOP_APLL2 = 73 — upd_ofs=0x0C upd_bit=9 */
	[CLK_TOP_APLL2] = TOPCK_MUX_GATE(0x0F8, 0x0FC, 0x100, 8, 4, 15, 1, 0x0C, 9),

	/* CLK_TOP_I2SO1 = 77 — upd_ofs=0x0C upd_bit=13 */
	[CLK_TOP_I2SO1] = TOPCK_MUX_GATE_DYN(0x104, 0x108, 0x10C, 8, 4, 15, 1, 0x0C, 13),

	/* CLK_TOP_I2SO2 = 78 — upd_ofs=0x0C upd_bit=14 */
	[CLK_TOP_I2SO2] = TOPCK_MUX_GATE_DYN(0x104, 0x108, 0x10C, 16, 4, 23, 1, 0x0C, 14),

	/* CLK_TOP_I2SI1 = 79 — upd_ofs=0x0C upd_bit=15 */
	[CLK_TOP_I2SI1] = TOPCK_MUX_GATE_DYN(0x104, 0x108, 0x10C, 24, 4, 31, 1, 0x0C, 15),

	/* CLK_TOP_I2SI2 = 80 — upd_ofs=0x0C upd_bit=16 */
	[CLK_TOP_I2SI2] = TOPCK_MUX_GATE_DYN(0x110, 0x114, 0x118, 0, 4, 7, 1, 0x0C, 16),

	/* CLK_TOP_AUD_IEC = 82 — upd_ofs=0x0C upd_bit=18 */
	[CLK_TOP_AUD_IEC] = TOPCK_MUX_GATE(0x110, 0x114, 0x118, 16, 4, 23, 1, 0x0C, 18),

	/* CLK_TOP_A1SYS_HP = 83 — upd_ofs=0x0C upd_bit=19 */
	/* Dynamic parent: clk26m (0) at power-on, apll1_d4 (1) while APLL1 active.
	 * Parent set via .configure (mt8390_apll1_enable); clock_on gate-only.
	 */
	[CLK_TOP_A1SYS_HP] = TOPCK_MUX_GATE_DYN(0x110, 0x114, 0x118, 24, 4, 31, 0, 0x0C, 19),

	/* CLK_TOP_A2SYS = 84 — upd_ofs=0x0C upd_bit=20 */
	[CLK_TOP_A2SYS] = TOPCK_MUX_GATE(0x11C, 0x120, 0x124, 0, 4, 7, 1, 0x0C, 20),

	/* CLK_TOP_APLL12_CK_DIV0 = 179 — gate@0x320 bit 0 */
	[CLK_TOP_APLL12_CK_DIV0] = TOPCK_DIV_GATE(0x320, 0),

	/* CLK_TOP_APLL12_CK_DIV1 = 180 — gate@0x320 bit 1 */
	[CLK_TOP_APLL12_CK_DIV1] = TOPCK_DIV_GATE(0x320, 1),

	/* CLK_TOP_APLL12_CK_DIV2 = 181 — gate@0x320 bit 2 */
	[CLK_TOP_APLL12_CK_DIV2] = TOPCK_DIV_GATE(0x320, 2),

	/* CLK_TOP_APLL12_CK_DIV3 = 182 — gate@0x320 bit 3 */
	[CLK_TOP_APLL12_CK_DIV3] = TOPCK_DIV_GATE(0x320, 3),

	/* CLK_TOP_APLL12_CK_DIV4 = 183 — gate@0x320 bit 4 */
	[CLK_TOP_APLL12_CK_DIV4] = TOPCK_DIV_GATE(0x320, 4),

	/* CLK_TOP_APLL12_CK_DIV9 = 184 — gate@0x320 bit 9 */
	[CLK_TOP_APLL12_CK_DIV9] = TOPCK_DIV_GATE(0x320, 9),
};

struct mtk_topckgen_data {
	DEVICE_MMIO_RAM;
};

struct mtk_topckgen_config {
	DEVICE_MMIO_ROM;
};

static int mtk_topckgen_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_topck_clk *clk;
	uint32_t mask, val;

	if (idx >= ARRAY_SIZE(topck_clk_table)) {
		return -EINVAL;
	}
	clk = &topck_clk_table[idx];

	/* Stub check */
	if (clk->mux_sta_reg == 0 && clk->div_gate_reg == 0) {
		return -ENOTSUP;
	}

	if (clk->div_gate_reg != 0) {
		/* DIV_GATE: inverted gate (Linux CLK_GATE_SET_TO_DISABLE) —
		 * enable by CLEARING the gate bit.
		 */
		sys_write32(sys_read32(base + clk->div_gate_reg) & ~BIT(clk->div_gate_bit),
			    base + clk->div_gate_reg);
		return 0;
	}

	/* MUX_GATE: configure mux first, then enable gate.
	 * Skip the mux write for dynamic-parent clocks — their parent is owned
	 * by the .configure callback, so writing the static mux_val here would
	 * clobber a runtime parent selection.
	 */
	if (clk->mux_width > 0 && !clk->dynamic_parent) {
		mask = GENMASK(clk->mux_shift + clk->mux_width - 1, clk->mux_shift);
		val = (uint32_t)clk->mux_val << clk->mux_shift;
		/* Use CLR/SET pair: clear field bits, then set desired value */
		sys_write32(mask, base + clk->mux_clr_reg);
		sys_write32(val,  base + clk->mux_set_reg);
		/*
		 * Write the update bit so the hardware applies the new parent.
		 * Linux clk-mux.c: regmap_write(upd_ofs, BIT(upd_shift))
		 * The bit is self-clearing — no poll needed.
		 */
		if (clk->upd_reg != 0) {
			sys_write32(BIT(clk->upd_bit), base + clk->upd_reg);
		}
	}

	/* Enable gate: write gate bit to clr_reg (inverted gate — clears disable bit) */
	sys_write32(BIT(clk->gate_bit), base + clk->mux_clr_reg);

	return 0;
}

static int mtk_topckgen_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_topck_clk *clk;

	if (idx >= ARRAY_SIZE(topck_clk_table)) {
		return -EINVAL;
	}
	clk = &topck_clk_table[idx];

	if (clk->mux_sta_reg == 0 && clk->div_gate_reg == 0) {
		return -ENOTSUP;
	}

	if (clk->div_gate_reg != 0) {
		/* DIV_GATE: inverted gate (Linux CLK_GATE_SET_TO_DISABLE) —
		 * disable by SETTING the gate bit.
		 */
		sys_write32(sys_read32(base + clk->div_gate_reg) | BIT(clk->div_gate_bit),
			    base + clk->div_gate_reg);
		return 0;
	}

	/* MUX_GATE: disable gate — write gate bit to set_reg (sets disable bit) */
	sys_write32(BIT(clk->gate_bit), base + clk->mux_set_reg);

	return 0;
}

static enum clock_control_status mtk_topckgen_clock_status(const struct device *dev,
							    clock_control_subsys_t sys)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_topck_clk *clk;

	if (idx >= ARRAY_SIZE(topck_clk_table)) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	clk = &topck_clk_table[idx];

	if (clk->mux_sta_reg == 0 && clk->div_gate_reg == 0) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	if (clk->div_gate_reg != 0) {
		/* Inverted gate (CLK_GATE_SET_TO_DISABLE): bit SET = disabled. */
		if (sys_read32(base + clk->div_gate_reg) & BIT(clk->div_gate_bit)) {
			return CLOCK_CONTROL_STATUS_OFF;
		}
		return CLOCK_CONTROL_STATUS_ON;
	}

	/* MUX_GATE: gate disabled = gate bit SET in sta_reg (inverted gate) */
	if (sys_read32(base + clk->mux_sta_reg) & BIT(clk->gate_bit)) {
		return CLOCK_CONTROL_STATUS_OFF;
	}
	return CLOCK_CONTROL_STATUS_ON;
}

/*
 * Reparent a mux clock at runtime (Linux clk_set_parent equivalent).
 * @data points to a uint8_t parent index (selector value for the mux field).
 * Only valid for MUX_GATE clocks (mux_width > 0); DIV_GATE/stub entries
 * return -ENOTSUP.
 */
static int mtk_topckgen_clock_configure(const struct device *dev,
					clock_control_subsys_t sys, void *data)
{
	size_t idx = (size_t)sys;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct mtk_topck_clk *clk;
	uint32_t mask, val;
	uint8_t parent;

	if (idx >= ARRAY_SIZE(topck_clk_table) || data == NULL) {
		return -EINVAL;
	}
	clk = &topck_clk_table[idx];

	if (clk->mux_width == 0) {
		return -ENOTSUP;
	}

	parent = *(const uint8_t *)data;
	if (parent >= BIT(clk->mux_width)) {
		return -EINVAL;
	}

	mask = GENMASK(clk->mux_shift + clk->mux_width - 1, clk->mux_shift);
	val  = (uint32_t)parent << clk->mux_shift;

	/* CLR field bits, then SET new parent (same sequence as clock_on). */
	sys_write32(mask, base + clk->mux_clr_reg);
	sys_write32(val,  base + clk->mux_set_reg);
	if (clk->upd_reg != 0) {
		sys_write32(BIT(clk->upd_bit), base + clk->upd_reg);
	}

	return 0;
}

static int mtk_topckgen_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(clock_control, mtk_topckgen_api) = {
	.on         = mtk_topckgen_clock_on,
	.off        = mtk_topckgen_clock_off,
	.get_status = mtk_topckgen_clock_status,
	.configure  = mtk_topckgen_clock_configure,
};

static struct mtk_topckgen_data topckgen_data;

static const struct mtk_topckgen_config topckgen_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
};

DEVICE_DT_INST_DEFINE(0, &mtk_topckgen_init, NULL,
		      &topckgen_data, &topckgen_cfg,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &mtk_topckgen_api);
