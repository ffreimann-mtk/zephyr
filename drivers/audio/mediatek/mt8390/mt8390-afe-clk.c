/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 AFE clock enable/disable — 4-tier sequence.
 * Ported from Linux mt8188-afe-clk.c mt8188_afe_enable_clocks().
 *
 * Tier 1: APMIXEDSYS  — APLL1 (48kHz) or APLL2 (44.1kHz) PLL
 * Tier 2: TOPCKGEN    — audio mux+gate clocks
 * Tier 3: INFRACFG_AO — infra audio gate clocks
 * Tier 4: AUDSYS      — direct MMIO on AFE base
 *
 * Note: CLK_TOP_APLL1_D4 / CLK_TOP_APLL2_D4 are fixed dividers derived
 * from APLL1/APLL2 — always available when the parent PLL is on. Their
 * clock_control_on() may return -ENOTSUP from the TOPCKGEN driver stub;
 * this is expected and ignored here.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/mtk_mt8188_clock.h>

#include "mt8390-afe.h"
#include "mt8390-audsys-clk.h"
#include "mt8390-reg.h"

/*
 * ADSP audio 26M clock gate.
 * Linux: "adsp_audio_26m" obtained via CCF from &adsp_audio26m DT node.
 * Physical base: 0x10b91100, register offset 0x80, bit 3.
 * CLK_GATE_SET_TO_DISABLE convention: set bit = disabled.
 *
 * The mapped virtual address is stored in afe->adsp_audio26m, set up by
 * device_map() in mt8390_afe_init().
 */
#define ADSP_AUDIO26M_CON0   0x80U
#define ADSP_AUDIO26M_BIT    BIT(3)

static void adsp_audio26m_enable(struct mt8390_afe *afe)
{
	uintptr_t reg = afe->adsp_audio26m + ADSP_AUDIO26M_CON0;

	sys_write32(sys_read32(reg) & ~ADSP_AUDIO26M_BIT, reg);
}

static void adsp_audio26m_disable(struct mt8390_afe *afe)
{
	uintptr_t reg = afe->adsp_audio26m + ADSP_AUDIO26M_CON0;

	sys_write32(sys_read32(reg) | ADSP_AUDIO26M_BIT, reg);
}

/* Ignore -ENOTSUP for fixed-divider stubs (e.g. APLL1_D4, APLL2_D4) */
#define CLK_ON_TOLERANT(dev, id)                                    \
	do {                                                         \
		int _r = clock_control_on((dev), (clock_control_subsys_t)(id)); \
		if (_r != 0 && _r != -ENOTSUP) {                    \
			return _r;                                   \
		}                                                    \
	} while (0)

#define CLK_OFF_TOLERANT(dev, id)                                    \
	do {                                                          \
		int _r = clock_control_off((dev), (clock_control_subsys_t)(id)); \
		if (_r != 0 && _r != -ENOTSUP) {                     \
			return _r;                                    \
		}                                                     \
	} while (0)

/* -------------------------------------------------------------------------
 * mt8390_afe_enable_main_clock / disable_main_clock
 * Linux: mt8188_afe_enable_main_clock() / mt8188_afe_disable_main_clock()
 *
 * Operations:
 *   enable:  set ASYS_TOP_CON_26M_TIMING_ON, then set AFE_DAC_CON0 bit 0
 *   disable: clear AFE_DAC_CON0 bit 0, then clear ASYS_TOP_CON_26M_TIMING_ON
 * -------------------------------------------------------------------------
 */
void mt8390_afe_enable_main_clock(struct mt8390_afe *afe)
{
	/* 26M timing — ASYS_TOP_CON bit 2 (set = on, non-inverted) */
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) |
		    ASYS_TOP_CON_26M_TIMING_ON,
		    afe->base + ASYS_TOP_CON);

	/* AFE global enable — AFE_DAC_CON0 bit 0 */
	sys_write32(sys_read32(afe->base + AFE_DAC_CON0) | BIT(0),
		    afe->base + AFE_DAC_CON0);
}

void mt8390_afe_disable_main_clock(struct mt8390_afe *afe)
{
	/* AFE global disable */
	sys_write32(sys_read32(afe->base + AFE_DAC_CON0) & ~BIT(0),
		    afe->base + AFE_DAC_CON0);

	/* 26M timing off */
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) &
		    ~ASYS_TOP_CON_26M_TIMING_ON,
		    afe->base + ASYS_TOP_CON);
}

/* -------------------------------------------------------------------------
 * APLL tuner configuration — ported from Linux struct mt8188_afe_tuner_cfg
 * and mt8188_afe_tuner_cfgs[] in mt8188-afe-clk.c.
 *
 * Each entry describes the register layout of one PLL tuner instance.
 * mt8390_afe_tuner_enable/disable operate generically on any entry.
 * -------------------------------------------------------------------------
 */
struct mt8390_afe_tuner_cfg {
	uint32_t apll_div_reg;
	uint8_t  apll_div_shift;
	uint32_t apll_div_mask;
	uint8_t  apll_div_default;
	uint32_t ref_ck_sel_reg;
	uint8_t  ref_ck_sel_shift;
	uint32_t ref_ck_sel_mask;
	uint8_t  ref_ck_sel_default;
	uint32_t tuner_en_reg;
	uint8_t  tuner_en_shift;
	uint32_t upper_bound_reg;
	uint8_t  upper_bound_shift;
	uint32_t upper_bound_mask;
	uint8_t  upper_bound_default;
	/* AUDIO_TOP_CON0 PDN gates feeding this tuner (Linux
	 * mt8188_afe_enable_tuner_clk). Only PLL1/PLL2 have them; for the
	 * other PLLs set has_audsys_gates = false.
	 */
	bool                     has_audsys_gates;
	enum mt8390_audsys_clk_id feeder_gate;  /* CLK_AUD_APLL / _APLL2 */
	enum mt8390_audsys_clk_id tuner_gate;   /* CLK_AUD_APLL{1,2}_TUNER */
};

enum mt8390_aud_pll {
	MT8390_AUD_PLL1 = 0,
	MT8390_AUD_PLL2,
	MT8390_AUD_PLL3,
	MT8390_AUD_PLL4,
	MT8390_AUD_PLL5,
	MT8390_AUD_PLL_NUM,
};

static const struct mt8390_afe_tuner_cfg
	mt8390_afe_tuner_cfgs[MT8390_AUD_PLL_NUM] = {
	[MT8390_AUD_PLL1] = {
		.apll_div_reg       = AFE_APLL_TUNER_CFG,
		.apll_div_shift     = 4,
		.apll_div_mask      = 0xf,
		.apll_div_default   = 0x7,
		.ref_ck_sel_reg     = AFE_APLL_TUNER_CFG,
		.ref_ck_sel_shift   = 1,
		.ref_ck_sel_mask    = 0x3,
		.ref_ck_sel_default = 0x2,
		.tuner_en_reg       = AFE_APLL_TUNER_CFG,
		.tuner_en_shift     = 0,
		.upper_bound_reg    = AFE_APLL_TUNER_CFG,
		.upper_bound_shift  = 8,
		.upper_bound_mask   = 0xff,
		.upper_bound_default = 0x3,
		.has_audsys_gates   = true,
		.feeder_gate        = MT8390_CLK_AUD_APLL,
		.tuner_gate         = MT8390_CLK_AUD_APLL1_TUNER,
	},
	[MT8390_AUD_PLL2] = {
		.apll_div_reg       = AFE_APLL_TUNER_CFG1,
		.apll_div_shift     = 4,
		.apll_div_mask      = 0xf,
		.apll_div_default   = 0x7,
		.ref_ck_sel_reg     = AFE_APLL_TUNER_CFG1,
		.ref_ck_sel_shift   = 1,
		.ref_ck_sel_mask    = 0x3,
		.ref_ck_sel_default = 0x1,
		.tuner_en_reg       = AFE_APLL_TUNER_CFG1,
		.tuner_en_shift     = 0,
		.upper_bound_reg    = AFE_APLL_TUNER_CFG1,
		.upper_bound_shift  = 8,
		.upper_bound_mask   = 0xff,
		.upper_bound_default = 0x3,
		.has_audsys_gates   = true,
		.feeder_gate        = MT8390_CLK_AUD_APLL2,
		.tuner_gate         = MT8390_CLK_AUD_APLL2_TUNER,
	},
	[MT8390_AUD_PLL3] = {
		.apll_div_reg       = AFE_EARC_APLL_TUNER_CFG,
		.apll_div_shift     = 4,
		.apll_div_mask      = 0x3f,
		.apll_div_default   = 0x3,
		.ref_ck_sel_reg     = AFE_EARC_APLL_TUNER_CFG,
		.ref_ck_sel_shift   = 24,
		.ref_ck_sel_mask    = 0x3,
		.ref_ck_sel_default = 0x0,
		.tuner_en_reg       = AFE_EARC_APLL_TUNER_CFG,
		.tuner_en_shift     = 0,
		.upper_bound_reg    = AFE_EARC_APLL_TUNER_CFG,
		.upper_bound_shift  = 12,
		.upper_bound_mask   = 0xff,
		.upper_bound_default = 0x4,
	},
	[MT8390_AUD_PLL4] = {
		.apll_div_reg       = AFE_SPDIFIN_APLL_TUNER_CFG,
		.apll_div_shift     = 4,
		.apll_div_mask      = 0x3f,
		.apll_div_default   = 0x7,
		.ref_ck_sel_reg     = AFE_SPDIFIN_APLL_TUNER_CFG1,
		.ref_ck_sel_shift   = 8,
		.ref_ck_sel_mask    = 0x1,
		.ref_ck_sel_default = 0,
		.tuner_en_reg       = AFE_SPDIFIN_APLL_TUNER_CFG,
		.tuner_en_shift     = 0,
		.upper_bound_reg    = AFE_SPDIFIN_APLL_TUNER_CFG,
		.upper_bound_shift  = 12,
		.upper_bound_mask   = 0xff,
		.upper_bound_default = 0x4,
	},
	[MT8390_AUD_PLL5] = {
		.apll_div_reg       = AFE_LINEIN_APLL_TUNER_CFG,
		.apll_div_shift     = 4,
		.apll_div_mask      = 0x3f,
		.apll_div_default   = 0x3,
		.ref_ck_sel_reg     = AFE_LINEIN_APLL_TUNER_CFG,
		.ref_ck_sel_shift   = 24,
		.ref_ck_sel_mask    = 0x1,
		.ref_ck_sel_default = 0,
		.tuner_en_reg       = AFE_LINEIN_APLL_TUNER_CFG,
		.tuner_en_shift     = 0,
		.upper_bound_reg    = AFE_LINEIN_APLL_TUNER_CFG,
		.upper_bound_shift  = 12,
		.upper_bound_mask   = 0xff,
		.upper_bound_default = 0x4,
	},
};

/*
 * mt8390_afe_tuner_enable / mt8390_afe_tuner_disable
 * Linux: mt8188_afe_setup_apll_tuner() + mt8188_afe_enable_tuner_clk()
 *        + enable/disable tuner_en bit.
 */
static void mt8390_afe_tuner_enable(struct mt8390_afe *afe,
				    const struct mt8390_afe_tuner_cfg *cfg)
{
	uint32_t val;

	/* Setup apll_div */
	val = sys_read32(afe->base + cfg->apll_div_reg);
	val &= ~(cfg->apll_div_mask << cfg->apll_div_shift);
	val |= ((uint32_t)cfg->apll_div_default << cfg->apll_div_shift);
	sys_write32(val, afe->base + cfg->apll_div_reg);

	/* Setup ref_ck_sel */
	val = sys_read32(afe->base + cfg->ref_ck_sel_reg);
	val &= ~(cfg->ref_ck_sel_mask << cfg->ref_ck_sel_shift);
	val |= ((uint32_t)cfg->ref_ck_sel_default << cfg->ref_ck_sel_shift);
	sys_write32(val, afe->base + cfg->ref_ck_sel_reg);

	/* Setup upper_bound */
	val = sys_read32(afe->base + cfg->upper_bound_reg);
	val &= ~(cfg->upper_bound_mask << cfg->upper_bound_shift);
	val |= ((uint32_t)cfg->upper_bound_default << cfg->upper_bound_shift);
	sys_write32(val, afe->base + cfg->upper_bound_reg);

	/* Ungate the APLL feeder + tuner clocks (AUDIO_TOP_CON0 PDN bits) —
	 * Linux mt8188_afe_enable_tuner_clk(): feeder first, then tuner.
	 */
	if (cfg->has_audsys_gates) {
		mt8390_audsys_clk_on(afe->base, cfg->feeder_gate);
		mt8390_audsys_clk_on(afe->base, cfg->tuner_gate);
	}

	/* Enable tuner */
	sys_write32(sys_read32(afe->base + cfg->tuner_en_reg) |
		    BIT(cfg->tuner_en_shift),
		    afe->base + cfg->tuner_en_reg);
}

static void mt8390_afe_tuner_disable(struct mt8390_afe *afe,
				     const struct mt8390_afe_tuner_cfg *cfg)
{
	sys_write32(sys_read32(afe->base + cfg->tuner_en_reg) &
		    ~BIT(cfg->tuner_en_shift),
		    afe->base + cfg->tuner_en_reg);

	/* Gate the tuner + feeder clocks — Linux mt8188_afe_disable_tuner_clk():
	 * tuner first, then feeder (reverse of enable).
	 */
	if (cfg->has_audsys_gates) {
		mt8390_audsys_clk_off(afe->base, cfg->tuner_gate);
		mt8390_audsys_clk_off(afe->base, cfg->feeder_gate);
	}
}

/* -------------------------------------------------------------------------
 * mt8390_afe_enable_a1sys / disable_a1sys
 * Linux: mt8188_afe_enable_a1sys() / mt8188_afe_disable_a1sys()
 *
 * Operations:
 *   enable:  CLK_AUD_A1SYS gate + ASYS_TOP_CON_A1SYS_TIMING_ON
 *   disable: clear timing bit, then CLK_AUD_A1SYS gate off
 * -------------------------------------------------------------------------
 */
static void mt8390_afe_enable_a1sys(struct mt8390_afe *afe)
{
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_A1SYS);
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) |
		    ASYS_TOP_CON_A1SYS_TIMING_ON,
		    afe->base + ASYS_TOP_CON);
}

static void mt8390_afe_disable_a1sys(struct mt8390_afe *afe)
{
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) &
		    ~ASYS_TOP_CON_A1SYS_TIMING_ON,
		    afe->base + ASYS_TOP_CON);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_A1SYS);
}

/* -------------------------------------------------------------------------
 * mt8390_afe_enable_a2sys / disable_a2sys
 * Linux: mt8188_afe_enable_a2sys() / mt8188_afe_disable_a2sys()
 *
 * Operations:
 *   enable:  CLK_AUD_A2SYS gate + ASYS_TOP_CON_A2SYS_TIMING_ON
 *   disable: clear timing bit, then CLK_AUD_A2SYS gate off
 * -------------------------------------------------------------------------
 */
static void mt8390_afe_enable_a2sys(struct mt8390_afe *afe)
{
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_A2SYS);
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) |
		    ASYS_TOP_CON_A2SYS_TIMING_ON,
		    afe->base + ASYS_TOP_CON);
}

static void mt8390_afe_disable_a2sys(struct mt8390_afe *afe)
{
	sys_write32(sys_read32(afe->base + ASYS_TOP_CON) &
		    ~ASYS_TOP_CON_A2SYS_TIMING_ON,
		    afe->base + ASYS_TOP_CON);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_A2SYS);
}

/* -------------------------------------------------------------------------
 * A1SYS_HP mux parent select helper.
 * Linux: clk_set_parent(A1SYS_HP_SEL, APLL1_D4) / clk_set_parent(..., clk26m)
 *   parent[0]=clk26m, parent[1]=apll1_d4
 *
 * Parent select and gate enable are independent: A1SYS_HP is a dynamic-parent
 * clock in the topckgen table, so clock_control_on/off() toggle only the gate
 * (they do not rewrite the mux), and .configure() owns the parent. This lets
 * the parent set here survive the gate enable.
 * -------------------------------------------------------------------------
 */
static void a1sys_hp_set_parent(struct mt8390_afe *afe, uint8_t parent_idx)
{
	clock_control_configure(afe->clk_topckgen,
				(clock_control_subsys_t)(uintptr_t)CLK_TOP_A1SYS_HP,
				&parent_idx);
}

/* -------------------------------------------------------------------------
 * mt8390_apll1_enable / mt8390_apll1_disable
 * Linux: mt8188_apll1_enable() / mt8188_apll1_disable()
 *
 * mt8188_apll1_enable sequence:
 *   1. enable CLK_TOP_APLL1_D4 (fixed divider — always on when APLL1 is on)
 *   2. set_parent(A1SYS_HP_SEL, APLL1_D4) — mux index 1
 *   3. enable APLL1 tuner
 *   4. enable A1SYS (gate + timing)
 *
 * mt8188_apll1_disable sequence (reverse):
 *   1. disable A1SYS
 *   2. disable APLL1 tuner
 *   3. set_parent(A1SYS_HP_SEL, clk26m) — mux index 0
 *   4. disable CLK_TOP_APLL1_D4
 * -------------------------------------------------------------------------
 */
int mt8390_apll1_enable(struct mt8390_afe *afe)
{
	/* Step 1: enable APLL1_D4 (fixed divider — tolerant if stub) */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL1_D4);

	/* Step 2: set A1SYS_HP mux parent to apll1_d4 (index 1) + enable gate.
	 * A1SYS_HP is a dynamic-parent clock, so clock_control_on() enables the
	 * gate only and leaves the parent set above intact.
	 */
	a1sys_hp_set_parent(afe, 1);
	clock_control_on(afe->clk_topckgen,
			 (clock_control_subsys_t)(uintptr_t)CLK_TOP_A1SYS_HP);

	/* Step 3: enable APLL1 tuner */
	mt8390_afe_tuner_enable(afe, &mt8390_afe_tuner_cfgs[MT8390_AUD_PLL1]);

	/* Step 4: enable A1SYS */
	mt8390_afe_enable_a1sys(afe);

	return 0;
}

int mt8390_apll1_disable(struct mt8390_afe *afe)
{
	/* Step 1: disable A1SYS */
	mt8390_afe_disable_a1sys(afe);

	/* Step 2: disable APLL1 tuner */
	mt8390_afe_tuner_disable(afe, &mt8390_afe_tuner_cfgs[MT8390_AUD_PLL1]);

	/* Step 3: disable gate, then revert A1SYS_HP mux to clk26m (index 0) */
	clock_control_off(afe->clk_topckgen,
			  (clock_control_subsys_t)(uintptr_t)CLK_TOP_A1SYS_HP);
	a1sys_hp_set_parent(afe, 0);

	/* Step 4: disable APLL1_D4 */
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL1_D4);

	return 0;
}

/* -------------------------------------------------------------------------
 * mt8390_apll2_enable / mt8390_apll2_disable
 * Linux: mt8188_apll2_enable() / mt8188_apll2_disable()
 *
 * mt8188_apll2_enable sequence:
 *   1. enable APLL tuner (PLL2)
 *   2. enable A2SYS (CLK_AUD_A2SYS + A2SYS_TIMING_ON)
 *
 * mt8188_apll2_disable sequence:
 *   1. disable A2SYS
 *   2. disable APLL tuner (PLL2)
 * -------------------------------------------------------------------------
 */
int mt8390_apll2_enable(struct mt8390_afe *afe)
{
	mt8390_afe_tuner_enable(afe, &mt8390_afe_tuner_cfgs[MT8390_AUD_PLL2]);
	mt8390_afe_enable_a2sys(afe);
	return 0;
}

int mt8390_apll2_disable(struct mt8390_afe *afe)
{
	mt8390_afe_disable_a2sys(afe);
	mt8390_afe_tuner_disable(afe, &mt8390_afe_tuner_cfgs[MT8390_AUD_PLL2]);
	return 0;
}

/* -------------------------------------------------------------------------
 * mt8390_afe_enable_reg_rw_clk / mt8390_afe_disable_reg_rw_clk
 * Linux: mt8188_afe_enable_reg_rw_clk() / mt8188_afe_disable_reg_rw_clk()
 *
 * Enables the minimum set of clocks required for AFE register access:
 *   1. CLK_TOP_AUDIO_LOCAL_BUS — bus clock for DRAM access
 *   2. CLK_TOP_AUD_INTBUS      — bus clock for AFE SRAM access
 *   3. CLK_ADSP_AUDIO_26M      — 26M reference
 *   4. CLK_AUD_AFE             — AFE HW gate
 *   5. CLK_AUD_A1SYS_HP        — A1SYS HP gate
 *   6. CLK_AUD_A1SYS           — A1SYS gate
 * -------------------------------------------------------------------------
 */
int mt8390_afe_enable_reg_rw_clk(struct mt8390_afe *afe)
{
	/* bus clock for AFE external access (DRAM) */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_AUDIO_LOCAL_BUS);

	/* bus clock for AFE internal access (SRAM) */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_AUD_INTBUS);

	/* audio 26M clock source */
	adsp_audio26m_enable(afe);

	/* AFE HW clock */
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_AFE);
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_A1SYS_HP);
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_A1SYS);

	return 0;
}

int mt8390_afe_disable_reg_rw_clk(struct mt8390_afe *afe)
{
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_A1SYS);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_A1SYS_HP);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_AFE);
	adsp_audio26m_disable(afe);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_AUD_INTBUS);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_AUDIO_LOCAL_BUS);

	return 0;
}

/* -------------------------------------------------------------------------
 * afe_enable_base_clocks / afe_disable_base_clocks
 *
 * Common base clocks enabled for all eTDM paths.
 * Calls mt8390_afe_enable_reg_rw_clk() + mt8390_afe_enable_main_clock(),
 * then additional eTDM-path clocks. A1SYS_TIMING_ON is NOT set here — it is
 * tied to APLL1 enable only (mt8390_apll1_enable), mirroring Linux.
 * -------------------------------------------------------------------------
 */
int afe_enable_base_clocks(struct mt8390_afe *afe)
{
	int ret;

	/* Core register-access clocks (includes the CLK_AUD_A1SYS gate) */
	ret = mt8390_afe_enable_reg_rw_clk(afe);
	if (ret) {
		return ret;
	}

	/* Main clock (26M timing + AFE_DAC_CON0).
	 * Note: A1SYS_TIMING_ON is deliberately NOT set here — like Linux, it
	 * is tied to APLL1 enable only (mt8390_apll1_enable via the rate-based
	 * domain), so a2sys-only (44.1k) paths don't assert a1sys timing.
	 */
	mt8390_afe_enable_main_clock(afe);

	/* --- Additional clocks for eTDM paths (dependent on bus clocks above) --- */

	/* INFRA audio gates — required for I2S DMA and audio 26M BCLK */
	CLK_ON_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_AUDIO);
	CLK_ON_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_AUDIO_26M_BCLK);
	CLK_ON_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_I2S_DMA);

	/* Audio H — required for hi-res DAC/ADC and DMIC */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_AUDIO_H);

	/* ASM clocks — required for GASRC and audio stream processing */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_ASM_H);
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_ASM_L);

	return 0;
}

int afe_disable_base_clocks(struct mt8390_afe *afe)
{
	/* Additional clocks off first (reverse of enable order) */
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_ASM_L);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_ASM_H);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_AUDIO_H);

	CLK_OFF_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_I2S_DMA);
	CLK_OFF_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_AUDIO_26M_BCLK);
	CLK_OFF_TOLERANT(afe->clk_infra_a0, CLK_INFRA_AO_AUDIO);

	/* Main clock */
	mt8390_afe_disable_main_clock(afe);

	/* A1SYS_TIMING_ON is owned by mt8390_apll1_disable (rate-based domain),
	 * not cleared here — mirrors Linux.
	 */

	/* Core register-access clocks (gates the CLK_AUD_A1SYS gate) */
	mt8390_afe_disable_reg_rw_clk(afe);

	return 0;
}

/* -------------------------------------------------------------------------
 * APLL domain selection by sample rate — Linux mt8188_get_apll_by_rate().
 *   rate % 8000 == 0 (48k family)  → APLL1 / a1sys
 *   else            (44.1k family) → APLL2 / a2sys
 * The a1sys_hp / a2sys muxes are fixed to apll1_d4 / apll2_d4 respectively,
 * so the timing domain *is* the APLL choice. Used by both the eTDM OUT and
 * IN clock paths — the domain follows the sample rate, not the port.
 * -------------------------------------------------------------------------
 */
static bool rate_uses_apll1(uint32_t rate)
{
	return (rate % 8000U) == 0U;
}

/* Enable the APLL1/a1sys or APLL2/a2sys timing domain for the given rate. */
static int enable_apll_domain(struct mt8390_afe *afe, uint32_t rate)
{
	if (rate_uses_apll1(rate)) {
		/* APLL1 domain (Linux mt8188_apll1_enable):
		 * root APLL1 + APLL1_D4 + A1SYS_HP reparent→apll1_d4 + tuner +
		 * A1SYS. mt8390_apll1_enable() subsumes the A1SYS_HP gate.
		 */
		CLK_ON_TOLERANT(afe->clk_apmixed, CLK_APMIXED_APLL1);
		mt8390_apll1_enable(afe);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL1);
	} else {
		/* APLL2 domain (Linux mt8188_apll2_enable):
		 * root APLL2 + APLL2 tuner + A2SYS (gate + A2SYS_TIMING_ON).
		 */
		CLK_ON_TOLERANT(afe->clk_apmixed, CLK_APMIXED_APLL2);
		mt8390_apll2_enable(afe);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL2);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_A2SYS);
	}

	return 0;
}

static int disable_apll_domain(struct mt8390_afe *afe, uint32_t rate)
{
	if (rate_uses_apll1(rate)) {
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL1);
		mt8390_apll1_disable(afe);
		CLK_OFF_TOLERANT(afe->clk_apmixed, CLK_APMIXED_APLL1);
	} else {
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_A2SYS);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL2);
		mt8390_apll2_disable(afe);
		CLK_OFF_TOLERANT(afe->clk_apmixed, CLK_APMIXED_APLL2);
	}

	return 0;
}

int mt8390_afe_enable_etdm_out_clocks(struct mt8390_afe *afe, enum mt8390_etdm_id id,
				      uint32_t rate)
{
	int ret;

	/* Base clocks (register bus + a1sys gate) stay on for the device
	 * lifetime — enabled once in mt8390_afe_init(). Only the per-path
	 * timing domain and port clocks are toggled here.
	 *
	 * Timing domain (APLL1/a1sys vs APLL2/a2sys) follows the rate, not
	 * the port — Linux selects by mt8188_get_apll_by_rate().
	 */
	ret = enable_apll_domain(afe, rate);
	if (ret) {
		return ret;
	}

	/* Per-port clocks: APLL12 dividers, I2SO mux, eTDM + memif gates. */
	if (id == MT8390_ETDM_OUT1) {
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV2);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SO1);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_OUT1);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_MEMIF_DL2);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_MEMIF_DL3);
	} else if (id == MT8390_ETDM_OUT2) {
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV3);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SO2);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_OUT2);
	}

	return 0;
}

int mt8390_afe_disable_etdm_out_clocks(struct mt8390_afe *afe, enum mt8390_etdm_id id,
				       uint32_t rate)
{
	if (id == MT8390_ETDM_OUT1) {
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_MEMIF_DL3);
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_MEMIF_DL2);
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_OUT1);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SO1);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV2);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);
	} else if (id == MT8390_ETDM_OUT2) {
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_OUT2);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SO2);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV3);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
	}

	/* Base clocks are NOT disabled here — they stay on for the device
	 * lifetime so concurrent streams and restart cycles keep the register
	 * bus clocked. Only the per-path timing domain is released.
	 */
	return disable_apll_domain(afe, rate);
}

int mt8390_afe_enable_etdm_in_clocks(struct mt8390_afe *afe, enum mt8390_etdm_id id,
				     uint32_t rate)
{
	int ret;

	/* Base clocks stay on for the device lifetime (see init); only the
	 * per-path timing domain and port clocks are toggled here.
	 *
	 * Timing domain (APLL1/a1sys vs APLL2/a2sys) follows the rate, not
	 * the port — Linux selects by mt8188_get_apll_by_rate().
	 */
	ret = enable_apll_domain(afe, rate);
	if (ret) {
		return ret;
	}

	/* Per-port clocks: APLL12 divider, I2SI mux, eTDM + memif gates. */
	if (id == MT8390_ETDM_IN1) {
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI1);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_IN1);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_MEMIF_UL8);
	} else if (id == MT8390_ETDM_IN2) {
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
		CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI2);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_IN2);
		mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_MEMIF_UL3);
	}

	return 0;
}

int mt8390_afe_disable_etdm_in_clocks(struct mt8390_afe *afe, enum mt8390_etdm_id id,
				      uint32_t rate)
{
	if (id == MT8390_ETDM_IN1) {
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_MEMIF_UL8);
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_IN1);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI1);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);
	} else if (id == MT8390_ETDM_IN2) {
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_MEMIF_UL3);
		mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_IN2);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI2);
		CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
	}

	/* Base clocks stay on for the device lifetime (see init). */
	return disable_apll_domain(afe, rate);
}

/* Enable clocks for CM0 cowork path (UL9 ← eTDM_IN1 + IN2).
 * Both cowork ports run at the same rate, so they share ONE timing domain
 * (APLL1/a1sys for 48k, APLL2/a2sys for 44.1k) selected by that rate.
 */
int mt8390_afe_enable_cm0_clocks(struct mt8390_afe *afe, uint32_t rate)
{
	int ret;

	/* Base clocks stay on for the device lifetime (see init).
	 * Single rate-selected timing domain shared by IN1 + IN2.
	 */
	ret = enable_apll_domain(afe, rate);
	if (ret) {
		return ret;
	}

	/* Per-port clocks for both cowork ports. */
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI1);
	CLK_ON_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI2);
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_IN1);
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_ETDM_IN2);
	mt8390_audsys_clk_on(afe->base, MT8390_CLK_AUD_MEMIF_UL9);

	return 0;
}

int mt8390_afe_disable_cm0_clocks(struct mt8390_afe *afe, uint32_t rate)
{
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_MEMIF_UL9);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_IN2);
	mt8390_audsys_clk_off(afe->base, MT8390_CLK_AUD_ETDM_IN1);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI2);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_I2SI1);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV1);
	CLK_OFF_TOLERANT(afe->clk_topckgen, CLK_TOP_APLL12_CK_DIV0);

	/* Base clocks stay on for the device lifetime (see init). */
	return disable_apll_domain(afe, rate);
}
