/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 AUDSYS internal clock gate enable/disable via direct MMIO.
 * Ported from Linux mt8188-audsys-clk.c.
 *
 * Convention: CLK_GATE_SET_TO_DISABLE — writing 1 to the bit disables.
 *   clock_on  → read-modify-write: clear the gate bit
 *   clock_off → read-modify-write: set the gate bit
 */

#include <zephyr/arch/cpu.h>

#include "mt8390-audsys-clk.h"
#include "mt8390-reg.h"

struct audsys_clk_entry {
	uint32_t reg;   /* AUDIO_TOP_CON* register offset */
	uint8_t  bit;   /* gate bit (set=disable, clear=enable) */
};

/*
 * Table indexed by mt8390_audsys_clk_id.
 * Register values from Linux mt8188-audsys-clk.c:
 *   AUDIO_TOP_CON0 = 0x0000
 *   AUDIO_TOP_CON1 = 0x0004
 *   AUDIO_TOP_CON4 = 0x0010  (mapped to CON4 in mt8188-reg.h)
 *   AUDIO_TOP_CON5 = 0x0014
 */
static const struct audsys_clk_entry audsys_clk_table[MT8390_CLK_AUDSYS_NR] = {
	/* MT8390_CLK_AUD_AFE    — AUDIO_TOP_CON0 bit 2 */
	[MT8390_CLK_AUD_AFE]       = { AUDIO_TOP_CON0, 2 },
	/* MT8390_CLK_AUD_A1SYS_HP — AUDIO_TOP_CON1 bit 2 */
	[MT8390_CLK_AUD_A1SYS_HP]  = { AUDIO_TOP_CON1, 2 },
	/* MT8390_CLK_AUD_A1SYS  — AUDIO_TOP_CON4 bit 21 */
	[MT8390_CLK_AUD_A1SYS]     = { AUDIO_TOP_CON4, 21 },
	/* MT8390_CLK_AUD_A2SYS  — AUDIO_TOP_CON4 bit 22 */
	[MT8390_CLK_AUD_A2SYS]     = { AUDIO_TOP_CON4, 22 },
	/* MT8390_CLK_AUD_APLL        — AUDIO_TOP_CON0 bit 23 (APLL1 feeder) */
	[MT8390_CLK_AUD_APLL]        = { AUDIO_TOP_CON0, 23 },
	/* MT8390_CLK_AUD_APLL2       — AUDIO_TOP_CON0 bit 24 (APLL2 feeder) */
	[MT8390_CLK_AUD_APLL2]       = { AUDIO_TOP_CON0, 24 },
	/* MT8390_CLK_AUD_APLL1_TUNER — AUDIO_TOP_CON0 bit 19 */
	[MT8390_CLK_AUD_APLL1_TUNER] = { AUDIO_TOP_CON0, 19 },
	/* MT8390_CLK_AUD_APLL2_TUNER — AUDIO_TOP_CON0 bit 20 */
	[MT8390_CLK_AUD_APLL2_TUNER] = { AUDIO_TOP_CON0, 20 },
	/* eTDM port→gate map per Linux mtk_dai_etdm_get_cg_id_by_dai_id():
	 *   ETDM_IN1 → tdm_in (bit 1), ETDM_IN2 → i2sin   (bit 0)
	 *   ETDM_OUT1→ tdm_out(bit 7), ETDM_OUT2→ i2s_out (bit 6)
	 */
	/* MT8390_CLK_AUD_ETDM_IN1  — AUDIO_TOP_CON4 bit 1 (TDM_IN) */
	[MT8390_CLK_AUD_ETDM_IN1]  = { AUDIO_TOP_CON4, 1 },
	/* MT8390_CLK_AUD_ETDM_IN2  — AUDIO_TOP_CON4 bit 0 (I2SIN) */
	[MT8390_CLK_AUD_ETDM_IN2]  = { AUDIO_TOP_CON4, 0 },
	/* MT8390_CLK_AUD_ETDM_OUT1 — AUDIO_TOP_CON4 bit 7 (TDM_OUT) */
	[MT8390_CLK_AUD_ETDM_OUT1] = { AUDIO_TOP_CON4, 7 },
	/* MT8390_CLK_AUD_ETDM_OUT2 — AUDIO_TOP_CON4 bit 6 (I2S_OUT) */
	[MT8390_CLK_AUD_ETDM_OUT2] = { AUDIO_TOP_CON4, 6 },
	/* MT8390_CLK_AUD_MEMIF_UL1  — AUDIO_TOP_CON5 bit 0 */
	[MT8390_CLK_AUD_MEMIF_UL1]  = { AUDIO_TOP_CON5, 0 },
	/* MT8390_CLK_AUD_MEMIF_UL2  — AUDIO_TOP_CON5 bit 1 */
	[MT8390_CLK_AUD_MEMIF_UL2]  = { AUDIO_TOP_CON5, 1 },
	/* MT8390_CLK_AUD_MEMIF_UL3  — AUDIO_TOP_CON5 bit 2 */
	[MT8390_CLK_AUD_MEMIF_UL3]  = { AUDIO_TOP_CON5, 2 },
	/* MT8390_CLK_AUD_MEMIF_UL4  — AUDIO_TOP_CON5 bit 3 */
	[MT8390_CLK_AUD_MEMIF_UL4]  = { AUDIO_TOP_CON5, 3 },
	/* MT8390_CLK_AUD_MEMIF_UL5  — AUDIO_TOP_CON5 bit 4 */
	[MT8390_CLK_AUD_MEMIF_UL5]  = { AUDIO_TOP_CON5, 4 },
	/* MT8390_CLK_AUD_MEMIF_UL6  — AUDIO_TOP_CON5 bit 5 */
	[MT8390_CLK_AUD_MEMIF_UL6]  = { AUDIO_TOP_CON5, 5 },
	/* MT8390_CLK_AUD_MEMIF_UL8  — AUDIO_TOP_CON5 bit 7 */
	[MT8390_CLK_AUD_MEMIF_UL8]  = { AUDIO_TOP_CON5, 7 },
	/* MT8390_CLK_AUD_MEMIF_UL9  — AUDIO_TOP_CON5 bit 8 */
	[MT8390_CLK_AUD_MEMIF_UL9]  = { AUDIO_TOP_CON5, 8 },
	/* MT8390_CLK_AUD_MEMIF_UL10 — AUDIO_TOP_CON5 bit 9 */
	[MT8390_CLK_AUD_MEMIF_UL10] = { AUDIO_TOP_CON5, 9 },
	/* MT8390_CLK_AUD_MEMIF_DL2  — AUDIO_TOP_CON5 bit 18 */
	[MT8390_CLK_AUD_MEMIF_DL2]  = { AUDIO_TOP_CON5, 18 },
	/* MT8390_CLK_AUD_MEMIF_DL3  — AUDIO_TOP_CON5 bit 19 */
	[MT8390_CLK_AUD_MEMIF_DL3]  = { AUDIO_TOP_CON5, 19 },
	/* MT8390_CLK_AUD_MEMIF_DL6  — AUDIO_TOP_CON5 bit 22 */
	[MT8390_CLK_AUD_MEMIF_DL6]  = { AUDIO_TOP_CON5, 22 },
	/* MT8390_CLK_AUD_MEMIF_DL7  — AUDIO_TOP_CON5 bit 23 */
	[MT8390_CLK_AUD_MEMIF_DL7]  = { AUDIO_TOP_CON5, 23 },
	/* MT8390_CLK_AUD_MEMIF_DL8  — AUDIO_TOP_CON5 bit 24 */
	[MT8390_CLK_AUD_MEMIF_DL8]  = { AUDIO_TOP_CON5, 24 },
	/* MT8390_CLK_AUD_MEMIF_DL10 — AUDIO_TOP_CON5 bit 26 */
	[MT8390_CLK_AUD_MEMIF_DL10] = { AUDIO_TOP_CON5, 26 },
	/* MT8390_CLK_AUD_MEMIF_DL11 — AUDIO_TOP_CON5 bit 27 */
	[MT8390_CLK_AUD_MEMIF_DL11] = { AUDIO_TOP_CON5, 27 },
};

int mt8390_audsys_clk_on(uintptr_t afe_base, enum mt8390_audsys_clk_id id)
{
	const struct audsys_clk_entry *e;
	uint32_t val;

	if ((unsigned int)id >= MT8390_CLK_AUDSYS_NR) {
		return -EINVAL;
	}
	e = &audsys_clk_table[id];
	/* CLK_GATE_SET_TO_DISABLE: clear bit to enable */
	val = sys_read32(afe_base + e->reg);
	val &= ~BIT(e->bit);
	sys_write32(val, afe_base + e->reg);
	return 0;
}

int mt8390_audsys_clk_off(uintptr_t afe_base, enum mt8390_audsys_clk_id id)
{
	const struct audsys_clk_entry *e;
	uint32_t val;

	if ((unsigned int)id >= MT8390_CLK_AUDSYS_NR) {
		return -EINVAL;
	}
	e = &audsys_clk_table[id];
	/* CLK_GATE_SET_TO_DISABLE: set bit to disable */
	val = sys_read32(afe_base + e->reg);
	val |= BIT(e->bit);
	sys_write32(val, afe_base + e->reg);
	return 0;
}
