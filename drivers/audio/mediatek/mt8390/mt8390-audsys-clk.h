/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 AUDSYS internal clock gate table.
 * Clocks are gated directly via MMIO on the AFE base address.
 * Register offsets are from mt8390-reg.h (AUDIO_TOP_CON0/1/4/5).
 *
 * CLK_GATE_SET_TO_DISABLE convention (same as Linux):
 *   writing 1 to the bit DISABLES the clock.
 *   clock_on  → clear bit (write 0)
 *   clock_off → set bit  (write 1)
 */

#ifndef DRIVERS_AUDIO_MEDIATEK_MT8390_AUDSYS_CLK_H
#define DRIVERS_AUDIO_MEDIATEK_MT8390_AUDSYS_CLK_H

#include <stdint.h>

/* AUDSYS clock IDs — eTDM paths + all memifs */
enum mt8390_audsys_clk_id {
	MT8390_CLK_AUD_AFE = 0,        /* AUDIO_TOP_CON0 bit 2  */
	MT8390_CLK_AUD_A1SYS_HP,       /* AUDIO_TOP_CON1 bit 2  */
	MT8390_CLK_AUD_A1SYS,          /* AUDIO_TOP_CON4 bit 21 */
	MT8390_CLK_AUD_A2SYS,          /* AUDIO_TOP_CON4 bit 22 */
	MT8390_CLK_AUD_APLL,           /* AUDIO_TOP_CON0 bit 23 (APLL1 feeder) */
	MT8390_CLK_AUD_APLL2,          /* AUDIO_TOP_CON0 bit 24 (APLL2 feeder) */
	MT8390_CLK_AUD_APLL1_TUNER,    /* AUDIO_TOP_CON0 bit 19 */
	MT8390_CLK_AUD_APLL2_TUNER,    /* AUDIO_TOP_CON0 bit 20 */
	MT8390_CLK_AUD_ETDM_IN1,       /* AUDIO_TOP_CON4 bit 0  (I2SIN)  */
	MT8390_CLK_AUD_ETDM_IN2,       /* AUDIO_TOP_CON4 bit 1  (TDM_IN) */
	MT8390_CLK_AUD_ETDM_OUT1,      /* AUDIO_TOP_CON4 bit 6  (I2S_OUT) */
	MT8390_CLK_AUD_ETDM_OUT2,      /* AUDIO_TOP_CON4 bit 7  (TDM_OUT) */
	MT8390_CLK_AUD_MEMIF_UL1,      /* AUDIO_TOP_CON5 bit 0  */
	MT8390_CLK_AUD_MEMIF_UL2,      /* AUDIO_TOP_CON5 bit 1  */
	MT8390_CLK_AUD_MEMIF_UL3,      /* AUDIO_TOP_CON5 bit 2  */
	MT8390_CLK_AUD_MEMIF_UL4,      /* AUDIO_TOP_CON5 bit 3  */
	MT8390_CLK_AUD_MEMIF_UL5,      /* AUDIO_TOP_CON5 bit 4  */
	MT8390_CLK_AUD_MEMIF_UL6,      /* AUDIO_TOP_CON5 bit 5  */
	MT8390_CLK_AUD_MEMIF_UL8,      /* AUDIO_TOP_CON5 bit 7  */
	MT8390_CLK_AUD_MEMIF_UL9,      /* AUDIO_TOP_CON5 bit 8  */
	MT8390_CLK_AUD_MEMIF_UL10,     /* AUDIO_TOP_CON5 bit 9  */
	MT8390_CLK_AUD_MEMIF_DL2,      /* AUDIO_TOP_CON5 bit 18 */
	MT8390_CLK_AUD_MEMIF_DL3,      /* AUDIO_TOP_CON5 bit 19 */
	MT8390_CLK_AUD_MEMIF_DL6,      /* AUDIO_TOP_CON5 bit 22 */
	MT8390_CLK_AUD_MEMIF_DL7,      /* AUDIO_TOP_CON5 bit 23 */
	MT8390_CLK_AUD_MEMIF_DL8,      /* AUDIO_TOP_CON5 bit 24 */
	MT8390_CLK_AUD_MEMIF_DL10,     /* AUDIO_TOP_CON5 bit 26 */
	MT8390_CLK_AUD_MEMIF_DL11,     /* AUDIO_TOP_CON5 bit 27 */
	MT8390_CLK_AUDSYS_NR,
};

int mt8390_audsys_clk_on(uintptr_t afe_base, enum mt8390_audsys_clk_id id);
int mt8390_audsys_clk_off(uintptr_t afe_base, enum mt8390_audsys_clk_id id);

#endif /* DRIVERS_AUDIO_MEDIATEK_MT8390_AUDSYS_CLK_H */
