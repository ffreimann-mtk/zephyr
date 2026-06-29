/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 AFE (Audio Front End) main driver.
 *
 * Ported from Linux mt8188-afe-pcm.c:
 *   - FS timing table (mt8188_afe_rates[])
 *   - Full memif descriptor table (memif_data[]) including mono, int_odd_flag
 *   - IRQ descriptor table (irq_data[]) — ASYS_IRQ1..16 + AFE_IRQ1..10
 *   - memif→IRQ mapping (mt8188_afe_memif_const_irqs[])
 *   - IRQ handler (mt8188_afe_irq_handler)
 */

#define DT_DRV_COMPAT mediatek_mt8390_afe

#include <zephyr/arch/cpu.h>
#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

/*
 * MediaTek SIP SMC constants — ported from Linux:
 *   include/linux/soc/mediatek/mtk_sip_svc.h
 *   sound/soc/mediatek/common/mtk-base-afe.h
 *
 * ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, SMC_64, OWNER_SIP, 0x517)
 * = 0xC2000517 on ARM64
 */
#define MTK_SIP_SMC_CMD(fn_id)  (0xC2000000U | (fn_id))
#define MTK_SIP_AUDIO_CONTROL   MTK_SIP_SMC_CMD(0x517)
#define MTK_AUDIO_SMC_OP_DOMAIN_SIDEBANDS  7

/*
 * Bus-protect (INFRACFG_AO AXI protection) — ported from Linux:
 *   include/linux/soc/mediatek/infracfg.h
 *   sound/soc/mediatek/mt8188/mt8188-afe-pcm.c bus_protect_enable/disable()
 *
 * Offsets are within the INFRACFG_AO block (the clk_infra_a0 device @0x10001000).
 */
#define INFRA_TOP_AXI_PROT_EN_2_SET    0x714U
#define INFRA_TOP_AXI_PROT_EN_2_CLR    0x718U
#define INFRA_TOP_AXI_PROT_EN_2_STA    0x724U
#define INFRA_AXI_PROT_AUDIO_STEP1     BIT(20)
#define INFRA_AXI_PROT_AUDIO_STEP2     BIT(12)
#define INFRA_AXI_PROT_POLL_US         10
#define INFRA_AXI_PROT_TIMEOUT_US      1000000

/*
 * Audiosys reset via TOPRGU/watchdog — ported from Linux:
 *   DT: resets = <&watchdog 14>; reset-names = "audiosys"
 *   drivers/watchdog/mtk_wdt.c toprgu_reset()
 *
 * The TOPRGU block @0x10007000 has no Zephyr driver, so map it directly
 * (same pattern as adsp_audio26m). WDT_SWSYSRST requires the 0x88 key in
 * the top byte on every write; bit 14 is the audiosys reset line.
 */
#define TOPRGU_BASE_ADDR       0x10007000UL
#define TOPRGU_BASE_SIZE       0x100U
#define WDT_SWSYSRST_OFS       0x18U
#define WDT_SWSYS_RST_KEY      0x88000000U
#define WDT_SWSYSRST_AUDIO_BIT BIT(14)

#include "mt8390-afe.h"
#include "mt8390-reg.h"

/* -------------------------------------------------------------------------
 * Sample rate → memif FS field value
 * Source: mt8188_afe_rates[] in Linux mt8188-afe-pcm.c
 * Used for: AFE_MEMIF_AGENT_FS_CON* registers
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_fs_timing(uint32_t rate)
{
	switch (rate) {
	case 8000:    return 0;
	case 12000:   return 1;
	case 16000:   return 2;
	case 24000:   return 3;
	case 32000:   return 4;
	case 48000:   return 5;
	case 96000:   return 6;
	case 192000:  return 7;
	case 384000:  return 8;
	case 7350:    return 16;
	case 11025:   return 17;
	case 14700:   return 18;
	case 22050:   return 19;
	case 29400:   return 20;
	case 44100:   return 21;
	case 88200:   return 22;
	case 176400:  return 23;
	case 352800:  return 24;
	default:      return -EINVAL;
	}
}

/* -------------------------------------------------------------------------
 * Memif descriptor
 * Source: mtk_base_memif_data in Linux mt8188-afe-pcm.c memif_data[]
 * -------------------------------------------------------------------------
 */
struct mt8390_memif_data {
	uint32_t reg_base;
	uint32_t reg_cur;
	uint32_t reg_end;
	int      fs_reg;          /* -1 = no FS register */
	uint8_t  fs_shift;
	uint32_t fs_mask;
	int      mono_reg;        /* -1 = no mono register (DL paths) */
	uint8_t  mono_shift;      /* bit 1 of ULx_CON0: 1=mono, 0=stereo */
	int      int_odd_flag_reg; /* -1 = no int_odd_flag */
	uint8_t  int_odd_flag_shift; /* bit 0 of ULx_CON0: interleaved odd */
	int      hd_reg;
	uint8_t  hd_shift;
	uint8_t  enable_shift;
	uint8_t  agent_dis_shift;
	int      ch_num_reg;
	uint8_t  ch_num_shift;
	uint32_t ch_num_mask;
	int      irq_id;          /* index into irq_table[], -1 = none */
	bool     active;
};

/* -------------------------------------------------------------------------
 * IRQ descriptor
 * Source: mtk_base_irq_data in Linux mt8188-afe-pcm.c irq_data[]
 * -------------------------------------------------------------------------
 */
struct mt8390_irq_data {
	int      irq_cnt_reg;      /* -1 = no period-count register */
	uint8_t  irq_cnt_shift;
	uint32_t irq_cnt_mask;
	int      irq_fs_reg;       /* -1 = no FS register */
	uint8_t  irq_fs_shift;
	uint32_t irq_fs_mask;
	uint32_t irq_en_reg;
	uint8_t  irq_en_shift;     /* bit 31 = enable */
	uint32_t irq_clr_reg;      /* ASYS_IRQ_CLR or AFE_IRQ_MCU_CLR */
	uint8_t  irq_clr_shift;
	uint8_t  irq_status_shift; /* bit in AFE_IRQ_STATUS / ASYS_IRQ_STATUS */
	bool     is_asys;          /* true = ASYS_IRQ_CLR, false = AFE_IRQ_MCU_CLR */
};

/*
 * IRQ IDs — indices into irq_table[].
 * Matches Linux MT8188_AFE_IRQ_* numbering where needed.
 */
enum mt8390_irq_id {
	MT8390_IRQ_AFE_1  = 0,
	MT8390_IRQ_AFE_2,
	MT8390_IRQ_AFE_3,
	MT8390_IRQ_AFE_8,
	MT8390_IRQ_AFE_9,
	MT8390_IRQ_AFE_10,
	MT8390_IRQ_ASYS_1,   /* ASYS_IRQ1  → IRQ_13 in Linux */
	MT8390_IRQ_ASYS_2,
	MT8390_IRQ_ASYS_3,
	MT8390_IRQ_ASYS_4,
	MT8390_IRQ_ASYS_5,
	MT8390_IRQ_ASYS_6,
	MT8390_IRQ_ASYS_7,
	MT8390_IRQ_ASYS_8,
	MT8390_IRQ_ASYS_9,
	MT8390_IRQ_ASYS_10,
	MT8390_IRQ_ASYS_11,
	MT8390_IRQ_ASYS_12,
	MT8390_IRQ_ASYS_13,
	MT8390_IRQ_NR,
};

/*
 * IRQ table — register values from Linux mt8188-afe-pcm.c irq_data[].
 * All ASYS IRQs share AFE_IRQ_STATUS for status and ASYS_IRQ_CLR for clear.
 * is_asys=true → status from ASYS_IRQ_STATUS, clear to ASYS_IRQ_CLR
 * is_asys=false → status from AFE_IRQ_STATUS,  clear to AFE_IRQ_MCU_CLR
 */
static const struct mt8390_irq_data irq_table[MT8390_IRQ_NR] = {
	[MT8390_IRQ_AFE_1] = {
		.irq_cnt_reg = -1, .irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ1_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 0,
		.irq_status_shift = 16, .is_asys = false,
	},
	[MT8390_IRQ_AFE_2] = {
		.irq_cnt_reg = -1, .irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ2_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 1,
		.irq_status_shift = 17, .is_asys = false,
	},
	[MT8390_IRQ_AFE_3] = {
		.irq_cnt_reg = AFE_IRQ3_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ3_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 2,
		.irq_status_shift = 18, .is_asys = false,
	},
	[MT8390_IRQ_AFE_8] = {
		.irq_cnt_reg = -1, .irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ8_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 7,
		.irq_status_shift = 23, .is_asys = false,
	},
	[MT8390_IRQ_AFE_9] = {
		.irq_cnt_reg = AFE_IRQ9_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ9_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 8,
		.irq_status_shift = 24, .is_asys = false,
	},
	[MT8390_IRQ_AFE_10] = {
		.irq_cnt_reg = -1, .irq_fs_reg = -1,
		.irq_en_reg = AFE_IRQ10_CON, .irq_en_shift = 31,
		.irq_clr_reg = AFE_IRQ_MCU_CLR, .irq_clr_shift = 9,
		.irq_status_shift = 25, .is_asys = false,
	},
	/* ASYS_IRQ1..16 — cnt/fs/en all in ASYS_IRQx_CON, clear in ASYS_IRQ_CLR */
	[MT8390_IRQ_ASYS_1] = {
		.irq_cnt_reg = ASYS_IRQ1_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ1_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ1_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 0,
		.irq_status_shift = 0, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_2] = {
		.irq_cnt_reg = ASYS_IRQ2_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ2_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ2_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 1,
		.irq_status_shift = 1, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_3] = {
		.irq_cnt_reg = ASYS_IRQ3_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ3_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ3_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 2,
		.irq_status_shift = 2, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_4] = {
		.irq_cnt_reg = ASYS_IRQ4_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ4_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ4_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 3,
		.irq_status_shift = 3, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_5] = {
		.irq_cnt_reg = ASYS_IRQ5_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ5_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ5_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 4,
		.irq_status_shift = 4, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_6] = {
		.irq_cnt_reg = ASYS_IRQ6_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ6_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ6_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 5,
		.irq_status_shift = 5, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_7] = {
		.irq_cnt_reg = ASYS_IRQ7_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ7_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ7_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 6,
		.irq_status_shift = 6, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_8] = {
		.irq_cnt_reg = ASYS_IRQ8_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ8_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ8_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 7,
		.irq_status_shift = 7, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_9] = {
		.irq_cnt_reg = ASYS_IRQ9_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ9_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ9_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 8,
		.irq_status_shift = 8, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_10] = {
		.irq_cnt_reg = ASYS_IRQ10_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ10_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ10_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 9,
		.irq_status_shift = 9, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_11] = {
		.irq_cnt_reg = ASYS_IRQ11_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ11_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ11_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 10,
		.irq_status_shift = 10, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_12] = {
		.irq_cnt_reg = ASYS_IRQ12_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ12_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ12_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 11,
		.irq_status_shift = 11, .is_asys = true,
	},
	[MT8390_IRQ_ASYS_13] = {
		.irq_cnt_reg = ASYS_IRQ13_CON, .irq_cnt_shift = 0, .irq_cnt_mask = 0xffffff,
		.irq_fs_reg = ASYS_IRQ13_CON, .irq_fs_shift = 24, .irq_fs_mask = 0x1ffff,
		.irq_en_reg = ASYS_IRQ13_CON, .irq_en_shift = 31,
		.irq_clr_reg = ASYS_IRQ_CLR, .irq_clr_shift = 12,
		.irq_status_shift = 12, .is_asys = true,
	},
};

/* -------------------------------------------------------------------------
 * Memif table
 * Source: memif_data[] in Linux mt8188-afe-pcm.c
 * All UL memifs have mono_reg = ULx_CON0 bit 1, int_odd_flag = bit 0.
 * All DL memifs have mono_reg = -1.
 * irq_id: maps Linux mt8188_afe_memif_const_irqs[] to MT8390_IRQ_* enum.
 *   DL2→ASYS_1, DL3→ASYS_2, UL3→ASYS_7, UL8→ASYS_10, UL9→ASYS_11
 * -------------------------------------------------------------------------
 */
/*
 * Complete memif table — all 16 memifs defined by Linux mt8188-afe-pcm.c.
 * Register values from memif_data[] and mt8188_afe_memif_const_irqs[].
 * active=true for the 5 paths wired to eTDM; others have full register
 * data and can be activated by setting active=true and wiring a path.
 *
 * IRQ mapping (Linux IRQ_* → Zephyr MT8390_IRQ_*):
 *   IRQ_1  → AFE_1    IRQ_2  → AFE_2    IRQ_3  → AFE_3
 *   IRQ_8  → AFE_8    IRQ_9  → AFE_9    IRQ_10 → AFE_10
 *   IRQ_13 → ASYS_1   IRQ_14 → ASYS_2   IRQ_15 → ASYS_3
 *   IRQ_16 → ASYS_4   IRQ_17 → ASYS_5   IRQ_18 → ASYS_6
 *   IRQ_19 → ASYS_7   IRQ_20 → ASYS_8   IRQ_21 → ASYS_9
 *   IRQ_22 → ASYS_10  IRQ_23 → ASYS_11  IRQ_24 → ASYS_12 (UL9)
 *   IRQ_25 → ASYS_13
 *
 * Note: DL1, DL4, DL5, DL9, UL7 do not exist in MT8188/MT8390 hardware and
 * are absent from the enum and this table.
 */
static const struct mt8390_memif_data memif_table[MT8390_MEMIF_NR] = {
	/* DL2 — IRQ_13 → ASYS_1 */
	[MT8390_DL2] = {
		.reg_base = AFE_DL2_BASE, .reg_cur = AFE_DL2_CUR, .reg_end = AFE_DL2_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON0, .fs_shift = 10, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL2_CON0, .hd_shift = 5,
		.enable_shift = 18, .agent_dis_shift = 18,
		.ch_num_reg = AFE_DL2_CON0, .ch_num_shift = 0, .ch_num_mask = 0x1f,
		.irq_id = MT8390_IRQ_ASYS_1,
		.active = false,
	},
	/* DL3 — IRQ_14 → ASYS_2 */
	[MT8390_DL3] = {
		.reg_base = AFE_DL3_BASE, .reg_cur = AFE_DL3_CUR, .reg_end = AFE_DL3_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON0, .fs_shift = 15, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL3_CON0, .hd_shift = 5,
		.enable_shift = 19, .agent_dis_shift = 19,
		.ch_num_reg = AFE_DL3_CON0, .ch_num_shift = 0, .ch_num_mask = 0x1f,
		.irq_id = MT8390_IRQ_ASYS_2,
		.active = false,
	},
	/* DL6 — IRQ_15 → ASYS_3 */
	[MT8390_DL6] = {
		.reg_base = AFE_DL6_BASE, .reg_cur = AFE_DL6_CUR, .reg_end = AFE_DL6_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON1, .fs_shift = 0, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL6_CON0, .hd_shift = 5,
		.enable_shift = 22, .agent_dis_shift = 22,
		.ch_num_reg = AFE_DL6_CON0, .ch_num_shift = 0, .ch_num_mask = 0x1f,
		.irq_id = MT8390_IRQ_ASYS_3,
		.active = false,
	},
	/* DL7 — fs_reg=-1 (fixed rate) — IRQ_1 → AFE_1 */
	[MT8390_DL7] = {
		.reg_base = AFE_DL7_BASE, .reg_cur = AFE_DL7_CUR, .reg_end = AFE_DL7_END,
		.fs_reg = -1,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL7_CON0, .hd_shift = 5,
		.enable_shift = 23, .agent_dis_shift = 23,
		.ch_num_reg = AFE_DL7_CON0, .ch_num_shift = 0, .ch_num_mask = 0x1f,
		.irq_id = MT8390_IRQ_AFE_1,
		.active = false,
	},
	/* DL8 — hd_shift=6, ch_num_mask=0x3f — IRQ_16 → ASYS_4 */
	[MT8390_DL8] = {
		.reg_base = AFE_DL8_BASE, .reg_cur = AFE_DL8_CUR, .reg_end = AFE_DL8_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON1, .fs_shift = 10, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL8_CON0, .hd_shift = 6,
		.enable_shift = 24, .agent_dis_shift = 24,
		.ch_num_reg = AFE_DL8_CON0, .ch_num_shift = 0, .ch_num_mask = 0x3f,
		.irq_id = MT8390_IRQ_ASYS_4,
		.active = false,
	},
	/* DL10 — IRQ_17 → ASYS_5 */
	[MT8390_DL10] = {
		.reg_base = AFE_DL10_BASE, .reg_cur = AFE_DL10_CUR, .reg_end = AFE_DL10_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON1, .fs_shift = 20, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL10_CON0, .hd_shift = 5,
		.enable_shift = 26, .agent_dis_shift = 26,
		.ch_num_reg = AFE_DL10_CON0, .ch_num_shift = 0, .ch_num_mask = 0x1f,
		.irq_id = MT8390_IRQ_ASYS_5,
		.active = false,
	},
	/* DL11 — hd_shift=7, ch_num_mask=0x7f — IRQ_18 → ASYS_6 */
	[MT8390_DL11] = {
		.reg_base = AFE_DL11_BASE, .reg_cur = AFE_DL11_CUR, .reg_end = AFE_DL11_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON1, .fs_shift = 25, .fs_mask = 0x1f,
		.mono_reg = -1, .int_odd_flag_reg = -1,
		.hd_reg = AFE_DL11_CON0, .hd_shift = 7,
		.enable_shift = 27, .agent_dis_shift = 27,
		.ch_num_reg = AFE_DL11_CON0, .ch_num_shift = 0, .ch_num_mask = 0x7f,
		.irq_id = MT8390_IRQ_ASYS_6,
		.active = false,
	},

	/* UL1 — fs_reg=-1 — IRQ_3 → AFE_3 */
	[MT8390_UL1] = {
		.reg_base = AFE_UL1_BASE, .reg_cur = AFE_UL1_CUR, .reg_end = AFE_UL1_END,
		.fs_reg = -1,
		.mono_reg = AFE_UL1_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL1_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL1_CON0, .hd_shift = 5,
		.enable_shift = 1, .agent_dis_shift = 0,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_AFE_3,
		.active = false,
	},
	/* UL2 — IRQ_19 → ASYS_7; CM1 user */
	[MT8390_UL2] = {
		.reg_base = AFE_UL2_BASE, .reg_cur = AFE_UL2_CUR, .reg_end = AFE_UL2_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON2, .fs_shift = 5, .fs_mask = 0x1f,
		.mono_reg = AFE_UL2_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL2_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL2_CON0, .hd_shift = 5,
		.enable_shift = 2, .agent_dis_shift = 1,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_ASYS_7,
		.active = false,
	},
	/* UL3 — active (eTDM_IN2) — IRQ_20 → ASYS_8 */
	[MT8390_UL3] = {
		.reg_base = AFE_UL3_BASE, .reg_cur = AFE_UL3_CUR, .reg_end = AFE_UL3_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON2, .fs_shift = 10, .fs_mask = 0x1f,
		.mono_reg = AFE_UL3_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL3_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL3_CON0, .hd_shift = 5,
		.enable_shift = 3, .agent_dis_shift = 2,
		.ch_num_reg = -1,
		/* IRQ disabled — timer-based (get_cur) data handling.
		 * Original: MT8390_IRQ_ASYS_8 (IRQ_20).
		 */
		.irq_id = -1,
		.active = true,
	},
	/* UL4 — IRQ_21 → ASYS_9 */
	[MT8390_UL4] = {
		.reg_base = AFE_UL4_BASE, .reg_cur = AFE_UL4_CUR, .reg_end = AFE_UL4_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON2, .fs_shift = 15, .fs_mask = 0x1f,
		.mono_reg = AFE_UL4_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL4_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL4_CON0, .hd_shift = 5,
		.enable_shift = 4, .agent_dis_shift = 3,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_ASYS_9,
		.active = false,
	},
	/* UL5 — IRQ_22 → ASYS_10 */
	[MT8390_UL5] = {
		.reg_base = AFE_UL5_BASE, .reg_cur = AFE_UL5_CUR, .reg_end = AFE_UL5_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON2, .fs_shift = 20, .fs_mask = 0x1f,
		.mono_reg = AFE_UL5_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL5_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL5_CON0, .hd_shift = 5,
		.enable_shift = 5, .agent_dis_shift = 4,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_ASYS_10,
		.active = false,
	},
	/* UL6 — fs_reg=-1 — IRQ_9 → AFE_9 */
	[MT8390_UL6] = {
		.reg_base = AFE_UL6_BASE, .reg_cur = AFE_UL6_CUR, .reg_end = AFE_UL6_END,
		.fs_reg = -1,
		.mono_reg = AFE_UL6_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL6_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL6_CON0, .hd_shift = 5,
		.enable_shift = 6, .agent_dis_shift = 5,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_AFE_9,
		.active = false,
	},
	/* UL8 — active (eTDM_IN1) — IRQ_23 → ASYS_11 */
	[MT8390_UL8] = {
		.reg_base = AFE_UL8_BASE, .reg_cur = AFE_UL8_CUR, .reg_end = AFE_UL8_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON3, .fs_shift = 5, .fs_mask = 0x1f,
		.mono_reg = AFE_UL8_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL8_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL8_CON0, .hd_shift = 5,
		.enable_shift = 8, .agent_dis_shift = 7,
		.ch_num_reg = -1,
		/* IRQ disabled — timer-based (get_cur) data handling.
		 * Original: MT8390_IRQ_ASYS_11 (IRQ_23).
		 */
		.irq_id = -1,
		.active = true,
	},
	/* UL9 — active (CM0 cowork) — IRQ_24 → ASYS_12 */
	[MT8390_UL9] = {
		.reg_base = AFE_UL9_BASE, .reg_cur = AFE_UL9_CUR, .reg_end = AFE_UL9_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON3, .fs_shift = 10, .fs_mask = 0x1f,
		.mono_reg = AFE_UL9_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL9_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL9_CON0, .hd_shift = 5,
		.enable_shift = 9, .agent_dis_shift = 8,
		.ch_num_reg = -1,
		/* IRQ disabled — timer-based (get_cur) data handling.
		 * Original: MT8390_IRQ_ASYS_12 (IRQ_24).
		 */
		.irq_id = -1,
		.active = true,
	},
	/* UL10 — CM2 user — IRQ_25 → ASYS_13 */
	[MT8390_UL10] = {
		.reg_base = AFE_UL10_BASE, .reg_cur = AFE_UL10_CUR, .reg_end = AFE_UL10_END,
		.fs_reg = AFE_MEMIF_AGENT_FS_CON3, .fs_shift = 15, .fs_mask = 0x1f,
		.mono_reg = AFE_UL10_CON0, .mono_shift = 1,
		.int_odd_flag_reg = AFE_UL10_CON0, .int_odd_flag_shift = 0,
		.hd_reg = AFE_UL10_CON0, .hd_shift = 5,
		.enable_shift = 10, .agent_dis_shift = 9,
		.ch_num_reg = -1,
		.irq_id = MT8390_IRQ_ASYS_13,
		.active = false,
	},
};

/* -------------------------------------------------------------------------
 * Channel Merge (CM) unit — AFE_CM0_CON / CM1_CON / CM2_CON
 * Source: mt8188_afe_channel_merge + mt8188_afe_cm[] in Linux mt8188-afe-pcm.c
 *
 * CM0 — used by UL9  (CM0_CON = 0x0660)
 * CM1 — used by UL2  (CM1_CON = 0x0664)  [UL2 not active in Zephyr]
 * CM2 — used by UL10 (CM2_CON = 0x0668)  [UL10 not active in Zephyr]
 *
 * Fields in AFE_CMx_CON:
 *   [31:30] sel     — merge source select (default=1)
 *   [7:2]   ch_num  — (channels-1); CM0 is 6-bit, CM1/CM2 are 5-bit
 *   [28:16] update_cnt — update counter (default=3)
 *   [0]     en      — enable
 * -------------------------------------------------------------------------
 */
struct mt8390_cm_data {
	uint32_t reg;
	uint8_t  sel_shift;
	uint32_t sel_mask;
	uint8_t  sel_default;
	uint8_t  ch_num_shift;
	uint32_t ch_num_mask;
	uint8_t  en_shift;
	uint8_t  update_cnt_shift;
	uint32_t update_cnt_mask;
	uint8_t  update_cnt_default;
};

enum mt8390_cm_id {
	MT8390_CM0 = 0,
	MT8390_CM1,
	MT8390_CM2,
	MT8390_CM_NR,
};

static const struct mt8390_cm_data cm_table[MT8390_CM_NR] = {
	[MT8390_CM0] = {
		.reg             = AFE_CM0_CON,
		.sel_shift       = 30, .sel_mask        = 0x1, .sel_default    = 1,
		.ch_num_shift    = 2,  .ch_num_mask     = 0x3f,
		.en_shift        = 0,
		.update_cnt_shift = 16, .update_cnt_mask = 0x1fff, .update_cnt_default = 3,
	},
	[MT8390_CM1] = {
		.reg             = AFE_CM1_CON,
		.sel_shift       = 30, .sel_mask        = 0x1, .sel_default    = 1,
		.ch_num_shift    = 2,  .ch_num_mask     = 0x1f,
		.en_shift        = 0,
		.update_cnt_shift = 16, .update_cnt_mask = 0x1fff, .update_cnt_default = 3,
	},
	[MT8390_CM2] = {
		.reg             = AFE_CM2_CON,
		.sel_shift       = 30, .sel_mask        = 0x1, .sel_default    = 1,
		.ch_num_shift    = 2,  .ch_num_mask     = 0x1f,
		.en_shift        = 0,
		.update_cnt_shift = 16, .update_cnt_mask = 0x1fff, .update_cnt_default = 3,
	},
};

/* memif → CM mapping — Linux mt8188_afe_found_cm():
 *   UL9  → CM0
 *   UL2  → CM1  (not active in Zephyr)
 *   UL10 → CM2  (not active in Zephyr)
 * Returns -1 if no CM for this memif.
 */
static int memif_to_cm(enum mt8390_memif_id id)
{
	switch (id) {
	case MT8390_UL9:  return MT8390_CM0;
	case MT8390_UL2:  return MT8390_CM1;
	case MT8390_UL10: return MT8390_CM2;
	default:          return -1;
	}
}

/* mt8188_afe_config_cm() — write sel, ch_num, update_cnt */
static void cm_config(uintptr_t base, const struct mt8390_cm_data *cm,
		      uint32_t channels)
{
	uint32_t val;

	/* sel field */
	val = sys_read32(base + cm->reg);
	val &= ~(cm->sel_mask << cm->sel_shift);
	val |= ((uint32_t)cm->sel_default << cm->sel_shift);
	sys_write32(val, base + cm->reg);

	/* ch_num field: (channels - 1) */
	val = sys_read32(base + cm->reg);
	val &= ~(cm->ch_num_mask << cm->ch_num_shift);
	val |= ((channels - 1) & cm->ch_num_mask) << cm->ch_num_shift;
	sys_write32(val, base + cm->reg);

	/* update_cnt field */
	val = sys_read32(base + cm->reg);
	val &= ~(cm->update_cnt_mask << cm->update_cnt_shift);
	val |= ((uint32_t)cm->update_cnt_default << cm->update_cnt_shift);
	sys_write32(val, base + cm->reg);
}

/* mt8188_afe_enable_cm() — set or clear the enable bit */
static void cm_enable(uintptr_t base, const struct mt8390_cm_data *cm,
		      bool enable)
{
	uint32_t val = sys_read32(base + cm->reg);

	if (enable) {
		val |= BIT(cm->en_shift);
	} else {
		val &= ~BIT(cm->en_shift);
	}
	sys_write32(val, base + cm->reg);
}

/* -------------------------------------------------------------------------
 * memif_to_etdm
 * -------------------------------------------------------------------------
 */
static enum mt8390_etdm_id memif_to_etdm(enum mt8390_memif_id id)
{
	switch (id) {
	case MT8390_UL8:
		return MT8390_ETDM_IN1;
	case MT8390_UL3:
		return MT8390_ETDM_IN2;
	case MT8390_UL9:
		return MT8390_ETDM_IN1;
	default:
		return MT8390_ETDM_NR;
	}
}

/* -------------------------------------------------------------------------
 * IRQ helpers
 * -------------------------------------------------------------------------
 */

/* Set period count in IRQ CON register (number of frames per period) */
static void afe_irq_set_period(uintptr_t base, const struct mt8390_irq_data *irq,
			   uint32_t period_frames)
{
	if (irq->irq_cnt_reg < 0) {
		return;
	}
	uint32_t val = sys_read32(base + irq->irq_cnt_reg);

	val &= ~(irq->irq_cnt_mask << irq->irq_cnt_shift);
	val |= (period_frames & irq->irq_cnt_mask) << irq->irq_cnt_shift;
	sys_write32(val, base + irq->irq_cnt_reg);
}

/* Set FS timing in IRQ CON register */
static void afe_irq_set_fs(uintptr_t base, const struct mt8390_irq_data *irq,
		       uint32_t fs)
{
	if (irq->irq_fs_reg < 0) {
		return;
	}
	uint32_t val = sys_read32(base + irq->irq_fs_reg);

	val &= ~(irq->irq_fs_mask << irq->irq_fs_shift);
	val |= (fs & irq->irq_fs_mask) << irq->irq_fs_shift;
	sys_write32(val, base + irq->irq_fs_reg);
}

static void afe_irq_enable(uintptr_t base, const struct mt8390_irq_data *irq)
{
	sys_write32(sys_read32(base + irq->irq_en_reg) | BIT(irq->irq_en_shift),
		    base + irq->irq_en_reg);
}

static void afe_irq_disable(uintptr_t base, const struct mt8390_irq_data *irq)
{
	sys_write32(sys_read32(base + irq->irq_en_reg) & ~BIT(irq->irq_en_shift),
		    base + irq->irq_en_reg);
}

static void afe_irq_clear(uintptr_t base, const struct mt8390_irq_data *irq)
{
	sys_write32(BIT(irq->irq_clr_shift), base + irq->irq_clr_reg);
}

/* -------------------------------------------------------------------------
 * ISR — mirrors Linux mt8188_afe_irq_handler()
 * -------------------------------------------------------------------------
 */
static void mt8390_afe_isr(const struct device *dev)
{
	struct mt8390_afe *afe = dev->data;
	uint32_t afe_status;
	uint32_t asys_status;
	uint32_t afe_clr  = 0;
	uint32_t asys_clr = 0;
	int i;

	afe_status  = sys_read32(afe->base + AFE_IRQ_STATUS);
	asys_status = sys_read32(afe->base + ASYS_IRQ_STATUS);

	for (i = 0; i < MT8390_MEMIF_NR; i++) {
		const struct mt8390_memif_data *m = &memif_table[i];
		const struct mt8390_irq_data *irq;
		uint32_t status_bit;

		if (!m->active || m->irq_id < 0 || m->irq_id >= MT8390_IRQ_NR) {
			continue;
		}
		irq = &irq_table[m->irq_id];
		status_bit = BIT(irq->irq_status_shift);

		if (irq->is_asys) {
			if (!(asys_status & status_bit)) {
				continue;
			}
			asys_clr |= BIT(irq->irq_clr_shift);
		} else {
			if (!(afe_status & status_bit)) {
				continue;
			}
			afe_clr |= BIT(irq->irq_clr_shift);
		}

		/* Invoke period callback if registered */
		if (afe->period_cb[i].cb != NULL) {
			afe->period_cb[i].cb(afe->period_cb[i].data);
		}
	}

	/* Clear all fired IRQs */
	if (asys_clr) {
		sys_write32(asys_clr, afe->base + ASYS_IRQ_CLR);
	}
	if (afe_clr) {
		sys_write32(afe_clr, afe->base + AFE_IRQ_MCU_CLR);
	}
}

/* -------------------------------------------------------------------------
 * Public API: configure
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_configure(const struct device *dev,
				    enum mt8390_memif_id memif_id,
				    const struct mt8390_afe_cfg *cfg)
{
	struct mt8390_afe *afe = dev->data;
	const struct mt8390_memif_data *m;
	enum mt8390_etdm_id etdm_id;
	int fs;
	uint32_t val;
	k_spinlock_key_t key;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return -EINVAL;
	}
	m = &memif_table[memif_id];
	if (!m->active) {
		return -ENOTSUP;
	}

	fs = mt8390_afe_fs_timing(cfg->rate);
	if (fs < 0) {
		return -EINVAL;
	}

	/* memif FS source override — Linux mt8188_memif_fs().
	 * memifs wired directly to an eTDM IN port track that port's Nx_EN
	 * clock rather than a rate-derived FS token:
	 *   UL8 ← eTDM_IN1 → ETDM_IN1_NX_EN
	 *   UL3 ← eTDM_IN2 → ETDM_IN2_NX_EN
	 */
	if (memif_id == MT8390_UL8) {
		fs = MT8390_ETDM_IN1_NX_EN;
	} else if (memif_id == MT8390_UL3) {
		fs = MT8390_ETDM_IN2_NX_EN;
	}

	key = k_spin_lock(&afe->lock);

	/* FS */
	if (m->fs_reg >= 0) {
		val = sys_read32(afe->base + m->fs_reg);
		val &= ~(m->fs_mask << m->fs_shift);
		val |= ((uint32_t)fs << m->fs_shift);
		sys_write32(val, afe->base + m->fs_reg);
	}

	/* HD (high-definition) — bit set = 32-bit */
	if (m->hd_reg >= 0) {
		val = sys_read32(afe->base + m->hd_reg);
		if (cfg->word_size >= 32) {
			val |= BIT(m->hd_shift);
		} else {
			val &= ~BIT(m->hd_shift);
		}
		sys_write32(val, afe->base + m->hd_reg);
	}

	/* Mono — ULx_CON0 bit 1: 0=stereo, 1=mono */
	if (m->mono_reg >= 0) {
		val = sys_read32(afe->base + m->mono_reg);
		if (cfg->channels == 1) {
			val |= BIT(m->mono_shift);
		} else {
			val &= ~BIT(m->mono_shift);
		}
		sys_write32(val, afe->base + m->mono_reg);
	}

	/* int_odd_flag — ULx_CON0 bit 0. Linux mtk_memif_set_channel() writes
	 * this with the same value as the mono bit:
	 *   mono = mono_invert ^ (channel == 1)  [mono_invert is 0 on mt8188]
	 *   int_odd_flag = mono
	 * So it is set only for mono (1 channel), matching the mono bit above.
	 */
	if (m->int_odd_flag_reg >= 0) {
		val = sys_read32(afe->base + m->int_odd_flag_reg);
		if (cfg->channels == 1) {
			val |= BIT(m->int_odd_flag_shift);
		} else {
			val &= ~BIT(m->int_odd_flag_shift);
		}
		sys_write32(val, afe->base + m->int_odd_flag_reg);
	}

	/* Channel count */
	if (m->ch_num_reg >= 0 && m->ch_num_mask != 0) {
		uint32_t ch_val = (cfg->channels - 1) & m->ch_num_mask;

		val = sys_read32(afe->base + m->ch_num_reg);
		val &= ~(m->ch_num_mask << m->ch_num_shift);
		val |= (ch_val << m->ch_num_shift);
		sys_write32(val, afe->base + m->ch_num_reg);
	}

	k_spin_unlock(&afe->lock, key);

	/* Store runtime params for use in start() — Linux stores these in
	 * snd_pcm_runtime which is available at trigger time.
	 */
	afe->memif_rt[memif_id].period_frames = cfg->period_frames;
	afe->memif_rt[memif_id].rate          = cfg->rate;
	afe->memif_rt[memif_id].channels      = cfg->channels;
	afe->memif_rt[memif_id].word_size     = cfg->word_size;

	/* Configure CM unit if this memif uses one — Linux: fe_hw_params calls
	 * mt8188_afe_config_cm() before mtk_afe_fe_hw_params().
	 */
	{
		int cm_id = memif_to_cm(memif_id);

		if (cm_id >= 0) {
			cm_config(afe->base, &cm_table[cm_id], cfg->channels);
		}
	}

	/* UL9 cowork setup — must run BEFORE mt8390_etdm_configure() so the
	 * cowork slave-propagation loop inside configure() fires.
	 *
	 * UL9 captures 32 channels by merging eTDM_IN1 + eTDM_IN2 through the
	 * CM0 channel-merge unit. The two ports run in co-clock mode: IN1 is
	 * the cowork master (drives BCLK/LRCK), IN2 is the slave synced to IN1.
	 *
	 * This linkage is fixed in the MT8390 hardware topology (UL9↔CM0↔
	 * IN1+IN2), so the driver sets it up internally rather than via DT or
	 * a caller API — matching how memif_to_etdm()/memif_to_cm() are also
	 * hardcoded for UL9.
	 */
	if (memif_id == MT8390_UL9) {
		/* Mark IN2 as cowork-slaved to IN1 (master), then build the
		 * master->slave linkage. After this, configuring IN1 also
		 * configures IN2 and writes both ports' COWORK_CON sync regs.
		 */
		mt8390_etdm_set_cowork_source(afe, MT8390_ETDM_IN2, MT8390_ETDM_IN1);
		afe->etdm[MT8390_ETDM_IN2].slave_mode = true;
		mt8390_etdm_update_sync_info(afe);
	} else if (memif_id == MT8390_UL3) {
		/* UL3 uses eTDM_IN2 standalone (master, no cowork). Clear any
		 * stale cowork linkage left over from a prior UL9 session so
		 * IN2 is not mistakenly treated as a slave.
		 */
		mt8390_etdm_set_cowork_source(afe, MT8390_ETDM_IN2,
					      MT8390_COWORK_ETDM_NONE);
		afe->etdm[MT8390_ETDM_IN2].slave_mode = false;
		mt8390_etdm_update_sync_info(afe);
	}

	/* Configure eTDM port(s). mt8390_etdm_configure() sets the MCLK source
	 * to a1sys/a2sys (1) for the port and all its cowork slaves, so UL9's
	 * IN1+IN2 both end up on the same timing domain for CM0 to merge them.
	 */
	etdm_id = memif_to_etdm(memif_id);
	if (etdm_id < MT8390_ETDM_NR) {
		int ret2;

		/* Apply caller-requested eTDM framing before configure():
		 *   - format + inversion (master/slave preserved)
		 *   - data mode (single-pin / multi-pin)
		 *   - TDM slot count + LRCK width override
		 *   - MCLK frequency (calibrated against the APLL via cal_mclk)
		 */
		mt8390_etdm_set_fmt(afe, etdm_id, cfg->fmt, false, false,
				    afe->etdm[etdm_id].slave_mode);
		mt8390_etdm_set_data_mode(afe, etdm_id, cfg->data_mode);
		mt8390_etdm_set_tdm_slot(afe, etdm_id, cfg->slots, cfg->lrck_width);

		ret2 = mt8390_etdm_set_sysclk(afe, etdm_id, cfg->mclk_freq,
					      cfg->mclk_dir);
		if (ret2) {
			return ret2;
		}

		/* For UL9 cowork, the slave port (IN2) must use the same framing
		 * as the master (IN1) — propagate format, data mode, slots and
		 * MCLK. IN2 stays in slave mode (set during cowork setup above).
		 * The cowork loop in etdm_configure() reads IN2's stored fmt for
		 * its CON0.
		 */
		if (memif_id == MT8390_UL9) {
			mt8390_etdm_set_fmt(afe, MT8390_ETDM_IN2, cfg->fmt,
					    false, false, true);
			mt8390_etdm_set_data_mode(afe, MT8390_ETDM_IN2,
						  cfg->data_mode);
			mt8390_etdm_set_tdm_slot(afe, MT8390_ETDM_IN2, cfg->slots,
						 cfg->lrck_width);
			ret2 = mt8390_etdm_set_sysclk(afe, MT8390_ETDM_IN2,
						      cfg->mclk_freq, cfg->mclk_dir);
			if (ret2) {
				return ret2;
			}
		}

		ret2 = mt8390_etdm_configure(afe, etdm_id, cfg);
		if (ret2) {
			return ret2;
		}
	}
	return 0;
}

/* -------------------------------------------------------------------------
 * Public API: route
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_route(const struct device *dev,
				enum mt8390_route_src src,
				enum mt8390_route_dst dst)
{
	struct mt8390_afe *afe = dev->data;

	return mt8390_afe_route(afe, src, dst);
}

/* -------------------------------------------------------------------------
 * Public API: start
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_start(const struct device *dev,
				enum mt8390_memif_id memif_id)
{
	struct mt8390_afe *afe = dev->data;
	const struct mt8390_memif_data *m;
	enum mt8390_etdm_id etdm_id;
	k_spinlock_key_t key;
	int ret;
	uint32_t val;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return -EINVAL;
	}
	m = &memif_table[memif_id];
	if (!m->active) {
		return -ENOTSUP;
	}

	etdm_id = memif_to_etdm(memif_id);

	/* Enable path clocks. The APLL timing domain is rate-selected
	 * (48k→APLL1, 44.1k→APLL2), so pass the configured rate.
	 */
	if (etdm_id == MT8390_ETDM_OUT1 || etdm_id == MT8390_ETDM_OUT2) {
		ret = mt8390_afe_enable_etdm_out_clocks(afe, etdm_id,
							afe->memif_rt[memif_id].rate);
	} else if (memif_id == MT8390_UL9) {
		ret = mt8390_afe_enable_cm0_clocks(afe, afe->memif_rt[memif_id].rate);
	} else {
		ret = mt8390_afe_enable_etdm_in_clocks(afe, etdm_id,
						       afe->memif_rt[memif_id].rate);
	}
	if (ret) {
		return ret;
	}

	/* Enable MCLK if configured.
	 * UL9 cowork: enable MCLK for both IN1 and IN2.
	 */
	if (etdm_id < MT8390_ETDM_NR) {
		ret = mt8390_etdm_enable_mclk(afe, etdm_id);
		if (ret) {
			return ret;
		}
		if (memif_id == MT8390_UL9) {
			ret = mt8390_etdm_enable_mclk(afe, MT8390_ETDM_IN2);
			if (ret) {
				return ret;
			}
		}
	}

	/* Start eTDM hardware.
	 * UL9 cowork: start both IN1 and IN2 (CM0 merges them).
	 */
	if (etdm_id < MT8390_ETDM_NR) {
		ret = mt8390_etdm_start(afe, etdm_id);
		if (ret) {
			return ret;
		}
		if (memif_id == MT8390_UL9) {
			ret = mt8390_etdm_start(afe, MT8390_ETDM_IN2);
			if (ret) {
				return ret;
			}
		}
	}

	key = k_spin_lock(&afe->lock);

	/* Enable CM unit before memif — Linux: fe_trigger calls enable_cm(true)
	 * before mtk_memif_set_enable().
	 */
	{
		int cm_id = memif_to_cm(memif_id);

		if (cm_id >= 0) {
			cm_enable(afe->base, &cm_table[cm_id], true);
		}
	}

	/* Clear agent-disable bit */
	val = sys_read32(afe->base + AUDIO_TOP_CON5);
	val &= ~BIT(m->agent_dis_shift);
	sys_write32(val, afe->base + AUDIO_TOP_CON5);

	/* Enable memif in AFE_DAC_CON0 */
	val = sys_read32(afe->base + AFE_DAC_CON0);
	val |= BIT(m->enable_shift);
	sys_write32(val, afe->base + AFE_DAC_CON0);

	/* Configure and enable period IRQ.
	 * Linux fe_trigger(TRIGGER_START) sequence:
	 *   1. set irq counter (runtime->period_size)
	 *   2. set irq fs
	 *   3. delay for uplink (capture paths only)
	 *   4. enable interrupt
	 */
	if (m->irq_id >= 0 && m->irq_id < MT8390_IRQ_NR) {
		const struct mt8390_irq_data *irq = &irq_table[m->irq_id];
		const struct mt8390_memif_rt *rt = &afe->memif_rt[memif_id];
		int fs = mt8390_afe_fs_timing(rt->rate);

		/* Step 1: write period frame count to irq_cnt_reg.
		 * Linux: counter = runtime->period_size
		 *   regmap_update_bits(irq_cnt_reg, irq_cnt_maskbit << shift,
		 *                      counter << shift)
		 */
		if (rt->period_frames > 0) {
			afe_irq_set_period(afe->base, irq, rt->period_frames);
		}

		/* Step 2: set FS in IRQ CON register (ASYS IRQs only) */
		if (fs >= 0) {
			afe_irq_set_fs(afe->base, irq, (uint32_t)fs);
		}

		k_spin_unlock(&afe->lock, key);

		/* Step 3: uplink start delay — Linux formula:
		 *   sample_delay = ((MEMIF_AXI_MINLEN+1)*64 +
		 *                   (channels*bits - 1)) / (channels*bits) + 1
		 * where MEMIF_AXI_MINLEN = 9 (register default).
		 * Applied only for capture (UL) paths.
		 */
		if (memif_id >= MT8390_UL1 && memif_id <= MT8390_UL10 &&
		    rt->rate > 0 && rt->channels > 0 && rt->word_size > 0) {
#define MEMIF_AXI_MINLEN 9
			uint32_t ch_bits = rt->channels * rt->word_size;
			uint32_t sample_delay =
				((MEMIF_AXI_MINLEN + 1) * 64 + (ch_bits - 1)) /
				ch_bits + 1;
			/* udelay(sample_delay * 1000000 / rate) in µs */
			k_busy_wait(sample_delay * 1000000U / rt->rate);
#undef MEMIF_AXI_MINLEN
		}

		/* Step 4: enable interrupt */
		k_spinlock_key_t irq_key = k_spin_lock(&afe->lock);

		afe_irq_enable(afe->base, irq);
		k_spin_unlock(&afe->lock, irq_key);
	} else {
		k_spin_unlock(&afe->lock, key);
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * Public API: stop
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_stop(const struct device *dev,
			       enum mt8390_memif_id memif_id)
{
	struct mt8390_afe *afe = dev->data;
	const struct mt8390_memif_data *m;
	enum mt8390_etdm_id etdm_id;
	k_spinlock_key_t key;
	uint32_t val;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return -EINVAL;
	}
	m = &memif_table[memif_id];
	if (!m->active) {
		return -ENOTSUP;
	}

	etdm_id = memif_to_etdm(memif_id);

	key = k_spin_lock(&afe->lock);

	/* Disable CM unit before memif — Linux: fe_trigger calls enable_cm(false)
	 * before mtk_memif_set_disable().
	 */
	{
		int cm_id = memif_to_cm(memif_id);

		if (cm_id >= 0) {
			cm_enable(afe->base, &cm_table[cm_id], false);
		}
	}

	/* Disable period IRQ first */
	if (m->irq_id >= 0 && m->irq_id < MT8390_IRQ_NR) {
		const struct mt8390_irq_data *irq = &irq_table[m->irq_id];

		afe_irq_disable(afe->base, irq);
		afe_irq_clear(afe->base, irq);
	}

	/* Disable memif */
	val = sys_read32(afe->base + AFE_DAC_CON0);
	val &= ~BIT(m->enable_shift);
	sys_write32(val, afe->base + AFE_DAC_CON0);

	/* Set agent-disable */
	val = sys_read32(afe->base + AUDIO_TOP_CON5);
	val |= BIT(m->agent_dis_shift);
	sys_write32(val, afe->base + AUDIO_TOP_CON5);

	k_spin_unlock(&afe->lock, key);

	/* Stop eTDM hardware.
	 * UL9 cowork: stop both IN2 and IN1.
	 */
	if (etdm_id < MT8390_ETDM_NR) {
		if (memif_id == MT8390_UL9) {
			mt8390_etdm_stop(afe, MT8390_ETDM_IN2);
		}
		mt8390_etdm_stop(afe, etdm_id);
	}

	/* Disable MCLK */
	if (etdm_id < MT8390_ETDM_NR) {
		mt8390_etdm_disable_mclk(afe, etdm_id);
		if (memif_id == MT8390_UL9) {
			mt8390_etdm_disable_mclk(afe, MT8390_ETDM_IN2);
		}
	}

	/* Disable path clocks — same rate-selected domain as enable. */
	if (etdm_id == MT8390_ETDM_OUT1 || etdm_id == MT8390_ETDM_OUT2) {
		mt8390_afe_disable_etdm_out_clocks(afe, etdm_id,
						   afe->memif_rt[memif_id].rate);
	} else if (memif_id == MT8390_UL9) {
		mt8390_afe_disable_cm0_clocks(afe, afe->memif_rt[memif_id].rate);
	} else {
		mt8390_afe_disable_etdm_in_clocks(afe, etdm_id,
						  afe->memif_rt[memif_id].rate);
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * Public API: set_buf / get_cur
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_set_buf(const struct device *dev,
				  enum mt8390_memif_id memif_id,
				  uint32_t base_addr, uint32_t buf_size)
{
	struct mt8390_afe *afe = dev->data;
	const struct mt8390_memif_data *m;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return -EINVAL;
	}
	m = &memif_table[memif_id];
	if (!m->active) {
		return -ENOTSUP;
	}

	/* reg_end holds the address of the last byte — Linux mtk_memif_set_addr():
	 * phys_buf_addr + dma_bytes - 1.
	 */
	sys_write32(base_addr,                    afe->base + m->reg_base);
	sys_write32(base_addr + buf_size - 1U,    afe->base + m->reg_end);

	return 0;
}

static uint32_t mt8390_afe_api_get_cur(const struct device *dev,
				       enum mt8390_memif_id memif_id)
{
	struct mt8390_afe *afe = dev->data;
	const struct mt8390_memif_data *m;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return 0;
	}
	m = &memif_table[memif_id];
	if (!m->active) {
		return 0;
	}

	return sys_read32(afe->base + m->reg_cur);
}

/* -------------------------------------------------------------------------
 * Public API: set_period_cb
 * -------------------------------------------------------------------------
 */
static int mt8390_afe_api_set_period_cb(const struct device *dev,
					enum mt8390_memif_id memif_id,
					void (*cb)(void *data), void *cb_data)
{
	struct mt8390_afe *afe = dev->data;
	k_spinlock_key_t key;

	if ((unsigned int)memif_id >= MT8390_MEMIF_NR) {
		return -EINVAL;
	}
	if (!memif_table[memif_id].active) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&afe->lock);
	afe->period_cb[memif_id].cb   = cb;
	afe->period_cb[memif_id].data = cb_data;
	k_spin_unlock(&afe->lock, key);

	return 0;
}

static const struct mt8390_afe_driver_api mt8390_afe_api = {
	.configure    = mt8390_afe_api_configure,
	.route        = mt8390_afe_api_route,
	.start        = mt8390_afe_api_start,
	.stop         = mt8390_afe_api_stop,
	.set_buf      = mt8390_afe_api_set_buf,
	.get_cur      = mt8390_afe_api_get_cur,
	.set_period_cb = mt8390_afe_api_set_period_cb,
};

/* -------------------------------------------------------------------------
 * Register defaults
 * Source: mt8188_afe_reg_defaults[] + mt8188_cg_patch[] in mt8188-afe-pcm.c
 *
 * mt8188_afe_reg_defaults:
 *   AFE_IRQ_MASK   = 0x387ffff  unmask CPU IRQs for all ASYS+AFE sources
 *   AFE_IRQ3_CON   = BIT(30)    IRQ3 period-count source select
 *   AFE_IRQ9_CON   = BIT(30)    IRQ9 period-count source select
 *   ETDM_IN1_CON4  = 0x12000100 IN1 relatch timing + clock source defaults
 *   ETDM_IN2_CON4  = 0x12000100 IN2 relatch timing + clock source defaults
 *
 * mt8188_cg_patch (applied after defaults):
 *   AUDIO_TOP_CON0 = 0xfffffffb all clock gates set (disabled) except bit 2
 *                                (AFE clock gate — bit 2 = 0 = enabled)
 *   AUDIO_TOP_CON1 = 0xfffffff8 all clock gates set except bits 0/1/2
 * -------------------------------------------------------------------------
 */
struct reg_default_entry {
	uint32_t reg;
	uint32_t val;
};

static const struct reg_default_entry afe_reg_defaults[] = {
	{ AFE_IRQ_MASK,    0x387ffffU },
	{ AFE_IRQ3_CON,    BIT(30)   },
	{ AFE_IRQ9_CON,    BIT(30)   },
	{ ETDM_IN1_CON4,   0x12000100U },
	{ ETDM_IN2_CON4,   0x12000100U },
};

static const struct reg_default_entry afe_cg_patch[] = {
	{ AUDIO_TOP_CON0, 0xfffffffbU },
	{ AUDIO_TOP_CON1, 0xfffffff8U },
};

static void mt8390_afe_init_registers(uintptr_t base)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(afe_reg_defaults); i++) {
		sys_write32(afe_reg_defaults[i].val,
			    base + afe_reg_defaults[i].reg);
	}
	for (i = 0; i < ARRAY_SIZE(afe_cg_patch); i++) {
		sys_write32(afe_cg_patch[i].val,
			    base + afe_cg_patch[i].reg);
	}
}

/* -------------------------------------------------------------------------
 * Device init
 * -------------------------------------------------------------------------
 */
struct mt8390_afe_mmio_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pinctrl_config;
#endif
};

IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(0);))

/*
 * bus_protect_enable / bus_protect_disable — raise/drop INFRACFG_AO AXI bus
 * protection around the audio subsystem. Linux mt8188 bus_protect_enable/
 * disable(): two ordered steps, each polled in the STA register for ack.
 * @infra is the INFRACFG_AO base (the clk_infra_a0 device's mapped address).
 *
 * bus_protect_step() writes one step to @set_clr_reg, then polls STA until it
 * reaches @want (the masked bits set for enable, or 0 for disable).
 */
static int bus_protect_step(uintptr_t infra, uint32_t set_clr_reg,
			    uint32_t mask, uint32_t want)
{
	sys_write32(mask, infra + set_clr_reg);

	if (!WAIT_FOR((sys_read32(infra + INFRA_TOP_AXI_PROT_EN_2_STA) & mask) == want,
		      INFRA_AXI_PROT_TIMEOUT_US, k_busy_wait(INFRA_AXI_PROT_POLL_US))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int bus_protect_enable(uintptr_t infra)
{
	int ret;

	ret = bus_protect_step(infra, INFRA_TOP_AXI_PROT_EN_2_SET,
			       INFRA_AXI_PROT_AUDIO_STEP1, INFRA_AXI_PROT_AUDIO_STEP1);
	if (ret) {
		return ret;
	}

	return bus_protect_step(infra, INFRA_TOP_AXI_PROT_EN_2_SET,
				INFRA_AXI_PROT_AUDIO_STEP2, INFRA_AXI_PROT_AUDIO_STEP2);
}

static int bus_protect_disable(uintptr_t infra)
{
	int ret;

	ret = bus_protect_step(infra, INFRA_TOP_AXI_PROT_EN_2_CLR,
			       INFRA_AXI_PROT_AUDIO_STEP2, 0);
	if (ret) {
		return ret;
	}

	return bus_protect_step(infra, INFRA_TOP_AXI_PROT_EN_2_CLR,
				INFRA_AXI_PROT_AUDIO_STEP1, 0);
}

/*
 * mt8390_afe_reset — hard-reset the audio subsystem at probe.
 * Linux mt8188 probe: bus_protect_enable() → reset_control_reset("audiosys")
 * → bus_protect_disable(). The reset pulses TOPRGU SWSYSRST bit 14 (assert
 * then deassert), each write carrying the 0x88 key in the top byte.
 *
 * Must run after the domain-sidebands SMC (so the non-secure world may touch
 * these registers) and before clock init / register defaults (the reset
 * would otherwise wipe values written afterward).
 */
static int mt8390_afe_reset(struct mt8390_afe *afe)
{
	uintptr_t toprgu;
	uint32_t val;
	int ret;

	ret = bus_protect_enable(DEVICE_MMIO_GET(afe->clk_infra_a0));
	if (ret) {
		return ret;
	}

	/* TOPRGU has no Zephyr driver — map it directly for the reset pulse. */
	device_map(&toprgu, TOPRGU_BASE_ADDR, TOPRGU_BASE_SIZE, K_MEM_CACHE_NONE);

	/* Assert: set bit 14 (with key). */
	val = (sys_read32(toprgu + WDT_SWSYSRST_OFS) | WDT_SWSYSRST_AUDIO_BIT) |
	      WDT_SWSYS_RST_KEY;
	sys_write32(val, toprgu + WDT_SWSYSRST_OFS);

	/* Deassert: clear bit 14 (with key). */
	val = (sys_read32(toprgu + WDT_SWSYSRST_OFS) & ~WDT_SWSYSRST_AUDIO_BIT) |
	      WDT_SWSYS_RST_KEY;
	sys_write32(val, toprgu + WDT_SWSYSRST_OFS);

	return bus_protect_disable(DEVICE_MMIO_GET(afe->clk_infra_a0));
}

static int mt8390_afe_init(const struct device *dev)
{
	struct mt8390_afe *afe = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	afe->base = DEVICE_MMIO_GET(dev);

#ifdef CONFIG_PINCTRL
	/* Apply default pin mux (eTDM/I2S signals) before touching the path. */
	{
		const struct mt8390_afe_mmio_cfg *cfg = dev->config;

		ret = pinctrl_apply_state(cfg->pinctrl_config, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
	}
#endif

	/* Map ADSP audio 26M clock gate — separate device @0x10b91100.
	 * No Zephyr driver for this block; map it directly.
	 */
	device_map(&afe->adsp_audio26m, 0x10b91100UL, 0x100, K_MEM_CACHE_NONE);

	afe->clk_apmixed  = DEVICE_DT_GET(DT_NODELABEL(apmixedsys));
	afe->clk_topckgen = DEVICE_DT_GET(DT_NODELABEL(topckgen));
	afe->clk_infra_a0 = DEVICE_DT_GET(DT_NODELABEL(clk_infra_a0));

	if (!device_is_ready(afe->clk_apmixed) ||
	    !device_is_ready(afe->clk_topckgen) ||
	    !device_is_ready(afe->clk_infra_a0)) {
		return -ENODEV;
	}

	/* Configure audio domain sidebands via SMC.
	 * Linux: arm_smccc_smc(MTK_SIP_AUDIO_CONTROL,
	 *                      MTK_AUDIO_SMC_OP_DOMAIN_SIDEBANDS, ...)
	 * called at the start of mt8188_afe_runtime_resume(), before
	 * enabling any clocks. This sets up TrustZone security permissions
	 * so the non-secure world (Zephyr) can access AFE registers.
	 */
	{
		struct arm_smccc_res smc_res;

		arm_smccc_smc(MTK_SIP_AUDIO_CONTROL,
			      MTK_AUDIO_SMC_OP_DOMAIN_SIDEBANDS,
			      0, 0, 0, 0, 0, 0, &smc_res);
	}

	/* Reset the audio subsystem to a known state before clock init.
	 * Linux mt8188 probe: bus_protect_enable → reset_control_reset →
	 * bus_protect_disable, run before mt8188_afe_init_clock().
	 */
	ret = mt8390_afe_reset(afe);
	if (ret) {
		return ret;
	}

	/* Enable AFE bus and HW clocks before touching registers.
	 * Linux: mt8188_afe_enable_reg_rw_clk() called after the SMC.
	 */
	ret = afe_enable_base_clocks(afe);
	if (ret) {
		return ret;
	}

	/* Write register defaults — Linux: mt8188_afe_init_registers() +
	 * regmap_register_patch(mt8188_cg_patch) called while clocks are on.
	 */
	mt8390_afe_init_registers(afe->base);

	/* Base clocks are intentionally left ENABLED after init.
	 *
	 * Unlike Linux (which gates them via pm_runtime_put_sync() and
	 * re-acquires with pm_runtime_get() inside startup/hw_params), this
	 * port has no runtime-PM. The public configure()/route()/set_buf()
	 * APIs touch AFE registers before start() re-enables clocks, so the
	 * register bus must stay clocked from probe onward — otherwise those
	 * accesses stall the bus and hang the caller.
	 */

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    mt8390_afe_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static struct mt8390_afe afe_data;

static const struct mt8390_afe_mmio_cfg afe_mmio_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	IF_ENABLED(CONFIG_PINCTRL, (.pinctrl_config = PINCTRL_DT_INST_DEV_CONFIG_GET(0),))
};

DEVICE_DT_INST_DEFINE(0, mt8390_afe_init, NULL,
		      &afe_data, &afe_mmio_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &mt8390_afe_api);
