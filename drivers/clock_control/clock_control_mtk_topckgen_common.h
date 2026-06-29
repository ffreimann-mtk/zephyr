/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MT8390 TOPCKGEN clock descriptor: covers both mux (parent select) and
 * gate (enable) in a single entry, matching the MUX_GATE_CLR_SET_UPD
 * pattern from Linux clk-mt8188-topckgen.c.
 */

#ifndef DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MTK_TOPCKGEN_COMMON_H
#define DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MTK_TOPCKGEN_COMMON_H

#include <stdbool.h>
#include <stdint.h>

/*
 * struct mtk_topck_clk - describes one TOPCKGEN mux+gate clock
 *
 * @mux_sta_reg:  mux status/read register offset (same word as gate bit)
 * @mux_clr_reg:  mux clear register offset (write 1s to clear field bits)
 * @mux_set_reg:  mux set register offset   (write 1s to set field bits)
 * @mux_shift:    LSB position of the mux selector field in the register
 * @mux_width:    number of bits in the mux selector field
 * @mux_val:      parent selector value to write (0 = clk26m reference)
 * @gate_bit:     bit position of the gate enable in mux_sta_reg word
 *                (same register word; bit set = gate enabled)
 * @upd_reg:      CLK_CFG_UPDATE register offset; written after mux change
 *                to trigger the hardware to apply the new parent selection.
 *                Linux clk-mux.c writes BIT(upd_bit) here after set_parent.
 *                0 = no update register (DIV_GATE clocks).
 * @upd_bit:      bit to set in upd_reg (self-clearing in hardware)
 *
 * For DIV_GATE clocks the gate is in a separate register:
 * @div_gate_reg: gate register offset (0 if not a DIV_GATE)
 * @div_gate_bit: gate bit in div_gate_reg
 *
 * For a pure gate-only clock (no mux), set mux_width = 0.
 *
 * @dynamic_parent: when true, clock_on() does NOT write mux_val — the parent
 *                  is selected at runtime via the .configure callback instead.
 *                  Used for clocks whose parent varies by sample rate (e.g.
 *                  A1SYS_HP → apll1_d4, and the I2SIx/I2SOx MCLK muxes). This
 *                  keeps "enable gate" and "select parent" independent so the
 *                  gate can be toggled without clobbering a configured parent.
 */
struct mtk_topck_clk {
	uint32_t mux_sta_reg;
	uint32_t mux_clr_reg;
	uint32_t mux_set_reg;
	uint8_t  mux_shift;
	uint8_t  mux_width;
	uint8_t  mux_val;
	uint8_t  gate_bit;
	bool     dynamic_parent;
	/* MUX update trigger — written after mux CLR/SET to apply parent change */
	uint32_t upd_reg;
	uint8_t  upd_bit;
	/* DIV_GATE fields — used only when div_gate_reg != 0 */
	uint32_t div_gate_reg;
	uint8_t  div_gate_bit;
};

#endif /* DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MTK_TOPCKGEN_COMMON_H */
