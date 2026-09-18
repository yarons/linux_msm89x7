// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 * Qualcomm 12nm DSI PHY and PLL, found on SDM429/SDM439.
 *
 * Ported from the msm-4.9 vendor kernel:
 *   drivers/clk/msm/mdss/mdss-dsi-pll-12nm.c
 *   drivers/clk/msm/mdss/mdss-dsi-pll-12nm-util.c
 *   drivers/video/fbdev/msm/mdss_dsi_phy_12nm.c
 *   drivers/video/fbdev/msm/msm_mdss_io_8974.c (call ordering)
 * and cross-checked against the LK bootloader implementation in
 *   platform/msm_shared/mipi_dsi_autopll_12nm.c
 *
 * Every register offset, value and sequence below comes from those sources.
 * The vendor code names only a handful of bits; where a bit has no vendor
 * name it is described by what the vendor code does with it.
 */

#include <dt-bindings/clock/qcom,dsi-phy-28nm.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/iopoll.h>
#include <linux/math64.h>

#include "dsi_phy.h"

/*
 * DSI PLL 12nm - clock diagram (eg: DSI0), after the clock tree model in the
 * vendor mdss-dsi-pll-12nm.c:
 *
 *                   dsi0post_div_clk
 *                  +--------------+     +----+
 *  dsi0vco_clk --o-| /1,2,4,..,32 |-----| /4 |--- dsi0pllbyte
 *                | +--------------+     +----+
 *                |
 *                |  dsi0gp_div_clk
 *                | +--------------+     +-----------+
 *                o-| /1,2,4,..,32 |-----| /(1..128) |--- dsi0pll
 *                  +--------------+     +-----------+
 *
 * The vendor driver models the two power-of-two dividers as a mux over six
 * fixed dividers each ("post_div_mux" and "gp_div_mux"); the mux select value
 * is log2 of the division factor, which is what is kept here as the divider
 * register value.
 *
 * None of the dividers has a register of its own that could be written at any
 * time: the post divider is encoded in analog PLL settings that depend on the
 * output frequency, and the vendor driver programs the whole PLL, including
 * the dividers, in one go right before starting it. This driver does the same:
 * .set_rate only updates a software copy, which is committed to the hardware in
 * the VCO .prepare callback. A PHY reset (done by the DSI host before every PHY
 * enable) therefore cannot lose PLL state, and no save/restore_pll_state
 * callbacks are needed.
 *
 * Boot loader handoff: the boot loader leaves the PLL running for its splash
 * screen, and a simple-framebuffer node typically holds the DSI byte and pixel
 * clocks. As soon as this PHY registers as their clock provider, the clk core
 * moves those prepare counts onto the PLL and calls the VCO .prepare while the
 * splash is still being scanned out. Like pll_vco_handoff_12nm() of the vendor
 * driver, .prepare therefore adopts a PLL that is found running and locked
 * instead of touching it, and the rates are read back from the hardware when
 * the clocks are registered.
 */

/*
 * Register map. The PLL and the PHY share one register block ("dsi_phy" in DT;
 * the vendor DT points both pll_base and the DSI "dsi_phy" region at it), so
 * everything here is an offset from phy->base. phy->pll_base is not used.
 *
 * For this draft the offsets are defined here instead of in a
 * registers/display/dsi_phy_12nm.xml file; they should move there before this
 * is submitted upstream.
 */
#define REG_DSI_12nm_PHY_T_TA_GO_TIM_COUNT			0x014
#define REG_DSI_12nm_PHY_T_TA_SURE_TIM_COUNT			0x018

#define REG_DSI_12nm_PHY_PLL_POWERUP_CTRL			0x034
#define DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR			BIT(0)
#define DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR_EN		BIT(1)

#define REG_DSI_12nm_PHY_PLL_PROP_CHRG_PUMP_CTRL		0x038
#define REG_DSI_12nm_PHY_PLL_INTEG_CHRG_PUMP_CTRL		0x03c
#define REG_DSI_12nm_PHY_PLL_ANA_TST_LOCK_ST_OVR_CTRL		0x044

#define REG_DSI_12nm_PHY_PLL_VCO_CTRL				0x048
#define DSI_12nm_PHY_PLL_VCO_CTRL_VCO_CNTRL__MASK		0x3f
/* the post divider is encoded in bits 5:4 of vco_cntrl, see post_div_cfg[] */
#define DSI_12nm_PHY_PLL_VCO_CTRL_POST_DIV__MASK		0x30
/* set unconditionally by the vendor driver, no name given */
#define DSI_12nm_PHY_PLL_VCO_CTRL_BIT6				BIT(6)

#define REG_DSI_12nm_PHY_PLL_GMP_CTRL_DIG_TST			0x04c
#define DSI_12nm_PHY_PLL_GMP_CTRL_DIG_TST_GMP_CNTRL(x)		(((x) & 0x3) << 4)

#define REG_DSI_12nm_PHY_PLL_PHA_ERR_CTRL_0			0x050
#define REG_DSI_12nm_PHY_PLL_LOCK_FILTER			0x054
#define REG_DSI_12nm_PHY_PLL_UNLOCK_FILTER			0x058
#define REG_DSI_12nm_PHY_PLL_INPUT_DIV_PLL_OVR			0x05c
#define REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_0			0x060
#define REG_DSI_12nm_PHY_PLL_INPUT_LOOP_DIV_RAT_CTRL		0x064
#define REG_DSI_12nm_PHY_PLL_PRO_DLY_RELOCK			0x06c

#define REG_DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL		0x070
/* set unconditionally by the vendor driver, no name given */
#define DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL_BIT4		BIT(4)
#define DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL_CPBIAS_CNTRL	BIT(6)

#define REG_DSI_12nm_PHY_PLL_LOCK_DET_MODE_SEL			0x074
#define REG_DSI_12nm_PHY_PLL_ANA_PROG_CTRL			0x07c

#define REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_CLKLANE		0x0c0
#define REG_DSI_12nm_PHY_HSTX_DATAREV_CTRL_CLKLANE		0x0d4
#define REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE0		0x100

#define REG_DSI_12nm_PHY_HS_FREQ_RAN_SEL			0x110
#define DSI_12nm_PHY_HS_FREQ_RAN_SEL_HSFREQRANGE__MASK		0x7f
/* set unconditionally by the vendor driver, no name given */
#define DSI_12nm_PHY_HS_FREQ_RAN_SEL_BIT7			BIT(7)

#define REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE0	0x114
#define REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE1		0x140
#define REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE1	0x154

/*
 * HS TX timing registers. The vendor driver ORs fixed bits into most of them;
 * they are not named there (by their position they look like "use the
 * programmed value" enables, but that is a guess).
 */
#define REG_DSI_12nm_PHY_HSTX_CLKLANE_REQSTATE_TIM_CTRL		0x180
#define REG_DSI_12nm_PHY_HSTX_CLKLANE_HS0STATE_TIM_CTRL		0x188
#define REG_DSI_12nm_PHY_HSTX_CLKLANE_TRALSTATE_TIM_CTRL	0x18c
#define REG_DSI_12nm_PHY_HSTX_CLKLANE_EXITSTATE_TIM_CTRL	0x190
#define REG_DSI_12nm_PHY_HSTX_CLKLANE_CLKPOSTSTATE_TIM_CTRL	0x194
#define REG_DSI_12nm_PHY_HSTX_DATALANE_REQSTATE_TIM_CTRL	0x1c0
#define REG_DSI_12nm_PHY_HSTX_DATALANE_HS0STATE_TIM_CTRL	0x1c8
#define REG_DSI_12nm_PHY_HSTX_DATALANE_TRAILSTATE_TIM_CTRL	0x1cc
#define REG_DSI_12nm_PHY_HSTX_DATALANE_EXITSTATE_TIM_CTRL	0x1d0
#define DSI_12nm_PHY_HSTX_HS0STATE_TIM_CTRL_BIT7		BIT(7)
#define DSI_12nm_PHY_HSTX_TRAILSTATE_TIM_CTRL_BIT6		BIT(6)
#define DSI_12nm_PHY_HSTX_CLKPOSTSTATE_TIM_CTRL_BIT6		BIT(6)
#define DSI_12nm_PHY_HSTX_EXITSTATE_TIM_CTRL_BIT7_6		(BIT(7) | BIT(6))

#define REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE2		0x200
#define REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE2	0x214
#define REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE3		0x240
#define REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE3	0x254
/* value written by mdss_dsi_12nm_phy_hstx_drv_ctrl() to enable the drivers */
#define DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_ENABLE		(BIT(3) | BIT(2))

#define REG_DSI_12nm_PHY_SLEWRATE_FSM_OVR_CTRL			0x280
#define REG_DSI_12nm_PHY_SLEWRATE_DDL_LOOP_CTRL			0x28c
#define REG_DSI_12nm_PHY_SLEWRATE_DDL_CYC_FRQ_ADJ_0		0x290
#define REG_DSI_12nm_PHY_PLL_PHA_ERR_CTRL_1			0x2e4
#define REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_1			0x2e8
#define REG_DSI_12nm_PHY_SLEWRATE_DDL_CYC_FRQ_ADJ_1		0x328

#define REG_DSI_12nm_PHY_SSC10					0x360
#define REG_DSI_12nm_PHY_SSC11					0x364
#define REG_DSI_12nm_PHY_SSC12					0x368
#define REG_DSI_12nm_PHY_SSC13					0x36c
#define REG_DSI_12nm_PHY_SSC14					0x370
#define REG_DSI_12nm_PHY_SSC15					0x374

#define REG_DSI_12nm_PHY_SSC0					0x394
/* value written by pll_db_commit_12nm_ssc() to turn SSC on */
#define DSI_12nm_PHY_SSC0_SSC_ENABLE				0x27
#define DSI_12nm_PHY_SSC0_GP_CLK_EN				BIT(6)

#define REG_DSI_12nm_PHY_SSC1					0x398
#define REG_DSI_12nm_PHY_SSC2					0x39c
#define REG_DSI_12nm_PHY_SSC3					0x3a0
#define REG_DSI_12nm_PHY_SSC4					0x3a4
#define REG_DSI_12nm_PHY_SSC5					0x3a8
#define REG_DSI_12nm_PHY_SSC6					0x3ac
#define REG_DSI_12nm_PHY_SSC7					0x3b0
#define REG_DSI_12nm_PHY_SSC8					0x3b4

#define REG_DSI_12nm_PHY_SSC9					0x3b8
#define DSI_12nm_PHY_SSC9_PIXEL_DIVHF__MASK			0x7f

/* major revision in bits 7:4, 0x2 on the 12nm PHY (mdss_dsi_host.c) */
#define REG_DSI_12nm_PHY_REVISION_ID3				0x3dc

#define REG_DSI_12nm_PHY_STAT0					0x3e0
#define DSI_12nm_PHY_STAT0_PLL_LOCKED				BIT(1)

#define REG_DSI_12nm_PHY_CTRL0					0x3e8
#define DSI_12nm_PHY_CTRL0_CFG_CLK_EN				BIT(0)

/*
 * SYS_CTRL is only ever written with whole-register magic values:
 *   0x09        PHY shut down (mdss_dsi_12nm_phy_shutdown(), LK PHY init)
 *   0x49, 0xc9  the two steps of the PLL enable sequence
 * and BIT(7) is tested by pll_vco_prepare_12nm() as "PHY is already enabled".
 */
#define REG_DSI_12nm_PHY_SYS_CTRL				0x3f0
#define DSI_12nm_PHY_SYS_CTRL_SHUTDOWN				0x09
#define DSI_12nm_PHY_SYS_CTRL_PLL_ENABLE_STEP1			0x49
#define DSI_12nm_PHY_SYS_CTRL_PLL_ENABLE_STEP2			0xc9
#define DSI_12nm_PHY_SYS_CTRL_PHY_ENABLED			BIT(7)

#define REG_DSI_12nm_PHY_PLL_CTRL				0x3f8
#define DSI_12nm_PHY_PLL_CTRL_CLK_SEL__MASK			0x03
#define DSI_12nm_PHY_PLL_CTRL_CLK_SEL(x)			((x) & 0x3)
/* low bits written together with the GP divider by pll_db_commit_12nm() */
#define DSI_12nm_PHY_PLL_CTRL_COMMIT_VAL			0x05
#define DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__SHIFT			5
#define DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__MASK			0x7

#define REG_DSI_12nm_PHY_REQ_DLY				0x3fc

#define VCO_REF_CLK_RATE		19200000
#define VCO_MIN_RATE			1000000000UL
#define VCO_MAX_RATE			2000000000UL

/* PLL lock polling, as done by LK: up to 50 reads, 500us apart */
#define POLL_SLEEP_US			500
#define POLL_TIMEOUT_US			(50 * POLL_SLEEP_US)

/* SSC defaults of the vendor sdm439.dtsi mdss_dsi0_pll/mdss_dsi1_pll nodes */
#define SSC_FREQ_HZ			31500
#define SSC_PPM				5000

enum dsi_pll_12nm_div_id {
	DSI_PLL_12NM_POST_DIV,
	DSI_PLL_12NM_GP_DIV,
	DSI_PLL_12NM_PIXEL_DIV,
	DSI_PLL_12NM_NUM_DIVS,
};

struct dsi_pll_12nm;

struct dsi_pll_12nm_div {
	struct clk_hw hw;
	struct dsi_pll_12nm *pll;
	enum dsi_pll_12nm_div_id id;
	const struct clk_div_table *table;
	u8 width;
};

#define to_pll_12nm_div(_hw) container_of(_hw, struct dsi_pll_12nm_div, hw)

struct dsi_pll_12nm {
	struct clk_hw clk_hw;

	struct msm_dsi_phy *phy;

	/*
	 * Software copy of the PLL configuration. Each item is read back from
	 * the hardware the first time it is needed (boot loader handoff, this
	 * happens when the clocks are registered) and is authoritative from
	 * then on; it is written out by the VCO .prepare. All users run under
	 * the clk prepare lock.
	 */
	unsigned long vco_rate;
	bool vco_rate_valid;
	u32 div_val[DSI_PLL_12NM_NUM_DIVS];
	bool div_valid[DSI_PLL_12NM_NUM_DIVS];

	bool ssc_en;
	u32 ssc_freq;
	u32 ssc_ppm;
};

#define to_pll_12nm(x)	container_of(x, struct dsi_pll_12nm, clk_hw)

/* struct dsi_pll_param of the vendor driver, minus the dividers */
struct dsi_pll_12nm_param {
	u32 hsfreqrange;
	u32 vco_cntrl;
	u32 osc_freq_target;
	u32 m_div;
	u32 prop_cntrl;
	u32 int_cntrl;
	u32 gmp_cntrl;
	u32 cpbias_cntrl;
	u32 fsm_ovr_ctrl;

	/* SSC / fractional divider */
	u32 mpll_ssc_peak_i;
	u32 mpll_stepsize_i;
	u32 mpll_mint_i;
	u32 mpll_frac_den;
	u32 mpll_frac_quot_i;
	u32 mpll_frac_rem;
};

/* /1, /2, /4, ..., /32; the register value is the vendor "mux sel" */
#define PLL_12NM_POW2_DIV_MAX_VAL	5

static const struct clk_div_table pll_12nm_pow2_div_table[] = {
	{ .val = 0, .div = 1 },
	{ .val = 1, .div = 2 },
	{ .val = 2, .div = 4 },
	{ .val = 3, .div = 8 },
	{ .val = 4, .div = 16 },
	{ .val = 5, .div = 32 },
	{ }
};

/*
 * Encoding of the post divider, indexed by the divider register value (log2
 * of the division factor), from __mdss_dsi_get_pll_vco_cntrl().
 *
 * Note that the vendor read-back helper get_post_div_mux_sel() decodes
 * { 0x30, cpbias 1 } as sel 2 (/4) although the encoder uses it for /2. The
 * encoder is what the vendor display stack (and LK) actually run with, so it
 * is taken as the reference for both directions here.
 */
static const struct {
	u8 vco_cntrl;
	u8 cpbias_cntrl;
} pll_12nm_post_div_cfg[] = {
	{ 0x00, 0 },	/* /1 */
	{ 0x30, 1 },	/* /2 */
	{ 0x10, 0 },	/* /4 */
	{ 0x20, 0 },	/* /8 */
	{ 0x30, 0 },	/* /16 */
	{ 0x00, 1 },	/* /32 */
};

/*
 * __mdss_dsi_get_hsfreqrange(): HS frequency range code by bit clock rate.
 * Each entry covers the bit clock rates up to (excluding) max_mbps; the first
 * range starts at 80 Mbps. Anything outside of 80..2500 Mbps gets 0x49.
 */
#define PLL_12NM_HSFREQRANGE_MIN_MBPS	80
#define PLL_12NM_HSFREQRANGE_DEFAULT	0x49

static const struct {
	u16 max_mbps;
	u8 hsfreqrange;
} pll_12nm_hsfreqrange[] = {
	{   90, 0x00 }, {  100, 0x10 }, {  110, 0x20 }, {  120, 0x30 },
	{  130, 0x01 }, {  140, 0x11 }, {  150, 0x21 }, {  160, 0x31 },
	{  170, 0x02 }, {  180, 0x12 }, {  190, 0x22 }, {  205, 0x32 },
	{  220, 0x03 }, {  235, 0x13 }, {  250, 0x23 }, {  275, 0x33 },
	{  300, 0x04 }, {  325, 0x14 }, {  350, 0x25 }, {  400, 0x35 },
	{  450, 0x05 }, {  500, 0x16 }, {  550, 0x26 }, {  600, 0x37 },
	{  650, 0x07 }, {  700, 0x18 }, {  750, 0x28 }, {  800, 0x39 },
	{  850, 0x09 }, {  900, 0x19 }, {  950, 0x29 }, { 1000, 0x3a },
	{ 1050, 0x0a }, { 1100, 0x1a }, { 1150, 0x2a }, { 1200, 0x3b },
	{ 1250, 0x0b }, { 1300, 0x1b }, { 1350, 0x2b }, { 1400, 0x3c },
	{ 1450, 0x0c }, { 1500, 0x1c }, { 1550, 0x2c }, { 1600, 0x3d },
	{ 1650, 0x0d }, { 1700, 0x1d }, { 1750, 0x2e }, { 1800, 0x3e },
	{ 1850, 0x0e }, { 1900, 0x1e }, { 1950, 0x2f }, { 2000, 0x3f },
	{ 2050, 0x0f }, { 2100, 0x40 }, { 2150, 0x41 }, { 2200, 0x42 },
	{ 2250, 0x43 }, { 2300, 0x44 }, { 2350, 0x45 }, { 2400, 0x46 },
	{ 2450, 0x47 }, { 2500, 0x48 },
};

/*
 * __mdss_dsi_get_pll_vco_cntrl(): VCO range, bits 1:0 of vco_cntrl, by PLL
 * output frequency (VCO rate / post divider). Each entry covers the
 * frequencies from min_mhz up to the previous entry; the first one ends at
 * 1250 MHz (inclusive). Anything outside of 44..1250 MHz gets 2.
 */
#define PLL_12NM_VCO_RANGE_MAX_MHZ	1250
#define PLL_12NM_VCO_RANGE_DEFAULT	2

static const struct {
	u16 min_mhz;
	u8 range;
} pll_12nm_vco_range[] = {
	{ 1092, 2 }, { 950, 3 }, { 712, 1 }, { 546, 2 }, { 475, 3 },
	{  356, 1 }, { 273, 2 }, { 237, 3 }, { 178, 1 }, { 136, 2 },
	{  118, 3 }, {  89, 1 }, {  68, 2 }, {  57, 3 }, {  44, 1 },
};

static u32 pll_12nm_get_hsfreqrange(u64 target_freq)
{
	u64 bitclk_rate_mhz = div_u64(target_freq * 2, 1000000);
	int i;

	if (bitclk_rate_mhz < PLL_12NM_HSFREQRANGE_MIN_MBPS)
		return PLL_12NM_HSFREQRANGE_DEFAULT;

	for (i = 0; i < ARRAY_SIZE(pll_12nm_hsfreqrange); i++)
		if (bitclk_rate_mhz < pll_12nm_hsfreqrange[i].max_mbps)
			return pll_12nm_hsfreqrange[i].hsfreqrange;

	return PLL_12NM_HSFREQRANGE_DEFAULT;
}

static void pll_12nm_get_vco_cntrl(u64 target_freq, u32 post_div,
				   u32 *vco_cntrl, u32 *cpbias_cntrl)
{
	u64 target_freq_mhz = div_u64(target_freq, 1000000);
	u32 range = PLL_12NM_VCO_RANGE_DEFAULT;
	int i;

	if (target_freq_mhz <= PLL_12NM_VCO_RANGE_MAX_MHZ) {
		for (i = 0; i < ARRAY_SIZE(pll_12nm_vco_range); i++) {
			if (target_freq_mhz >= pll_12nm_vco_range[i].min_mhz) {
				range = pll_12nm_vco_range[i].range;
				break;
			}
		}
	}

	*vco_cntrl = pll_12nm_post_div_cfg[post_div].vco_cntrl | range;
	*cpbias_cntrl = pll_12nm_post_div_cfg[post_div].cpbias_cntrl;
}

/* __mdss_dsi_get_osc_freq_target() */
static u32 pll_12nm_get_osc_freq_target(u64 target_freq)
{
	u64 target_freq_mhz = div_u64(target_freq, 1000000);

	if (target_freq_mhz <= 1000)
		return 1315;
	else if (target_freq_mhz <= 1500)
		return 1839;

	return 0;
}

/* __mdss_dsi_get_fsm_ovr_ctrl() */
static u32 pll_12nm_get_fsm_ovr_ctrl(u64 target_freq)
{
	u64 bitclk_rate_mhz = div_u64(target_freq * 2, 1000000);

	if (bitclk_rate_mhz > 1500 && bitclk_rate_mhz <= 2500)
		return 0;

	return BIT(6);
}

/* get_post_div_mux_sel(), get_gp_mux_sel() and pixel_div_get_div() */
static u32 pll_12nm_div_read_hw(struct dsi_pll_12nm *pll,
				enum dsi_pll_12nm_div_id id)
{
	void __iomem *base = pll->phy->base;
	u32 vco_cntrl, cpbias_cntrl, val;
	int i;

	switch (id) {
	case DSI_PLL_12NM_POST_DIV:
		vco_cntrl = readl(base + REG_DSI_12nm_PHY_PLL_VCO_CTRL);
		vco_cntrl &= DSI_12nm_PHY_PLL_VCO_CTRL_POST_DIV__MASK;
		cpbias_cntrl = readl(base + REG_DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL);
		cpbias_cntrl = !!(cpbias_cntrl &
				  DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL_CPBIAS_CNTRL);

		for (i = 0; i < ARRAY_SIZE(pll_12nm_post_div_cfg); i++)
			if (pll_12nm_post_div_cfg[i].vco_cntrl == vco_cntrl &&
			    pll_12nm_post_div_cfg[i].cpbias_cntrl == cpbias_cntrl)
				return i;

		return 0;
	case DSI_PLL_12NM_GP_DIV:
		val = readl(base + REG_DSI_12nm_PHY_PLL_CTRL);
		val >>= DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__SHIFT;
		val &= DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__MASK;

		return val <= PLL_12NM_POW2_DIV_MAX_VAL ? val : 0;
	case DSI_PLL_12NM_PIXEL_DIV:
		return readl(base + REG_DSI_12nm_PHY_SSC9) &
		       DSI_12nm_PHY_SSC9_PIXEL_DIVHF__MASK;
	default:
		return 0;
	}
}

/*
 * Software copy of a divider, read back from the hardware the first time it
 * is needed.
 */
static u32 pll_12nm_div_val(struct dsi_pll_12nm *pll,
			    enum dsi_pll_12nm_div_id id)
{
	if (!pll->div_valid[id]) {
		pll->div_val[id] = pll_12nm_div_read_hw(pll, id);
		pll->div_valid[id] = true;
	}

	return pll->div_val[id];
}

/* mdss_dsi_pll_12nm_calc_reg() */
static void pll_12nm_calc_reg(struct dsi_pll_12nm *pll,
			      struct dsi_pll_12nm_param *param)
{
	u32 post_div = pll_12nm_div_val(pll, DSI_PLL_12NM_POST_DIV);
	u64 target_freq = div_u64(pll->vco_rate, BIT(post_div));

	param->hsfreqrange = pll_12nm_get_hsfreqrange(target_freq);
	pll_12nm_get_vco_cntrl(target_freq, post_div, &param->vco_cntrl,
			       &param->cpbias_cntrl);
	param->osc_freq_target = pll_12nm_get_osc_freq_target(target_freq);
	param->m_div = div_u64((u64)pll->vco_rate * 4, VCO_REF_CLK_RATE);
	param->fsm_ovr_ctrl = pll_12nm_get_fsm_ovr_ctrl(target_freq);
	param->prop_cntrl = 0x05;
	param->int_cntrl = 0x00;
	param->gmp_cntrl = 0x1;
}

/*
 * __mdss_dsi_get_multi_intX100(): VCO multiplier times 100, floored in steps
 * of 0.25, and the remainder of the VCO rate below that step.
 */
static u32 pll_12nm_get_multi_intx100(u64 vco_rate, u32 *rem)
{
	const u32 quarter = VCO_REF_CLK_RATE / 4;
	u32 remainder, quarters;
	u64 temp;

	temp = div_u64_rem(vco_rate, VCO_REF_CLK_RATE, &remainder);
	quarters = remainder / quarter;

	*rem = remainder - quarters * quarter;

	return temp * 100 + quarters * 25;
}

/* mdss_dsi_pll_12nm_calc_ssc() */
static void pll_12nm_calc_ssc(struct dsi_pll_12nm *pll,
			      struct dsi_pll_12nm_param *param)
{
	u64 multi_intx100, temp;
	u32 rem1, rem2;

	multi_intx100 = pll_12nm_get_multi_intx100(pll->vco_rate, &rem1);

	temp = multi_intx100 * pll->ssc_ppm * BIT(17);
	temp = div_u64(temp, 100);	/* multi_intx100 */
	param->mpll_ssc_peak_i = div_u64(temp, 1000000);	/* ppm */

	temp = (u64)param->mpll_ssc_peak_i * pll->ssc_freq * BIT(10);
	param->mpll_stepsize_i = div_u64(temp, VCO_REF_CLK_RATE);

	param->mpll_mint_i = div_u64(multi_intx100 * 4, 100) - 32;

	param->mpll_frac_den = VCO_REF_CLK_RATE /
			       gcd(pll->vco_rate, VCO_REF_CLK_RATE);

	temp = (u64)rem1 * BIT(17);
	param->mpll_frac_quot_i = div_u64_rem(temp, VCO_REF_CLK_RATE, &rem2);

	param->mpll_frac_rem = div_u64((u64)rem2 * param->mpll_frac_den,
				       VCO_REF_CLK_RATE);

	DBG("mpll_ssc_peak_i=%u mpll_stepsize_i=%u mpll_mint_i=%u",
	    param->mpll_ssc_peak_i, param->mpll_stepsize_i,
	    param->mpll_mint_i);
	DBG("mpll_frac_den=%u mpll_frac_quot_i=%u mpll_frac_rem=%u",
	    param->mpll_frac_den, param->mpll_frac_quot_i,
	    param->mpll_frac_rem);
}

/* pll_db_commit_12nm_ssc() */
static void pll_12nm_commit_ssc(struct dsi_pll_12nm *pll,
				struct dsi_pll_12nm_param *param)
{
	void __iomem *base = pll->phy->base;

	writel(DSI_12nm_PHY_SSC0_SSC_ENABLE, base + REG_DSI_12nm_PHY_SSC0);

	writel(param->mpll_mint_i & 0xff, base + REG_DSI_12nm_PHY_SSC7);
	writel((param->mpll_mint_i & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC8);

	writel(param->mpll_ssc_peak_i & 0xff, base + REG_DSI_12nm_PHY_SSC1);
	writel((param->mpll_ssc_peak_i & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC2);
	writel((param->mpll_ssc_peak_i & 0xf0000) >> 16,
	       base + REG_DSI_12nm_PHY_SSC3);

	writel(param->mpll_stepsize_i & 0xff, base + REG_DSI_12nm_PHY_SSC4);
	writel((param->mpll_stepsize_i & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC5);
	writel((param->mpll_stepsize_i & 0x1f0000) >> 16,
	       base + REG_DSI_12nm_PHY_SSC6);

	writel(param->mpll_frac_quot_i & 0xff, base + REG_DSI_12nm_PHY_SSC10);
	writel((param->mpll_frac_quot_i & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC11);

	writel(param->mpll_frac_rem & 0xff, base + REG_DSI_12nm_PHY_SSC12);
	writel((param->mpll_frac_rem & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC13);

	writel(param->mpll_frac_den & 0xff, base + REG_DSI_12nm_PHY_SSC14);
	writel((param->mpll_frac_den & 0xff00) >> 8,
	       base + REG_DSI_12nm_PHY_SSC15);
}

/* pll_db_commit_12nm(), same register order */
static void pll_12nm_commit(struct dsi_pll_12nm *pll,
			    struct dsi_pll_12nm_param *param)
{
	void __iomem *base = pll->phy->base;
	u32 data;

	writel(DSI_12nm_PHY_CTRL0_CFG_CLK_EN, base + REG_DSI_12nm_PHY_CTRL0);
	writel(DSI_12nm_PHY_PLL_CTRL_COMMIT_VAL,
	       base + REG_DSI_12nm_PHY_PLL_CTRL);
	writel(0x01, base + REG_DSI_12nm_PHY_SLEWRATE_DDL_LOOP_CTRL);

	data = param->hsfreqrange & DSI_12nm_PHY_HS_FREQ_RAN_SEL_HSFREQRANGE__MASK;
	data |= DSI_12nm_PHY_HS_FREQ_RAN_SEL_BIT7;
	writel(data, base + REG_DSI_12nm_PHY_HS_FREQ_RAN_SEL);

	data = param->vco_cntrl & DSI_12nm_PHY_PLL_VCO_CTRL_VCO_CNTRL__MASK;
	data |= DSI_12nm_PHY_PLL_VCO_CTRL_BIT6;
	writel(data, base + REG_DSI_12nm_PHY_PLL_VCO_CTRL);

	writel(param->osc_freq_target & 0x7f,
	       base + REG_DSI_12nm_PHY_SLEWRATE_DDL_CYC_FRQ_ADJ_0);
	writel((param->osc_freq_target & 0xf80) >> 7,
	       base + REG_DSI_12nm_PHY_SLEWRATE_DDL_CYC_FRQ_ADJ_1);
	writel(0x30, base + REG_DSI_12nm_PHY_PLL_INPUT_LOOP_DIV_RAT_CTRL);

	writel(param->m_div & 0x3f, base + REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_0);
	writel((param->m_div & 0xfc0) >> 6,
	       base + REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_1);
	writel(0x60, base + REG_DSI_12nm_PHY_PLL_INPUT_DIV_PLL_OVR);

	writel(param->prop_cntrl & 0x3f,
	       base + REG_DSI_12nm_PHY_PLL_PROP_CHRG_PUMP_CTRL);
	writel(param->int_cntrl & 0x3f,
	       base + REG_DSI_12nm_PHY_PLL_INTEG_CHRG_PUMP_CTRL);
	writel(DSI_12nm_PHY_PLL_GMP_CTRL_DIG_TST_GMP_CNTRL(param->gmp_cntrl),
	       base + REG_DSI_12nm_PHY_PLL_GMP_CTRL_DIG_TST);

	data = DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL_BIT4;
	if (param->cpbias_cntrl)
		data |= DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL_CPBIAS_CNTRL;
	writel(data, base + REG_DSI_12nm_PHY_PLL_CHAR_PUMP_BIAS_CTRL);

	data = pll_12nm_div_val(pll, DSI_PLL_12NM_GP_DIV) &
	       DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__MASK;
	data <<= DSI_12nm_PHY_PLL_CTRL_GP_DIV_MUX__SHIFT;
	data |= DSI_12nm_PHY_PLL_CTRL_COMMIT_VAL;
	writel(data, base + REG_DSI_12nm_PHY_PLL_CTRL);

	writel(pll_12nm_div_val(pll, DSI_PLL_12NM_PIXEL_DIV) &
	       DSI_12nm_PHY_SSC9_PIXEL_DIVHF__MASK,
	       base + REG_DSI_12nm_PHY_SSC9);

	writel(0x03, base + REG_DSI_12nm_PHY_PLL_ANA_PROG_CTRL);
	writel(0x50, base + REG_DSI_12nm_PHY_PLL_ANA_TST_LOCK_ST_OVR_CTRL);
	writel(param->fsm_ovr_ctrl,
	       base + REG_DSI_12nm_PHY_SLEWRATE_FSM_OVR_CTRL);
	writel(0x01, base + REG_DSI_12nm_PHY_PLL_PHA_ERR_CTRL_0);
	writel(0x00, base + REG_DSI_12nm_PHY_PLL_PHA_ERR_CTRL_1);
	writel(0xff, base + REG_DSI_12nm_PHY_PLL_LOCK_FILTER);
	writel(0x03, base + REG_DSI_12nm_PHY_PLL_UNLOCK_FILTER);
	writel(0x0c, base + REG_DSI_12nm_PHY_PLL_PRO_DLY_RELOCK);
	writel(0x02, base + REG_DSI_12nm_PHY_PLL_LOCK_DET_MODE_SEL);

	if (pll->ssc_en)
		pll_12nm_commit_ssc(pll, param);
}

/*
 * The SYS_CTRL part of dsi_pll_enable_seq_12nm() and dsi_pll_relock(), which
 * is identical in both.
 */
static int pll_12nm_start(struct dsi_pll_12nm *pll)
{
	void __iomem *base = pll->phy->base;
	u32 status;
	int ret;

	writel(DSI_12nm_PHY_SYS_CTRL_PLL_ENABLE_STEP1,
	       base + REG_DSI_12nm_PHY_SYS_CTRL);
	wmb();	/* make sure the register is committed before the delay */
	udelay(5);	/* h/w recommended delay */
	writel(DSI_12nm_PHY_SYS_CTRL_PLL_ENABLE_STEP2,
	       base + REG_DSI_12nm_PHY_SYS_CTRL);
	wmb();	/* make sure the register is committed before the delay */
	usleep_range(50, 100);	/* h/w recommended delay is 50us */

	ret = readl_poll_timeout(base + REG_DSI_12nm_PHY_STAT0, status,
				 status & DSI_12nm_PHY_STAT0_PLL_LOCKED,
				 POLL_SLEEP_US, POLL_TIMEOUT_US);
	if (ret)
		DRM_DEV_ERROR(&pll->phy->pdev->dev,
			      "DSI PLL%d failed to lock, STAT0=0x%x\n",
			      pll->phy->id, status);

	return ret;
}

/*
 * dsi_pll_relock(): restart a PLL that was stopped by pll_12nm_stop() while
 * the PHY stayed enabled. The PLL configuration is still in the registers.
 */
static int pll_12nm_relock(struct dsi_pll_12nm *pll)
{
	void __iomem *base = pll->phy->base;
	u32 data;
	int ret;

	data = readl(base + REG_DSI_12nm_PHY_PLL_POWERUP_CTRL);
	data &= ~DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR_EN;
	data |= DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR;
	writel(data, base + REG_DSI_12nm_PHY_PLL_POWERUP_CTRL);
	ndelay(500);	/* h/w recommended delay */

	ret = pll_12nm_start(pll);
	if (ret)
		return ret;
	ndelay(50);	/* h/w recommended delay */

	data = readl(base + REG_DSI_12nm_PHY_PLL_CTRL);
	data |= DSI_12nm_PHY_PLL_CTRL_CLK_SEL(1);
	writel(data, base + REG_DSI_12nm_PHY_PLL_CTRL);
	ndelay(500);	/* h/w recommended delay */

	return 0;
}

/* dsi_pll_disable() */
static void pll_12nm_stop(struct dsi_pll_12nm *pll)
{
	void __iomem *base = pll->phy->base;
	u32 data;

	data = readl(base + REG_DSI_12nm_PHY_SSC0);
	data &= ~DSI_12nm_PHY_SSC0_GP_CLK_EN;
	writel(data, base + REG_DSI_12nm_PHY_SSC0);
	ndelay(500);	/* h/w recommended delay */

	data = readl(base + REG_DSI_12nm_PHY_PLL_CTRL);
	data &= ~DSI_12nm_PHY_PLL_CTRL_CLK_SEL__MASK;
	writel(data, base + REG_DSI_12nm_PHY_PLL_CTRL);
	ndelay(500);	/* h/w recommended delay */

	data = readl(base + REG_DSI_12nm_PHY_PLL_POWERUP_CTRL);
	data &= ~DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR;
	data |= DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR_EN;
	writel(data, base + REG_DSI_12nm_PHY_PLL_POWERUP_CTRL);
	ndelay(500);	/* h/w recommended delay */
}

static const u16 dsi_12nm_phy_hstx_driv_regs[] = {
	REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_CLKLANE,
	REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE0,
	REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE1,
	REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE2,
	REG_DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_LANE3,
};

/*
 * mdss_dsi_12nm_phy_hstx_drv_ctrl(). Registers that already hold the wanted
 * value are left alone, so that adopting a running link writes nothing.
 */
static void dsi_12nm_phy_hstx_drv_ctrl(struct msm_dsi_phy *phy, bool enable)
{
	void __iomem *base = phy->base;
	u32 data = enable ? DSI_12nm_PHY_HSTX_DRIV_INDATA_CTRL_ENABLE : 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(dsi_12nm_phy_hstx_driv_regs); i++)
		if (readl(base + dsi_12nm_phy_hstx_driv_regs[i]) != data)
			writel(data, base + dsi_12nm_phy_hstx_driv_regs[i]);
}

/*
 * A PLL that is running although this driver did not start it, in practice
 * the one the boot loader uses for its splash screen. The vendor handoff
 * (pll_vco_handoff_12nm()) only looks at the lock bit; SYS_CTRL and the power
 * up override are checked as well here so that a PLL which pll_12nm_stop()
 * has forced off is never taken for a running one.
 */
static bool pll_12nm_is_running(struct dsi_pll_12nm *pll)
{
	void __iomem *base = pll->phy->base;
	u32 data;

	data = readl(base + REG_DSI_12nm_PHY_SYS_CTRL);
	if (!(data & DSI_12nm_PHY_SYS_CTRL_PHY_ENABLED))
		return false;

	data = readl(base + REG_DSI_12nm_PHY_PLL_POWERUP_CTRL);
	data &= DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR_EN |
		DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR;
	if (data == DSI_12nm_PHY_PLL_POWERUP_CTRL_ONPLL_OVR_EN)
		return false;

	data = readl(base + REG_DSI_12nm_PHY_STAT0);

	return data & DSI_12nm_PHY_STAT0_PLL_LOCKED;
}

/*
 * VCO rate as programmed in the hardware. pll_vco_get_rate_12nm() only looks
 * at the integer feedback divider, which is floored to VCO_REF_CLK_RATE / 4.
 * If the SSC block is on, it also holds the fractional part: undo
 * pll_12nm_calc_ssc() to get the exact rate, and fall back to the integer one
 * if the result is not within that quarter step.
 */
static unsigned long pll_12nm_read_vco_rate(struct dsi_pll_12nm *pll)
{
	void __iomem *base = pll->phy->base;
	u32 m_div, mint, quot, rem, den;
	u64 rate, frac_rate, temp;

	m_div = readl(base + REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_1) & 0x3f;
	m_div <<= 6;
	m_div |= readl(base + REG_DSI_12nm_PHY_PLL_LOOP_DIV_RATIO_0) & 0x3f;

	rate = div_u64((u64)VCO_REF_CLK_RATE * m_div, 4);

	if ((readl(base + REG_DSI_12nm_PHY_SSC0) &
	     DSI_12nm_PHY_SSC0_SSC_ENABLE) != DSI_12nm_PHY_SSC0_SSC_ENABLE)
		return rate;

	mint = (readl(base + REG_DSI_12nm_PHY_SSC7) & 0xff) |
	       (readl(base + REG_DSI_12nm_PHY_SSC8) & 0xff) << 8;
	quot = (readl(base + REG_DSI_12nm_PHY_SSC10) & 0xff) |
	       (readl(base + REG_DSI_12nm_PHY_SSC11) & 0xff) << 8;
	rem = (readl(base + REG_DSI_12nm_PHY_SSC12) & 0xff) |
	      (readl(base + REG_DSI_12nm_PHY_SSC13) & 0xff) << 8;
	den = (readl(base + REG_DSI_12nm_PHY_SSC14) & 0xff) |
	      (readl(base + REG_DSI_12nm_PHY_SSC15) & 0xff) << 8;

	/* mint = 4 * multiplier - 32, in steps of a quarter of the reference */
	frac_rate = (u64)(VCO_REF_CLK_RATE / 4) * (mint + 32);

	/* the rest below that step, from quot and rem / den of it * 2^17 / ref */
	temp = (u64)quot * VCO_REF_CLK_RATE;
	if (den)
		temp += div_u64((u64)rem * VCO_REF_CLK_RATE, den);
	frac_rate += DIV_ROUND_CLOSEST_ULL(temp, BIT(17));

	if (frac_rate < rate || frac_rate - rate >= VCO_REF_CLK_RATE / 4)
		return rate;

	return frac_rate;
}

/*
 * VCO clock Callbacks
 */
static int dsi_pll_12nm_vco_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct dsi_pll_12nm *pll = to_pll_12nm(hw);

	DBG("DSI PLL%d rate=%lu, parent's=%lu", pll->phy->id, rate,
	    parent_rate);

	/* Programmed during prepare, like pll_vco_set_rate_12nm() */
	pll->vco_rate = rate;
	pll->vco_rate_valid = true;

	return 0;
}

static unsigned long dsi_pll_12nm_vco_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct dsi_pll_12nm *pll = to_pll_12nm(hw);

	if (!pll->vco_rate_valid) {
		pll->vco_rate = pll_12nm_read_vco_rate(pll);
		pll->vco_rate_valid = true;
	}

	DBG("DSI PLL%d returning vco rate = %lu", pll->phy->id, pll->vco_rate);

	return pll->vco_rate;
}

/*
 * Take over a running PLL, see the handoff note at the top. Nothing is written
 * if the hardware is in the state the boot loader leaves it in.
 */
static void pll_12nm_adopt(struct dsi_pll_12nm *pll)
{
	struct device *dev = &pll->phy->pdev->dev;
	unsigned long hw_rate = pll_12nm_read_vco_rate(pll);
	enum dsi_pll_12nm_div_id id;
	bool match = hw_rate == pll->vco_rate;

	for (id = 0; id < DSI_PLL_12NM_NUM_DIVS; id++)
		if (pll_12nm_div_read_hw(pll, id) != pll_12nm_div_val(pll, id))
			match = false;

	if (match)
		dev_info(dev, "DSI PLL%d: adopting the running PLL, VCO at %lu Hz\n",
			 pll->phy->id, hw_rate);
	else
		dev_warn(dev, "DSI PLL%d: adopting the running PLL at %lu Hz, but %lu Hz was set\n",
			 pll->phy->id, hw_rate, pll->vco_rate);
}

static int dsi_pll_12nm_vco_prepare(struct clk_hw *hw)
{
	struct dsi_pll_12nm *pll = to_pll_12nm(hw);
	struct device *dev = &pll->phy->pdev->dev;
	void __iomem *base = pll->phy->base;
	struct dsi_pll_12nm_param param = { };
	bool adopted = false;
	u32 data;
	int ret;

	DBG("");

	if (unlikely(pll->phy->pll_on))
		return 0;

	if (dsi_pll_12nm_vco_recalc_rate(hw, VCO_REF_CLK_RATE) == 0)
		dsi_pll_12nm_vco_set_rate(hw, pll->phy->cfg->min_pll_rate,
					  VCO_REF_CLK_RATE);

	/*
	 * pll_vco_prepare_12nm(): if the PHY is already up, the PLL was only
	 * stopped by pll_12nm_stop() and still holds its configuration.
	 * Otherwise program all of it.
	 *
	 * With the msm DSI host the first case is not expected: the PHY is
	 * reset and dsi_12nm_phy_enable() shuts it down before the link
	 * clocks, and with them this PLL, are turned on.
	 *
	 * Before either of them: a PLL that already runs must not be pulsed
	 * through SYS_CTRL under a live link, it is adopted as it is.
	 */
	data = readl(base + REG_DSI_12nm_PHY_SYS_CTRL);
	if (pll_12nm_is_running(pll)) {
		pll_12nm_adopt(pll);
		adopted = true;
		ret = 0;
	} else if (data & DSI_12nm_PHY_SYS_CTRL_PHY_ENABLED) {
		dev_info(dev, "DSI PLL%d: PHY is up, relocking\n", pll->phy->id);
		ret = pll_12nm_relock(pll);
	} else {
		pll_12nm_calc_reg(pll, &param);
		if (pll->ssc_en)
			pll_12nm_calc_ssc(pll, &param);

		DBG("DSI PLL%d vco=%lu post_div=%u gp_div=%u pixel_divhf=%u",
		    pll->phy->id, pll->vco_rate,
		    pll_12nm_div_val(pll, DSI_PLL_12NM_POST_DIV),
		    pll_12nm_div_val(pll, DSI_PLL_12NM_GP_DIV),
		    pll_12nm_div_val(pll, DSI_PLL_12NM_PIXEL_DIV));
		DBG("DSI PLL%d hsfreqrange=0x%x vco_cntrl=0x%x cpbias=%u m_div=%u",
		    pll->phy->id, param.hsfreqrange, param.vco_cntrl,
		    param.cpbias_cntrl, param.m_div);

		pll_12nm_commit(pll, &param);
		ret = pll_12nm_start(pll);
	}
	if (ret)
		return ret;

	/* pll_vco_enable_12nm(): let the pixel (GP) clock out */
	data = readl(base + REG_DSI_12nm_PHY_SSC0);
	if (!(data & DSI_12nm_PHY_SSC0_GP_CLK_EN))
		writel(data | DSI_12nm_PHY_SSC0_GP_CLK_EN,
		       base + REG_DSI_12nm_PHY_SSC0);

	/*
	 * mdss_dsi_post_clkon_cb(): the vendor driver turns the HS TX drivers
	 * on once the HS link clocks run, and off again before they stop. The
	 * closest hook here is the PLL itself; LK also enables the drivers
	 * right after the PLL has locked.
	 */
	dsi_12nm_phy_hstx_drv_ctrl(pll->phy, true);

	DBG("DSI PLL%d %s", pll->phy->id, adopted ? "adopted" : "lock success");
	pll->phy->pll_on = true;

	return 0;
}

static void dsi_pll_12nm_vco_unprepare(struct clk_hw *hw)
{
	struct dsi_pll_12nm *pll = to_pll_12nm(hw);

	DBG("");

	if (unlikely(!pll->phy->pll_on))
		return;

	/* mdss_dsi_pre_clkoff_cb() */
	dsi_12nm_phy_hstx_drv_ctrl(pll->phy, false);

	pll_12nm_stop(pll);

	pll->phy->pll_on = false;
}

/* Hardware state, for the clk core's view of a PLL the boot loader started */
static int dsi_pll_12nm_vco_is_prepared(struct clk_hw *hw)
{
	return pll_12nm_is_running(to_pll_12nm(hw));
}

static int dsi_pll_12nm_vco_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct dsi_pll_12nm *pll = to_pll_12nm(hw);

	req->rate = clamp_t(unsigned long, req->rate,
			    pll->phy->cfg->min_pll_rate,
			    pll->phy->cfg->max_pll_rate);

	return 0;
}

static const struct clk_ops clk_ops_dsi_pll_12nm_vco = {
	.determine_rate = dsi_pll_12nm_vco_determine_rate,
	.set_rate = dsi_pll_12nm_vco_set_rate,
	.recalc_rate = dsi_pll_12nm_vco_recalc_rate,
	.prepare = dsi_pll_12nm_vco_prepare,
	.unprepare = dsi_pll_12nm_vco_unprepare,
	.is_prepared = dsi_pll_12nm_vco_is_prepared,
};

/*
 * Divider clock callbacks
 */

static unsigned long dsi_pll_12nm_div_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct dsi_pll_12nm_div *div = to_pll_12nm_div(hw);

	return divider_recalc_rate(hw, parent_rate,
				   pll_12nm_div_val(div->pll, div->id),
				   div->table, 0, div->width);
}

static int dsi_pll_12nm_div_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct dsi_pll_12nm_div *div = to_pll_12nm_div(hw);

	return divider_determine_rate(hw, req, div->table, div->width, 0);
}

static int dsi_pll_12nm_div_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct dsi_pll_12nm_div *div = to_pll_12nm_div(hw);
	struct dsi_pll_12nm *pll = div->pll;
	int value;

	value = divider_get_val(rate, parent_rate, div->table, div->width, 0);
	if (value < 0)
		return value;

	DBG("DSI PLL%d div%d rate=%lu parent rate=%lu val=%d", pll->phy->id,
	    div->id, rate, parent_rate, value);

	/*
	 * Programmed during the VCO prepare, like set_post_div_mux_sel(),
	 * set_gp_mux_sel() and pixel_div_set_div()
	 */
	pll->div_val[div->id] = value;
	pll->div_valid[div->id] = true;

	return 0;
}

static const struct clk_ops clk_ops_dsi_pll_12nm_div = {
	.recalc_rate = dsi_pll_12nm_div_recalc_rate,
	.determine_rate = dsi_pll_12nm_div_determine_rate,
	.set_rate = dsi_pll_12nm_div_set_rate,
};

static struct clk_hw *pll_12nm_div_register(struct dsi_pll_12nm *pll,
					    const char *name,
					    const struct clk_hw *parent_hw,
					    unsigned long flags,
					    enum dsi_pll_12nm_div_id id)
{
	struct device *dev = &pll->phy->pdev->dev;
	struct dsi_pll_12nm_div *div;
	struct clk_init_data div_init = {
		.parent_hws = (const struct clk_hw *[]) { parent_hw },
		.num_parents = 1,
		.name = name,
		.flags = flags,
		.ops = &clk_ops_dsi_pll_12nm_div,
	};
	int ret;

	div = devm_kzalloc(dev, sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	div->pll = pll;
	div->id = id;
	if (id == DSI_PLL_12NM_PIXEL_DIV) {
		/* divides by the register value + 1: /1 to /128 */
		div->width = 7;
	} else {
		div->table = pll_12nm_pow2_div_table;
		div->width = 3;
	}
	div->hw.init = &div_init;

	ret = devm_clk_hw_register(dev, &div->hw);
	if (ret)
		return ERR_PTR(ret);

	return &div->hw;
}

static int pll_12nm_register(struct dsi_pll_12nm *pll,
			     struct clk_hw **provided_clocks)
{
	char clk_name[32];
	struct clk_init_data vco_init = {
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "ref",
		},
		.num_parents = 1,
		.name = clk_name,
		.flags = CLK_IGNORE_UNUSED,
		.ops = &clk_ops_dsi_pll_12nm_vco,
	};
	struct device *dev = &pll->phy->pdev->dev;
	struct clk_hw *hw, *post_div, *gp_div;
	int ret;

	DBG("DSI%d", pll->phy->id);

	snprintf(clk_name, sizeof(clk_name), "dsi%dvco_clk", pll->phy->id);
	pll->clk_hw.init = &vco_init;

	ret = devm_clk_hw_register(dev, &pll->clk_hw);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name), "dsi%dpost_div_clk", pll->phy->id);

	post_div = pll_12nm_div_register(pll, clk_name, &pll->clk_hw,
					 CLK_SET_RATE_PARENT,
					 DSI_PLL_12NM_POST_DIV);
	if (IS_ERR(post_div))
		return PTR_ERR(post_div);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpllbyte", pll->phy->id);

	/* DSI Byte clock = VCO_CLK / POST_DIV / 4 */
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
							 post_div,
							 CLK_SET_RATE_PARENT,
							 1, 4);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	provided_clocks[DSI_BYTE_PLL_CLK] = hw;

	snprintf(clk_name, sizeof(clk_name), "dsi%dgp_div_clk", pll->phy->id);

	/*
	 * The pixel clock path must not change the VCO rate, which is set up
	 * for the byte clock (the vendor driver uses "slave" dividers here).
	 */
	gp_div = pll_12nm_div_register(pll, clk_name, &pll->clk_hw, 0,
				       DSI_PLL_12NM_GP_DIV);
	if (IS_ERR(gp_div))
		return PTR_ERR(gp_div);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpll", pll->phy->id);

	/* DSI pixel clock = VCO_CLK / GP_DIV / PIXEL_DIV */
	hw = pll_12nm_div_register(pll, clk_name, gp_div, CLK_SET_RATE_PARENT,
				   DSI_PLL_12NM_PIXEL_DIV);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	provided_clocks[DSI_PIXEL_PLL_CLK] = hw;

	return 0;
}

static int dsi_pll_12nm_init(struct msm_dsi_phy *phy)
{
	struct platform_device *pdev = phy->pdev;
	struct dsi_pll_12nm *pll;
	int ret;

	if (!pdev)
		return -ENODEV;

	pll = devm_kzalloc(&pdev->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	DBG("PLL%d", phy->id);

	pll->phy = phy;

	/*
	 * The vendor DT enables SSC (down spread) on SDM439, and only the SSC
	 * setup programs the fractional part of the feedback divider. Without
	 * it the VCO rate is floored to a multiple of VCO_REF_CLK_RATE / 4.
	 * We might need DT props for this, like the other PHYs.
	 */
	pll->ssc_en = true;
	pll->ssc_freq = SSC_FREQ_HZ;
	pll->ssc_ppm = SSC_PPM;

	ret = pll_12nm_register(pll, phy->provided_clocks->hws);
	if (ret) {
		DRM_DEV_ERROR(&pdev->dev, "failed to register PLL: %d\n", ret);
		return ret;
	}

	phy->vco_hw = &pll->clk_hw;

	return 0;
}

/*
 * PHY timings.
 *
 * The vendor driver does not calculate the HS TX timing register values, it
 * takes them from the panel DT (qcom,mdss-dsi-panel-timings-phy-12nm, 8 values
 * made by a vendor tool), and the units of these registers are not documented.
 * None of the msm_dsi_dphy_timing_calc*() helpers reproduces the vendor values:
 * REQSTATE, TRAILSTATE and CLKPOSTSTATE come out close to hs_rqst, clk_trail/
 * hs_trail and clk_post of the v3 helper, but HS0STATE and EXITSTATE do not
 * (e.g. the data lane HS0STATE is 0 in the vendor sets below 877 Mbps where
 * every helper gives an hs_zero of 24 or more).
 *
 * Until the register semantics are known, use the vendor sets as they are and
 * pick the one made for the closest bit clock rate. The rate given with each
 * set is that of the vendor panel it was taken from (h_total * v_total * fps *
 * bpp / lanes). The three sets for the single lane SDM429W watch panels
 * (287..335 Mbps) are left out, they do not follow the pattern of the others.
 */
struct dsi_phy_12nm_timing {
	unsigned long bitclk_rate;
	/* in the order of the vendor DT property / timing_12nm[] */
	u8 clk_hs0state;	/* [0] */
	u8 clk_trailstate;	/* [1] */
	u8 clk_poststate;	/* [2] */
	u8 clk_reqstate;	/* [3] */
	u8 data_hs0state;	/* [4] */
	u8 data_trailstate;	/* [5] */
	u8 data_reqstate;	/* [6] */
	u8 exitstate;		/* [7], used for clock and data lanes */
};

static const struct dsi_phy_12nm_timing dsi_phy_12nm_timings[] = {
	/* dsi-panel-sitronix-sc7705-wxga-video.dtsi */
	{ 396207360, 0x08, 0x05, 0x09, 0x02, 0x00, 0x04, 0x02, 0x07 },
	/* dsi-panel-jd9366-txd-wxga-video.dtsi */
	{ 429235200, 0x08, 0x06, 0x09, 0x02, 0x00, 0x04, 0x02, 0x08 },
	/* dsi-panel-gh8555bl-wxga-video.dtsi, dsi-panel-ili9881c-wxga-video-*.dtsi */
	{ 430500960, 0x09, 0x06, 0x09, 0x02, 0x00, 0x04, 0x02, 0x08 },
	/* dsi-panel-gh8555bl-wxga-video-txd.dtsi */
	{ 441452160, 0x09, 0x06, 0x0a, 0x02, 0x00, 0x05, 0x02, 0x08 },
	/* dsi-panel-hx8399c-hd-plus-video.dtsi (sdm439-cdp.dtsi, sdm439-qrd.dtsi) */
	{ 455270400, 0x09, 0x06, 0x0a, 0x02, 0x00, 0x05, 0x02, 0x08 },
	/* dsi-panel-ft8201-truly-wxga-video.dtsi */
	{ 455984640, 0x09, 0x06, 0x0a, 0x02, 0x00, 0x05, 0x02, 0x08 },
	/* dsi-panel-jd9367-wxga-video-txd.dtsi */
	{ 561005280, 0x0c, 0x07, 0x0b, 0x03, 0x00, 0x06, 0x03, 0x09 },
	/* dsi-panel-truly-1080p-video.dtsi (sdm439-cdp.dtsi) */
	{ 877642560, 0x17, 0x0a, 0x0f, 0x06, 0x02, 0x08, 0x06, 0x0e },
	/* dsi-panel-nt35695b-truly-fhd-video.dtsi (sdm439-cdp.dtsi) */
	{ 886533120, 0x17, 0x0a, 0x0f, 0x06, 0x03, 0x08, 0x06, 0x0e },
	/* dsi-panel-hx8399c-fhd-plus-video.dtsi (sdm439-cdp.dtsi, sdm439-qrd.dtsi) */
	{ 924736320, 0x18, 0x0a, 0x10, 0x06, 0x03, 0x08, 0x06, 0x0e },
};

static const struct dsi_phy_12nm_timing *
dsi_12nm_phy_get_timing(struct msm_dsi_phy *phy, unsigned long bitclk_rate)
{
	const struct dsi_phy_12nm_timing *best = &dsi_phy_12nm_timings[0];
	int i;

	for (i = 1; i < ARRAY_SIZE(dsi_phy_12nm_timings); i++) {
		const struct dsi_phy_12nm_timing *t = &dsi_phy_12nm_timings[i];

		if (abs_diff(t->bitclk_rate, bitclk_rate) <
		    abs_diff(best->bitclk_rate, bitclk_rate))
			best = t;
	}

	if (abs_diff(best->bitclk_rate, bitclk_rate) > best->bitclk_rate / 10)
		dev_warn(&phy->pdev->dev,
			 "no vendor timings near %lu bps, using those for %lu bps\n",
			 bitclk_rate, best->bitclk_rate);

	return best;
}

static int dsi_12nm_phy_enable(struct msm_dsi_phy *phy,
			       struct msm_dsi_phy_clk_request *clk_req)
{
	const struct dsi_phy_12nm_timing *t;
	struct device *dev = &phy->pdev->dev;
	void __iomem *base = phy->base;
	u32 rev;

	DBG("");

	/*
	 * Neither the vendor driver nor LK set up anything for sharing one PLL
	 * between two PHYs (LK only switches the HS TX drivers of the second
	 * PHY on), so there is nothing to port for bonded DSI.
	 */
	if (phy->usecase != MSM_DSI_PHY_STANDALONE) {
		DRM_DEV_ERROR(dev, "bonded DSI is not supported on the 12nm PHY\n");
		return -EINVAL;
	}

	/*
	 * The host resets the PHY before it gets here, which takes a running
	 * PLL down without the clk core knowing. With the msm DSI host the
	 * link clocks are off at this point; anything else (say a splash
	 * framebuffer that still holds them) would leave a dead PLL behind.
	 */
	if (phy->pll_on)
		dev_warn(dev, "PHY enabled while the PLL is still prepared\n");

	/*
	 * The 12nm PHY takes care of the clock lane timing on its own: the
	 * vendor driver and LK never write DSI_CLKOUT_TIMING_CTRL of the host
	 * for it, and the register reads 0 while the boot loader drives the
	 * panel. The msm DSI host always writes it from the shared timings,
	 * so hand it zeros to end up in the same state.
	 */
	memset(&phy->timing.shared_timings, 0,
	       sizeof(phy->timing.shared_timings));

	t = dsi_12nm_phy_get_timing(phy, clk_req->bitclk_rate);

	rev = readl(base + REG_DSI_12nm_PHY_REVISION_ID3);
	if (rev >> 4 != 0x2)
		dev_warn(dev, "unexpected PHY revision 0x%x\n", rev);

	DBG("DSI%d PHY rev 0x%x, bitclk %lu, using the timings for %lu",
	    phy->id, rev, clk_req->bitclk_rate, t->bitclk_rate);

	/* "Shutdown PHY initially", only done by LK */
	writel(DSI_12nm_PHY_SYS_CTRL_SHUTDOWN, base + REG_DSI_12nm_PHY_SYS_CTRL);

	/* mdss_dsi_12nm_phy_config() */
	writel(DSI_12nm_PHY_CTRL0_CFG_CLK_EN, base + REG_DSI_12nm_PHY_CTRL0);

	/* DSI PHY clock lane timings */
	writel(t->clk_hs0state | DSI_12nm_PHY_HSTX_HS0STATE_TIM_CTRL_BIT7,
	       base + REG_DSI_12nm_PHY_HSTX_CLKLANE_HS0STATE_TIM_CTRL);
	writel(t->clk_trailstate | DSI_12nm_PHY_HSTX_TRAILSTATE_TIM_CTRL_BIT6,
	       base + REG_DSI_12nm_PHY_HSTX_CLKLANE_TRALSTATE_TIM_CTRL);
	writel(t->clk_poststate | DSI_12nm_PHY_HSTX_CLKPOSTSTATE_TIM_CTRL_BIT6,
	       base + REG_DSI_12nm_PHY_HSTX_CLKLANE_CLKPOSTSTATE_TIM_CTRL);
	writel(t->clk_reqstate,
	       base + REG_DSI_12nm_PHY_HSTX_CLKLANE_REQSTATE_TIM_CTRL);
	writel(t->exitstate | DSI_12nm_PHY_HSTX_EXITSTATE_TIM_CTRL_BIT7_6,
	       base + REG_DSI_12nm_PHY_HSTX_CLKLANE_EXITSTATE_TIM_CTRL);

	/* DSI PHY data lane timings */
	writel(t->data_hs0state | DSI_12nm_PHY_HSTX_HS0STATE_TIM_CTRL_BIT7,
	       base + REG_DSI_12nm_PHY_HSTX_DATALANE_HS0STATE_TIM_CTRL);
	writel(t->data_trailstate | DSI_12nm_PHY_HSTX_TRAILSTATE_TIM_CTRL_BIT6,
	       base + REG_DSI_12nm_PHY_HSTX_DATALANE_TRAILSTATE_TIM_CTRL);
	writel(t->data_reqstate,
	       base + REG_DSI_12nm_PHY_HSTX_DATALANE_REQSTATE_TIM_CTRL);
	writel(t->exitstate | DSI_12nm_PHY_HSTX_EXITSTATE_TIM_CTRL_BIT7_6,
	       base + REG_DSI_12nm_PHY_HSTX_DATALANE_EXITSTATE_TIM_CTRL);

	writel(0x03, base + REG_DSI_12nm_PHY_T_TA_GO_TIM_COUNT);
	writel(0x01, base + REG_DSI_12nm_PHY_T_TA_SURE_TIM_COUNT);
	writel(0x85, base + REG_DSI_12nm_PHY_REQ_DLY);

	/* DSI lane control registers */
	writel(0x00, base + REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE0);
	writel(0x00, base + REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE1);
	writel(0x00, base + REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE2);
	writel(0x00, base + REG_DSI_12nm_PHY_HSTX_READY_DLY_DATA_REV_CTRL_LANE3);
	writel(0x00, base + REG_DSI_12nm_PHY_HSTX_DATAREV_CTRL_CLKLANE);

	return 0;
}

static void dsi_12nm_phy_disable(struct msm_dsi_phy *phy)
{
	/*
	 * mdss_dsi_12nm_phy_shutdown(). The vendor driver follows this with a
	 * PHY reset through the DSI controller; the msm DSI host does that
	 * reset before the next PHY enable instead.
	 */
	writel(DSI_12nm_PHY_SYS_CTRL_SHUTDOWN,
	       phy->base + REG_DSI_12nm_PHY_SYS_CTRL);

	/* ensure that the phy is completely disabled */
	wmb();
}

/* qcom,phy-supply-entries of the vendor msm8937-mdss.dtsi, kept for SDM439 */
static const struct regulator_bulk_data dsi_phy_12nm_regulators[] = {
	{ .supply = "vddio", .init_load_uA = 100000 },
};

/*
 * There is nothing to do for the PHY regulator block on this PHY
 * (mdss_dsi_12nm_phy_regulator_enable() is empty), so has_phy_regulator is not
 * set although the DT node lists a "dsi_phy_regulator" region.
 */
const struct msm_dsi_phy_cfg dsi_phy_12nm_cfgs = {
	.regulator_data = dsi_phy_12nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_12nm_regulators),
	.ops = {
		.enable = dsi_12nm_phy_enable,
		.disable = dsi_12nm_phy_disable,
		.pll_init = dsi_pll_12nm_init,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0x1a94400, 0x1a96400 },
	.num_dsi_phy = 2,
};
