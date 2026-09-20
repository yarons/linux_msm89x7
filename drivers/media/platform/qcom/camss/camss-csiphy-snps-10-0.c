// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csiphy-snps-10-0.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY Module v10.0, Synopsys D-PHY pair
 *
 * The CSI-2 receiver of the 12 nm MSM8937 derivatives (SDM429, SDM439). Each
 * CSIPHY holds two Synopsys D-PHYs with two data lanes and a clock lane each,
 * "PHY A" and "PHY B". They receive on their own, or together as one four
 * lane receiver clocked by PHY A ("aggregate mode").
 *
 * No documentation is public. Register offsets, values and their order follow
 * the msm-4.9 vendor driver, drivers/media/platform/msm/camera_v2/sensor/
 * csiphy/msm_csiphy.c and include/msm_csiphy_10_0_0_hwreg.h, where the block
 * is called "csiphy v10.00" / "snps":
 * Copyright (c) 2011-2019, The Linux Foundation. All rights reserved.
 */

#include "camss.h"
#include "camss-csiphy.h"

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/units.h>

/* Registers of one PHY: PHY A at 0, PHY B at CSIPHY_SNPS_PHY_B */
#define CSIPHY_SNPS_PHY_B			0x800

#define CSIPHY_SNPS_RX_SYS_7			0x020
#define CSIPHY_SNPS_RX_SYS_9			0x028
#define CSIPHY_SNPS_RX_CLK_LANE_3		0x0c0
#define CSIPHY_SNPS_RX_CLK_LANE_4		0x0c4
#define CSIPHY_SNPS_RX_CLK_LANE_6		0x0c8
#define CSIPHY_SNPS_RX_CLK_LANE_7		0x0cc
#define CSIPHY_SNPS_RX_LANE_0_7			0x12c
#define CSIPHY_SNPS_RX_LANE_1_7			0x220
#define CSIPHY_SNPS_RX_STARTUP_OBS_2		0x324
#define		RX_STARTUP_OBS_2_READY		BIT(4)
#define CSIPHY_SNPS_RX_STARTUP_OVR_0		0x380
#define CSIPHY_SNPS_RX_STARTUP_OVR_1		0x384
#define CSIPHY_SNPS_RX_STARTUP_OVR_2		0x388
#define CSIPHY_SNPS_RX_STARTUP_OVR_3		0x38c
#define CSIPHY_SNPS_RX_STARTUP_OVR_4		0x390
#define CSIPHY_SNPS_RX_STARTUP_OVR_5		0x394
#define CSIPHY_SNPS_RX_DUAL_PHY_0		0x4cc
#define CSIPHY_SNPS_RX_CB_2			0x6b0

/* Registers of the wrapper around the two PHYs, once per CSIPHY */
#define CSIPHY_SNPS_SYS_CTRL			0x560
#define CSIPHY_SNPS_CTRL_1			0x564
#define CSIPHY_SNPS_CTRL_2			0x568
#define CSIPHY_SNPS_CTRL_3			0x56c
#define		CTRL_HS_FREQ_RANGE_MASK		0x7f
#define CSIPHY_SNPS_ENABLE			0x570
#define CSIPHY_SNPS_ENABLE_CLK			0x574
#define CSIPHY_SNPS_BASEDIR			0x578
#define CSIPHY_SNPS_FORCE_MODE			0x57c
#define CSIPHY_SNPS_FIFO_CTRL			0x580
#define CSIPHY_SNPS_GLBL_IRQ_CMD		0x588
#define CSIPHY_SNPS_IRQ_MASK_LANEn(n)		(0x58c + 0x4 * (n))
#define CSIPHY_SNPS_IRQ_CLEARn(n)		(0x59c + 0x4 * (n))
#define CSIPHY_SNPS_SYS_CTRL_1			0x5ac
#define CSIPHY_SNPS_IRQ_STATUSn(n)		(0x5b4 + 0x4 * (n))
#define CSIPHY_SNPS_IRQ_MASK_CLKn(n)		(0x5c8 + 0x4 * (n))
#define CSIPHY_SNPS_IRQ_CLK_CLEARn(n)		(0x5d0 + 0x4 * (n))
#define CSIPHY_SNPS_IRQ_CLK_STATUSn(n)		(0x5d8 + 0x4 * (n))

#define CSIPHY_SNPS_NUM_IRQ_REGS		4
#define CSIPHY_SNPS_NUM_CLK_IRQ_REGS		2

/* Wrapper register fields, one set of bits per PHY */
#define CSIPHY_SNPS_A_CTRL_1			0x5
#define CSIPHY_SNPS_B_CTRL_1			0xa
#define CSIPHY_SNPS_A_ENABLE			0x3
#define CSIPHY_SNPS_B_ENABLE			0xc
#define CSIPHY_SNPS_A_ENABLE_CLK		0x1
#define CSIPHY_SNPS_B_ENABLE_CLK		0x2
#define CSIPHY_SNPS_A_BASEDIR			0x1
#define CSIPHY_SNPS_B_BASEDIR			0x2
#define CSIPHY_SNPS_A_FORCE_MODE		0x3
#define CSIPHY_SNPS_B_FORCE_MODE		0xc

/*
 * Lane numbers of the endpoint, the bit numbers of the vendor lane mask:
 * data lanes 0 and 1 are PHY A, 2 and 3 PHY B, clock lane 4 is the clock of
 * PHY A and 5 the clock of PHY B.
 */
#define CSIPHY_SNPS_LANE_CLK_A			4
#define CSIPHY_SNPS_LANE_CLK_B			5
#define CSIPHY_SNPS_LANE_MASK_PHY_A		0x13
#define CSIPHY_SNPS_LANE_MASK_AGGREGATE		0x1f

/* Mbit/s per lane, SNPS_MAX_DATA_RATE_PER_LANE of the vendor driver */
#define CSIPHY_SNPS_MAX_BIT_RATE		2500

/**
 * struct csiphy_snps_band - Settings for a range of bit rates
 * @bit_rate: nominal bit rate of a lane in Mbit/s
 * @hs_freq: HS frequency range selection
 * @osc_freq: oscillator frequency target
 */
struct csiphy_snps_band {
	u16 bit_rate;
	u8 hs_freq;
	u16 osc_freq;
};

/* snps_v100_freq_values[] of msm_csiphy_10_0_0_hwreg.h, unchanged */
static const struct csiphy_snps_band csiphy_snps_bands[] = {
	{   80, 0x00, 460 }, {   90, 0x10, 460 }, {  100, 0x20, 460 },
	{  110, 0x30, 460 }, {  120, 0x01, 460 }, {  130, 0x11, 460 },
	{  140, 0x21, 460 }, {  150, 0x31, 460 }, {  160, 0x02, 460 },
	{  170, 0x12, 460 }, {  180, 0x22, 460 }, {  190, 0x32, 460 },
	{  205, 0x03, 460 }, {  220, 0x13, 460 }, {  235, 0x23, 460 },
	{  250, 0x33, 460 }, {  275, 0x04, 460 }, {  300, 0x14, 460 },
	{  325, 0x25, 460 }, {  350, 0x35, 460 }, {  400, 0x05, 460 },
	{  450, 0x16, 460 }, {  500, 0x26, 460 }, {  550, 0x37, 460 },
	{  600, 0x07, 460 }, {  650, 0x18, 460 }, {  700, 0x28, 460 },
	{  750, 0x39, 460 }, {  800, 0x09, 460 }, {  850, 0x19, 460 },
	{  900, 0x29, 460 }, {  950, 0x3a, 460 }, { 1000, 0x0a, 460 },
	{ 1050, 0x1a, 460 }, { 1100, 0x2a, 460 }, { 1150, 0x3b, 460 },
	{ 1200, 0x0b, 460 }, { 1250, 0x1b, 460 }, { 1300, 0x2b, 460 },
	{ 1350, 0x3c, 460 }, { 1400, 0x0c, 460 }, { 1450, 0x1c, 460 },
	{ 1500, 0x2c, 460 }, { 1550, 0x3d, 285 }, { 1600, 0x0d, 295 },
	{ 1650, 0x1d, 304 }, { 1700, 0x2e, 313 }, { 1750, 0x3e, 322 },
	{ 1800, 0x0e, 331 }, { 1850, 0x1e, 341 }, { 1900, 0x2f, 350 },
	{ 1950, 0x3f, 359 }, { 2000, 0x0f, 368 }, { 2050, 0x40, 377 },
	{ 2100, 0x41, 387 }, { 2150, 0x42, 396 }, { 2200, 0x43, 405 },
	{ 2250, 0x44, 414 }, { 2300, 0x45, 423 }, { 2350, 0x46, 432 },
	{ 2400, 0x47, 442 }, { 2450, 0x48, 451 }, { 2500, 0x49, 460 },
};

static void csiphy_snps_set_bits(struct csiphy_device *csiphy, u32 reg,
				 u32 bits)
{
	writel_relaxed(readl_relaxed(csiphy->base + reg) | bits,
		       csiphy->base + reg);
}

/*
 * Validate the lanes of the endpoint. The return value tells PHY A on its
 * own (false) from aggregate mode (true).
 */
static int csiphy_snps_lanes_check(struct csiphy_device *csiphy,
				   struct csiphy_lanes_cfg *c, bool *aggregate)
{
	struct device *dev = csiphy->camss->dev;
	u8 data_mask = 0;
	int i;

	for (i = 0; i < c->num_data; i++) {
		if (c->data[i].pos > 3 || (data_mask & BIT(c->data[i].pos))) {
			dev_err(dev, "csiphy%d: invalid data lane %u\n",
				csiphy->id, c->data[i].pos);
			return -EINVAL;
		}
		data_mask |= BIT(c->data[i].pos);
	}

	if (c->clk.pos == CSIPHY_SNPS_LANE_CLK_B ||
	    (c->num_data <= 2 && data_mask && !(data_mask & 0x3))) {
		dev_err(dev,
			"csiphy%d: PHY B on its own (data lanes 2 and 3, clock lane 5) is not supported\n",
			csiphy->id);
		return -EINVAL;
	}

	if (c->clk.pos != CSIPHY_SNPS_LANE_CLK_A) {
		dev_err(dev, "csiphy%d: clock lane %u, PHY A needs %u\n",
			csiphy->id, c->clk.pos, CSIPHY_SNPS_LANE_CLK_A);
		return -EINVAL;
	}

	if (c->num_data == 4) {
		*aggregate = true;
		return 0;
	}

	if ((c->num_data == 1 || c->num_data == 2) && !(data_mask & ~0x3)) {
		*aggregate = false;
		return 0;
	}

	dev_err(dev,
		"csiphy%d: %d data lanes (mask 0x%x): need lanes 0-1 for PHY A or 0-3 for both PHYs\n",
		csiphy->id, c->num_data, data_mask);

	return -EINVAL;
}

/* The vendor lane masks: all of PHY A, or all of both PHYs */
static u8 csiphy_get_lane_mask(struct csiphy_lanes_cfg *lane_cfg)
{
	if (lane_cfg->num_data == 4)
		return CSIPHY_SNPS_LANE_MASK_AGGREGATE;

	return CSIPHY_SNPS_LANE_MASK_PHY_A;
}

static void csiphy_hw_version_read(struct csiphy_device *csiphy,
				   struct device *dev)
{
	/* The vendor driver knows no version register for this block */
	dev_dbg(dev, "CSIPHY%d: Synopsys D-PHY pair (v10.0)\n", csiphy->id);
}

/*
 * csiphy_reset - The vendor driver's "reset" of this block
 *
 * It does not reset the PHYs: it toggles the interrupt command register,
 * "to enable IRQ".
 */
static void csiphy_reset(struct csiphy_device *csiphy)
{
	writel_relaxed(0x1, csiphy->base + CSIPHY_SNPS_GLBL_IRQ_CMD);
	usleep_range(5000, 8000);
	writel_relaxed(0x0, csiphy->base + CSIPHY_SNPS_GLBL_IRQ_CMD);
}

static void csiphy_snps_irq_mask(struct csiphy_device *csiphy, u8 mask)
{
	int i;

	for (i = 0; i < CSIPHY_SNPS_NUM_IRQ_REGS; i++)
		writel_relaxed(mask, csiphy->base +
			       CSIPHY_SNPS_IRQ_MASK_LANEn(i));

	for (i = 0; i < CSIPHY_SNPS_NUM_CLK_IRQ_REGS; i++)
		writel_relaxed(mask, csiphy->base +
			       CSIPHY_SNPS_IRQ_MASK_CLKn(i));
}

/*
 * The band with the nearest nominal rate, the higher one of two equally near
 * ones, as the vendor driver picks it.
 */
static const struct csiphy_snps_band *csiphy_snps_find_band(u32 bit_rate)
{
	const struct csiphy_snps_band *best = &csiphy_snps_bands[0];
	u32 best_diff = abs_diff((u32)best->bit_rate, bit_rate);
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(csiphy_snps_bands); i++) {
		u32 rate = csiphy_snps_bands[i].bit_rate;
		u32 diff = abs_diff(rate, bit_rate);

		if (diff > best_diff)
			break;

		best = &csiphy_snps_bands[i];
		best_diff = diff;
	}

	return best;
}

/* msm_csiphy_snps_2_lane_config(): the bit rate dependent part, per PHY */
static void csiphy_snps_phy_config(struct csiphy_device *csiphy, bool phy_b,
				   const struct csiphy_snps_band *band)
{
	u32 offset = phy_b ? CSIPHY_SNPS_PHY_B : 0;

	if (!phy_b) {
		writel_relaxed(0x9, csiphy->base + CSIPHY_SNPS_SYS_CTRL);
		writel_relaxed(band->hs_freq & CTRL_HS_FREQ_RANGE_MASK,
			       csiphy->base + CSIPHY_SNPS_CTRL_3);
	} else {
		writel_relaxed(0x9, csiphy->base + CSIPHY_SNPS_SYS_CTRL_1);
		writel_relaxed(band->hs_freq & CTRL_HS_FREQ_RANGE_MASK,
			       csiphy->base + CSIPHY_SNPS_CTRL_2);
	}

	csiphy_snps_set_bits(csiphy, offset + CSIPHY_SNPS_RX_SYS_7, BIT(5));
	writel_relaxed(0x43, csiphy->base + offset + CSIPHY_SNPS_RX_SYS_9);
	writel_relaxed(0x1, csiphy->base + offset +
		       CSIPHY_SNPS_RX_STARTUP_OVR_4);
	writel_relaxed(band->osc_freq & 0xff, csiphy->base + offset +
		       CSIPHY_SNPS_RX_STARTUP_OVR_2);
	writel_relaxed((band->osc_freq & 0xf00) >> 8, csiphy->base + offset +
		       CSIPHY_SNPS_RX_STARTUP_OVR_3);
	writel_relaxed(0x1, csiphy->base + offset +
		       CSIPHY_SNPS_RX_STARTUP_OVR_5);
	csiphy_snps_set_bits(csiphy, offset + CSIPHY_SNPS_RX_CB_2, BIT(6));
}

/* msm_csiphy_snps_lane_config(): PHY A as master, PHY B as slave */
static void csiphy_snps_aggregate_config(struct csiphy_device *csiphy)
{
	const u32 b = CSIPHY_SNPS_PHY_B;
	u32 val;

	writel_relaxed(0x1, csiphy->base + CSIPHY_SNPS_RX_DUAL_PHY_0);
	writel_relaxed(0x0, csiphy->base + b + CSIPHY_SNPS_RX_DUAL_PHY_0);

	csiphy_snps_set_bits(csiphy, CSIPHY_SNPS_RX_LANE_0_7, BIT(5));
	csiphy_snps_set_bits(csiphy, b + CSIPHY_SNPS_RX_LANE_0_7, BIT(5));
	csiphy_snps_set_bits(csiphy, CSIPHY_SNPS_RX_LANE_1_7, BIT(5));
	csiphy_snps_set_bits(csiphy, b + CSIPHY_SNPS_RX_LANE_1_7, BIT(5));

	writel_relaxed(0x0, csiphy->base + CSIPHY_SNPS_RX_CLK_LANE_7);
	writel_relaxed(BIT(3), csiphy->base + b + CSIPHY_SNPS_RX_CLK_LANE_7);

	csiphy_snps_set_bits(csiphy, b + CSIPHY_SNPS_RX_STARTUP_OVR_0,
			     BIT(0) | BIT(1));

	val = readl_relaxed(csiphy->base + b + CSIPHY_SNPS_RX_STARTUP_OVR_1);
	val &= ~BIT(0);
	val |= BIT(1);
	writel_relaxed(val, csiphy->base + b + CSIPHY_SNPS_RX_STARTUP_OVR_1);

	csiphy_snps_set_bits(csiphy, CSIPHY_SNPS_RX_CLK_LANE_6, BIT(2));

	val = readl_relaxed(csiphy->base + b + CSIPHY_SNPS_RX_CLK_LANE_6);
	val |= BIT(3) | BIT(7);
	val &= ~BIT(2);
	writel_relaxed(val, csiphy->base + b + CSIPHY_SNPS_RX_CLK_LANE_6);

	csiphy_snps_set_bits(csiphy, b + CSIPHY_SNPS_RX_CLK_LANE_3, BIT(7));
	writel_relaxed(0xa, csiphy->base + b + CSIPHY_SNPS_RX_CLK_LANE_4);

	writel_relaxed(0x7e, csiphy->base + CSIPHY_SNPS_FIFO_CTRL);
	writel_relaxed(0x7f, csiphy->base + CSIPHY_SNPS_FIFO_CTRL);
}

static int csiphy_lanes_enable(struct csiphy_device *csiphy,
			       struct csiphy_config *cfg,
			       s64 link_freq, u8 lane_mask)
{
	struct csiphy_lanes_cfg *c = &cfg->csi2->lane_cfg;
	struct device *dev = csiphy->camss->dev;
	const struct csiphy_snps_band *band;
	bool aggregate;
	u32 bit_rate;
	u32 val;
	int ret;

	ret = csiphy_snps_lanes_check(csiphy, c, &aggregate);
	if (ret)
		return ret;

	/* Bit rate of one lane in Mbit/s */
	bit_rate = div_u64(2 * link_freq, HZ_PER_MHZ);
	if (!bit_rate || bit_rate > CSIPHY_SNPS_MAX_BIT_RATE) {
		dev_err(dev, "csiphy%d: unsupported link frequency %lld Hz\n",
			csiphy->id, link_freq);
		return -EINVAL;
	}

	band = csiphy_snps_find_band(bit_rate);

	dev_dbg(dev,
		"csiphy%d: %s, %u Mbit/s per lane: band %u, hs_freq 0x%02x, osc_freq %u\n",
		csiphy->id, aggregate ? "PHY A + B" : "PHY A", bit_rate,
		band->bit_rate, band->hs_freq, band->osc_freq);

	/*
	 * The caller has routed the clock of PHY A to the CSID (low nibble of
	 * the clock mux register). In aggregate mode the vendor driver sets
	 * the nibble of PHY B to the same CSID.
	 */
	if (aggregate && csiphy->base_clk_mux) {
		val = readl_relaxed(csiphy->base_clk_mux);
		val &= ~0xf0;
		val |= cfg->csid_id << 4;
		writel_relaxed(val, csiphy->base_clk_mux);

		/* Enforce reg write ordering between clk mux & lane enabling */
		wmb();
	}

	csiphy_snps_phy_config(csiphy, false, band);
	if (aggregate)
		csiphy_snps_phy_config(csiphy, true, band);

	csiphy_snps_irq_mask(csiphy, 0xff);

	val = CSIPHY_SNPS_A_FORCE_MODE;
	if (aggregate)
		val |= CSIPHY_SNPS_B_FORCE_MODE;
	writel_relaxed(val, csiphy->base + CSIPHY_SNPS_FORCE_MODE);

	if (aggregate)
		csiphy_snps_aggregate_config(csiphy);

	val = CSIPHY_SNPS_A_ENABLE;
	if (aggregate)
		val |= CSIPHY_SNPS_B_ENABLE;
	writel_relaxed(val, csiphy->base + CSIPHY_SNPS_ENABLE);

	val = CSIPHY_SNPS_A_BASEDIR;
	if (aggregate)
		val |= CSIPHY_SNPS_B_BASEDIR;
	writel_relaxed(val, csiphy->base + CSIPHY_SNPS_BASEDIR);

	val = CSIPHY_SNPS_A_ENABLE_CLK;
	if (aggregate)
		val |= CSIPHY_SNPS_B_ENABLE_CLK;
	writel_relaxed(val, csiphy->base + CSIPHY_SNPS_ENABLE_CLK);

	val = CSIPHY_SNPS_A_CTRL_1;
	if (aggregate)
		val |= CSIPHY_SNPS_B_CTRL_1;
	writel_relaxed(val, csiphy->base + CSIPHY_SNPS_CTRL_1);

	/* The vendor driver looks at PHY A six times, 100 us apart */
	ret = readl_poll_timeout(csiphy->base + CSIPHY_SNPS_RX_STARTUP_OBS_2,
				 val, val & RX_STARTUP_OBS_2_READY, 100, 1000);
	if (ret) {
		dev_err(dev,
			"csiphy%d: PHY start-up failed, RX_STARTUP_OBS_2 0x%02x\n",
			csiphy->id, val);
		return ret;
	}

	writel_relaxed(0x0, csiphy->base + CSIPHY_SNPS_FORCE_MODE);

	return 0;
}

/*
 * csiphy_lanes_disable - Stop receiving
 *
 * The vendor driver masks the interrupts and leaves the PHYs as they are,
 * until the clocks go. Nothing is known about a power down sequence.
 */
static void csiphy_lanes_disable(struct csiphy_device *csiphy,
				 struct csiphy_config *cfg)
{
	csiphy_snps_irq_mask(csiphy, 0x0);
}

static irqreturn_t csiphy_isr(int irq, void *dev)
{
	struct csiphy_device *csiphy = dev;
	u32 val;
	int i;

	for (i = 0; i < CSIPHY_SNPS_NUM_IRQ_REGS; i++) {
		val = readl_relaxed(csiphy->base + CSIPHY_SNPS_IRQ_STATUSn(i));
		writel_relaxed(val, csiphy->base + CSIPHY_SNPS_IRQ_CLEARn(i));
		writel_relaxed(0x0, csiphy->base + CSIPHY_SNPS_IRQ_CLEARn(i));
		if (val)
			dev_dbg_ratelimited(csiphy->camss->dev,
					    "csiphy%d: IRQ status %d = 0x%02x\n",
					    csiphy->id, i, val);
	}

	for (i = 0; i < CSIPHY_SNPS_NUM_CLK_IRQ_REGS; i++) {
		val = readl_relaxed(csiphy->base +
				    CSIPHY_SNPS_IRQ_CLK_STATUSn(i));
		writel_relaxed(val, csiphy->base +
			       CSIPHY_SNPS_IRQ_CLK_CLEARn(i));
		writel_relaxed(0x0, csiphy->base +
			       CSIPHY_SNPS_IRQ_CLK_CLEARn(i));
		if (val)
			dev_dbg_ratelimited(csiphy->camss->dev,
					    "csiphy%d: clock IRQ status %d = 0x%02x\n",
					    csiphy->id, i, val);
	}

	writel_relaxed(0x1, csiphy->base + CSIPHY_SNPS_GLBL_IRQ_CMD);
	writel_relaxed(0x0, csiphy->base + CSIPHY_SNPS_GLBL_IRQ_CMD);

	return IRQ_HANDLED;
}

static int csiphy_init(struct csiphy_device *csiphy)
{
	return 0;
}

const struct csiphy_hw_ops csiphy_ops_snps_10_0 = {
	.get_lane_mask = csiphy_get_lane_mask,
	.hw_version_read = csiphy_hw_version_read,
	.reset = csiphy_reset,
	.lanes_enable = csiphy_lanes_enable,
	.lanes_disable = csiphy_lanes_disable,
	.isr = csiphy_isr,
	.init = csiphy_init,
};
