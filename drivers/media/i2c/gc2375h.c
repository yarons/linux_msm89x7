// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the GalaxyCore GC2375H image sensor
 *
 * The register tables and the gain handling follow the GC2375H driver of
 * the Rockchip BSP kernel:
 *   Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 * with the settings for a 19.2 MHz master clock of the Samsung Galaxy Tab A
 * 8.0 (2019) camera module.
 *
 * Copyright (C) 2026 Yaron Shahrabani
 */
#include <linux/array_size.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/*
 * The registers are 8 bit wide and banked: GC2375H_REG_PAGE selects the
 * bank. The register tables switch pages themselves, everything the driver
 * touches at run time is on page 0.
 */
#define GC2375H_REG_PAGE		CCI_REG8(0xfe)
#define GC2375H_PAGE_0			0x00
#define GC2375H_PAGE_3			0x03

#define GC2375H_REG_CHIP_ID_H		CCI_REG8(0xf0)
#define GC2375H_REG_CHIP_ID_L		CCI_REG8(0xf1)
#define GC2375H_CHIP_ID			0x2375

#define GC2375H_REG_PLL_MULT		CCI_REG8(0xf8)

/* Page 0 */
#define GC2375H_REG_EXPOSURE_H		CCI_REG8(0x03)
#define GC2375H_REG_EXPOSURE_L		CCI_REG8(0x04)
#define GC2375H_EXPOSURE_MIN		4
#define GC2375H_EXPOSURE_STEP		1
#define GC2375H_EXPOSURE_MARGIN		16

#define GC2375H_REG_HB_H		CCI_REG8(0x05)
#define GC2375H_REG_HB_L		CCI_REG8(0x06)
#define GC2375H_REG_VB_H		CCI_REG8(0x07)
#define GC2375H_REG_VB_L		CCI_REG8(0x08)
#define GC2375H_VB_MIN			16

#define GC2375H_REG_ANALOG_1C		CCI_REG8(0x1c)
#define GC2375H_REG_ANALOG_20		CCI_REG8(0x20)
#define GC2375H_REG_ANALOG_22		CCI_REG8(0x22)
#define GC2375H_REG_ANALOG_26		CCI_REG8(0x26)

#define GC2375H_REG_PREGAIN_H		CCI_REG8(0xb1)
#define GC2375H_REG_PREGAIN_L		CCI_REG8(0xb2)
#define GC2375H_REG_AGAIN_INDEX		CCI_REG8(0xb6)

#define GC2375H_REG_STREAM		CCI_REG8(0xef)
#define GC2375H_STREAM_ON		0x90
#define GC2375H_STREAM_OFF		0x00

/* Total gain in 1/64 steps */
#define GC2375H_GAIN_MIN		64
#define GC2375H_GAIN_MAX		(8 * 64)
#define GC2375H_GAIN_STEP		1
#define GC2375H_GAIN_DEFAULT		64

#define GC2375H_NATIVE_WIDTH		1600
#define GC2375H_NATIVE_HEIGHT		1200

/*
 * The line length is 2 * (horizontal blanking register + 438) pixel clock
 * periods, the frame length the window height of 1208 rows + 16 + the
 * vertical blanking register.
 */
#define GC2375H_HTS(hb)			(2 * ((hb) + 438))
#define GC2375H_VTS_OFFSET		1224
#define GC2375H_VTS_MAX			8190

#define GC2375H_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10
#define GC2375H_DATA_LANES		1

/*
 * No source gives the PLL to MIPI ratio, see struct gc2375h_clk_cfg. The
 * endpoint picks one of the values that go with the master clock in use.
 */
static const s64 gc2375h_link_freq_menu[] = {
	300000000,
	364800000,
	312000000,
};

/* Supply name and the load the vendor software requests for it, in uA */
static const struct {
	const char *name;
	int load_ua;
} gc2375h_supplies[] = {
	{ "dovdd", 0 },
	{ "dvdd", 0 },
	{ "avdd", 80000 },
};

/*
 * The PLL multiplies the master clock by the multiplier register + 1 and a
 * quarter of that is the pixel clock: both register sets give the 30 frames
 * per second their sources state with it.
 *
 * The MIPI bit rate at 19.2 MHz is one of two values:
 *  - 600 Mbit/s (300 MHz) is what the vendor software sets its receiver up
 *    for: the sensor library declares 1200 Mbit/s and the CSIPHY driver
 *    counts a lone PHY as two lanes. The libraries of the other one lane
 *    sensors of that software declare twice their lane rate the same way.
 *  - 729.6 Mbit/s (364.8 MHz) is twice the PLL output, the lowest multiple
 *    that carries a 1600 pixel line within the line time (691 Mbit/s needed).
 *    The vendor library of the GC2375A, with the same PLL registers and
 *    clock, states this value as its output pixel clock (72.96 MHz * 10).
 * The first is the receiver setting known to work, the second more likely
 * the truth. For 24 MHz only the computed value exists (624 Mbit/s).
 */
struct gc2375h_clk_cfg {
	unsigned long xclk_freq;
	unsigned long link_freq_mask;
	u32 pixel_rate;
	u32 hb;
	u32 vb_def;
	u32 exposure_def;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

struct gc2375h {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc2375h_supplies)];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;

	struct regmap *regmap;
	const struct gc2375h_clk_cfg *clk_cfg;
	unsigned int link_freq_index;
};

/* Complete sensor setup for 1600x1200 and a 19.2 MHz master clock */
static const struct cci_reg_sequence gc2375h_common_regs[] = {
	/* System, PLL */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xf7), 0x01 },
	{ CCI_REG8(0xf8), 0x12 },
	{ CCI_REG8(0xf9), 0x42 },
	{ CCI_REG8(0xfa), 0x88 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x88), 0x03 },
	/* Sensor timing and analog */
	{ CCI_REG8(0x03), 0x05 },
	{ CCI_REG8(0x04), 0x10 },
	{ CCI_REG8(0x05), 0x02 },
	{ CCI_REG8(0x06), 0x6a },
	{ CCI_REG8(0x07), 0x00 },
	{ CCI_REG8(0x08), 0xd6 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x04 },
	{ CCI_REG8(0x0b), 0x00 },
	{ CCI_REG8(0x0c), 0x14 },
	{ CCI_REG8(0x0d), 0x04 },
	{ CCI_REG8(0x0e), 0xb8 },
	{ CCI_REG8(0x0f), 0x06 },
	{ CCI_REG8(0x10), 0x48 },
	{ CCI_REG8(0x17), 0xd4 },
	{ CCI_REG8(0x1c), 0x13 },
	{ CCI_REG8(0x1d), 0x13 },
	{ CCI_REG8(0x20), 0x0b },
	{ CCI_REG8(0x21), 0x6d },
	{ CCI_REG8(0x22), 0x0c },
	{ CCI_REG8(0x25), 0xc1 },
	{ CCI_REG8(0x26), 0x0e },
	{ CCI_REG8(0x27), 0x22 },
	{ CCI_REG8(0x29), 0x5f },
	{ CCI_REG8(0x2b), 0x88 },
	{ CCI_REG8(0x2f), 0x12 },
	{ CCI_REG8(0x38), 0x86 },
	{ CCI_REG8(0x3d), 0x00 },
	{ CCI_REG8(0xcd), 0xa3 },
	{ CCI_REG8(0xce), 0x57 },
	{ CCI_REG8(0xd0), 0x09 },
	{ CCI_REG8(0xd1), 0xca },
	{ CCI_REG8(0xd2), 0x34 },
	{ CCI_REG8(0xd3), 0xbb },
	{ CCI_REG8(0xd8), 0x60 },
	{ CCI_REG8(0xe0), 0x08 },
	{ CCI_REG8(0xe1), 0x1f },
	{ CCI_REG8(0xe4), 0xf8 },
	{ CCI_REG8(0xe5), 0x0c },
	{ CCI_REG8(0xe6), 0x10 },
	{ CCI_REG8(0xe7), 0xcc },
	{ CCI_REG8(0xe8), 0x02 },
	{ CCI_REG8(0xe9), 0x01 },
	{ CCI_REG8(0xea), 0x02 },
	{ CCI_REG8(0xeb), 0x01 },
	/* Output window */
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x92), 0x04 },
	{ CCI_REG8(0x94), 0x04 },
	{ CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xb0 },
	{ CCI_REG8(0x97), 0x06 },
	{ CCI_REG8(0x98), 0x40 },
	/* Black level */
	{ CCI_REG8(0x18), 0x02 },
	{ CCI_REG8(0x1a), 0x18 },
	{ CCI_REG8(0x28), 0x00 },
	{ CCI_REG8(0x3f), 0x40 },
	{ CCI_REG8(0x40), 0x26 },
	{ CCI_REG8(0x41), 0x00 },
	{ CCI_REG8(0x43), 0x03 },
	{ CCI_REG8(0x4a), 0x00 },
	{ CCI_REG8(0x4e), 0x00 },
	{ CCI_REG8(0x4f), 0x3c },
	{ CCI_REG8(0x66), 0x00 },
	{ CCI_REG8(0x67), 0x03 },
	/* Dark sun */
	{ CCI_REG8(0x68), 0x00 },
	/* Gain */
	{ CCI_REG8(0xb0), 0x58 },
	{ CCI_REG8(0xb1), 0x01 },
	{ CCI_REG8(0xb2), 0x00 },
	{ CCI_REG8(0xb6), 0x00 },
	/* MIPI */
	{ CCI_REG8(0xef), 0x00 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x03 },
	{ CCI_REG8(0x02), 0x33 },
	{ CCI_REG8(0x03), 0x90 },
	{ CCI_REG8(0x04), 0x04 },
	{ CCI_REG8(0x05), 0x00 },
	{ CCI_REG8(0x06), 0x80 },
	{ CCI_REG8(0x11), 0x2b },
	{ CCI_REG8(0x12), 0xd0 },
	{ CCI_REG8(0x13), 0x07 },
	{ CCI_REG8(0x15), 0x00 },
	{ CCI_REG8(0x21), 0x0a },
	{ CCI_REG8(0x22), 0x06 },
	{ CCI_REG8(0x23), 0x1a },
	{ CCI_REG8(0x24), 0x03 },
	{ CCI_REG8(0x25), 0x16 },
	{ CCI_REG8(0x26), 0x09 },
	{ CCI_REG8(0x29), 0x06 },
	{ CCI_REG8(0x2a), 0x09 },
	{ CCI_REG8(0x2b), 0x0a },
	{ CCI_REG8(0xfe), 0x00 },
};

/* What differs from the table above with a 24 MHz master clock */
static const struct cci_reg_sequence gc2375h_24mhz_regs[] = {
	{ GC2375H_REG_PAGE, GC2375H_PAGE_0 },
	{ GC2375H_REG_PLL_MULT, 0x0c },
	{ GC2375H_REG_HB_H, 0x02 },
	{ GC2375H_REG_HB_L, 0x5a },
	{ GC2375H_REG_ANALOG_1C, 0x10 },
	{ GC2375H_REG_PAGE, GC2375H_PAGE_3 },
	{ CCI_REG8(0x21), 0x08 },
	{ CCI_REG8(0x22), 0x05 },
	{ CCI_REG8(0x23), 0x13 },
	{ CCI_REG8(0x24), 0x02 },
	{ CCI_REG8(0x25), 0x13 },
	{ CCI_REG8(0x26), 0x08 },
	{ CCI_REG8(0x2a), 0x08 },
	{ CCI_REG8(0x2b), 0x08 },
	{ GC2375H_REG_PAGE, GC2375H_PAGE_0 },
};

static const struct gc2375h_clk_cfg gc2375h_clk_cfgs[] = {
	{
		/* 19.2 MHz * 19: 91.2 MHz pixel clock */
		.xclk_freq = 19200000,
		.link_freq_mask = BIT(0) | BIT(1),
		.pixel_rate = 91200000,
		.hb = 0x026a,
		.vb_def = 0x00d6,
		.exposure_def = 0x0510,
	},
	{
		/* 24 MHz * 13: 78 MHz pixel clock */
		.xclk_freq = 24000000,
		.link_freq_mask = BIT(2),
		.pixel_rate = 78000000,
		.hb = 0x025a,
		.vb_def = 0x0010,
		.exposure_def = 0x0465,
		.regs = gc2375h_24mhz_regs,
		.num_regs = ARRAY_SIZE(gc2375h_24mhz_regs),
	},
};

/*
 * Analog gain steps in 1/64 and the analog settings that go with each step.
 * What a step does not cover is made up with the digital pre-gain.
 */
static const struct {
	u16 level;
	u8 reg20;
	u8 reg22;
	u8 reg26;
} gc2375h_again[] = {
	{  64, 0x0b, 0x0c, 0x0e },	/* 1.00 */
	{  92, 0x0c, 0x0e, 0x0e },	/* 1.43 */
	{ 128, 0x0c, 0x0e, 0x0e },	/* 2.00 */
	{ 182, 0x0c, 0x0e, 0x0e },	/* 2.84 */
	{ 254, 0x0c, 0x0e, 0x0e },	/* 3.97 */
	{ 363, 0x0e, 0x0e, 0x0e },	/* 5.67 */
};

static inline struct gc2375h *to_gc2375h(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc2375h, sd);
}

static int gc2375h_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc2375h *gc2375h = to_gc2375h(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(gc2375h_supplies),
				    gc2375h->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(gc2375h->xclk);
	if (ret) {
		dev_err(dev, "failed to enable the clock: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(gc2375h_supplies),
				       gc2375h->supplies);
		return ret;
	}

	fsleep(5 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc2375h->powerdown_gpio, 0);
	fsleep(1 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc2375h->reset_gpio, 1);
	fsleep(1 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc2375h->reset_gpio, 0);
	fsleep(5 * USEC_PER_MSEC);

	return 0;
}

static int gc2375h_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc2375h *gc2375h = to_gc2375h(sd);

	gpiod_set_value_cansleep(gc2375h->powerdown_gpio, 1);
	fsleep(2 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc2375h->reset_gpio, 1);
	fsleep(2 * USEC_PER_MSEC);
	clk_disable_unprepare(gc2375h->xclk);
	fsleep(1 * USEC_PER_MSEC);
	regulator_bulk_disable(ARRAY_SIZE(gc2375h_supplies),
			       gc2375h->supplies);

	return 0;
}

static int gc2375h_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = GC2375H_MBUS_CODE;

	return 0;
}

static int gc2375h_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != GC2375H_MBUS_CODE)
		return -EINVAL;

	fse->min_width = GC2375H_NATIVE_WIDTH;
	fse->max_width = GC2375H_NATIVE_WIDTH;
	fse->min_height = GC2375H_NATIVE_HEIGHT;
	fse->max_height = GC2375H_NATIVE_HEIGHT;

	return 0;
}

static int gc2375h_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = GC2375H_NATIVE_WIDTH;
		sel->r.height = GC2375H_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc2375h_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	/* The only mode: get_fmt and set_fmt both return it */
	format = v4l2_subdev_state_get_format(state, 0);
	format->width = GC2375H_NATIVE_WIDTH;
	format->height = GC2375H_NATIVE_HEIGHT;
	format->code = GC2375H_MBUS_CODE;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(format->colorspace);
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int gc2375h_set_exposure(struct gc2375h *gc2375h, u32 exposure)
{
	int ret = 0;

	cci_write(gc2375h->regmap, GC2375H_REG_PAGE, GC2375H_PAGE_0, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_EXPOSURE_H,
		  (exposure >> 8) & 0x3f, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_EXPOSURE_L, exposure & 0xff,
		  &ret);

	return ret;
}

static int gc2375h_set_gain(struct gc2375h *gc2375h, u32 gain)
{
	unsigned int i = ARRAY_SIZE(gc2375h_again) - 1;
	u32 pregain;
	int ret = 0;

	while (i && gain < gc2375h_again[i].level)
		i--;

	/* Digital pre-gain, 6 fractional bits, covers the rest */
	pregain = 64 * gain / gc2375h_again[i].level;

	cci_write(gc2375h->regmap, GC2375H_REG_PAGE, GC2375H_PAGE_0, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_ANALOG_20,
		  gc2375h_again[i].reg20, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_ANALOG_22,
		  gc2375h_again[i].reg22, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_ANALOG_26,
		  gc2375h_again[i].reg26, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_AGAIN_INDEX, i, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_PREGAIN_H, pregain >> 6, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_PREGAIN_L,
		  (pregain << 2) & 0xfc, &ret);

	return ret;
}

static int gc2375h_set_vblank(struct gc2375h *gc2375h, u32 vblank)
{
	u32 vb = GC2375H_NATIVE_HEIGHT + vblank - GC2375H_VTS_OFFSET;
	int ret = 0;

	cci_write(gc2375h->regmap, GC2375H_REG_PAGE, GC2375H_PAGE_0, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_VB_H, (vb >> 8) & 0x1f, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_VB_L, vb & 0xff, &ret);

	return ret;
}

static int gc2375h_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2375h *gc2375h = container_of(ctrl->handler, struct gc2375h,
					       ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		/* The exposure has to stay below the new frame length */
		s64 exposure_max = GC2375H_NATIVE_HEIGHT + ctrl->val -
				   GC2375H_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(gc2375h->exposure,
					       GC2375H_EXPOSURE_MIN,
					       exposure_max,
					       GC2375H_EXPOSURE_STEP,
					       min_t(s64, exposure_max,
						     gc2375h->clk_cfg->exposure_def));
		if (ret)
			return ret;
	}

	/* The registers are written when streaming starts */
	if (!pm_runtime_get_if_active(gc2375h->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc2375h_set_exposure(gc2375h, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc2375h_set_gain(gc2375h, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = gc2375h_set_vblank(gc2375h, ctrl->val);
		break;
	default:
		break;
	}

	pm_runtime_put(gc2375h->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc2375h_ctrl_ops = {
	.s_ctrl = gc2375h_set_ctrl,
};

static int gc2375h_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc2375h *gc2375h = to_gc2375h(sd);
	const struct gc2375h_clk_cfg *cfg = gc2375h->clk_cfg;
	int ret;

	ret = pm_runtime_resume_and_get(gc2375h->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(gc2375h->regmap, gc2375h_common_regs,
				  ARRAY_SIZE(gc2375h_common_regs), NULL);
	if (ret)
		goto err_rpm_put;

	if (cfg->num_regs) {
		ret = cci_multi_reg_write(gc2375h->regmap, cfg->regs,
					  cfg->num_regs, NULL);
		if (ret)
			goto err_rpm_put;
	}

	ret = __v4l2_ctrl_handler_setup(&gc2375h->ctrls);
	if (ret)
		goto err_rpm_put;

	cci_write(gc2375h->regmap, GC2375H_REG_PAGE, GC2375H_PAGE_0, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_STREAM, GC2375H_STREAM_ON,
		  &ret);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	dev_err(gc2375h->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(gc2375h->dev);
	return ret;
}

static int gc2375h_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct gc2375h *gc2375h = to_gc2375h(sd);
	int ret = 0;

	cci_write(gc2375h->regmap, GC2375H_REG_PAGE, GC2375H_PAGE_0, &ret);
	cci_write(gc2375h->regmap, GC2375H_REG_STREAM, GC2375H_STREAM_OFF,
		  &ret);
	if (ret)
		dev_err(gc2375h->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(gc2375h->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops gc2375h_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc2375h_pad_ops = {
	.enum_mbus_code = gc2375h_enum_mbus_code,
	.enum_frame_size = gc2375h_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = v4l2_subdev_get_fmt,
	.get_selection = gc2375h_get_selection,
	.enable_streams = gc2375h_enable_streams,
	.disable_streams = gc2375h_disable_streams,
};

static const struct v4l2_subdev_ops gc2375h_subdev_ops = {
	.video = &gc2375h_video_ops,
	.pad = &gc2375h_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc2375h_internal_ops = {
	.init_state = gc2375h_init_state,
};

static int gc2375h_parse_fwnode(struct gc2375h *gc2375h)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device *dev = gc2375h->dev;
	struct fwnode_handle *endpoint;
	unsigned long link_freq_bitmap;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC2375H_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "only %u data lane is supported\n",
				    GC2375H_DATA_LANES);
		goto done;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc2375h_link_freq_menu,
				       ARRAY_SIZE(gc2375h_link_freq_menu),
				       &link_freq_bitmap);
	if (ret)
		goto done;

	link_freq_bitmap &= gc2375h->clk_cfg->link_freq_mask;
	if (!link_freq_bitmap) {
		ret = dev_err_probe(dev, -EINVAL,
				    "no link frequency for a %lu Hz clock in the endpoint\n",
				    gc2375h->clk_cfg->xclk_freq);
		goto done;
	}

	gc2375h->link_freq_index = __ffs(link_freq_bitmap);

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc2375h_init_controls(struct gc2375h *gc2375h)
{
	const struct gc2375h_clk_cfg *cfg = gc2375h->clk_cfg;
	struct v4l2_ctrl_handler *hdl = &gc2375h->ctrls;
	struct v4l2_fwnode_device_properties props;
	s64 hblank = GC2375H_HTS(cfg->hb) - GC2375H_NATIVE_WIDTH;
	s64 vblank_def = GC2375H_VTS_OFFSET + cfg->vb_def -
			 GC2375H_NATIVE_HEIGHT;
	s64 vblank_min = GC2375H_VTS_OFFSET + GC2375H_VB_MIN -
			 GC2375H_NATIVE_HEIGHT;
	s64 exposure_max = GC2375H_NATIVE_HEIGHT + vblank_def -
			   GC2375H_EXPOSURE_MARGIN;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 8);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &gc2375h_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(gc2375h_link_freq_menu) - 1,
				      gc2375h->link_freq_index,
				      gc2375h_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &gc2375h_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  cfg->pixel_rate, cfg->pixel_rate, 1,
			  cfg->pixel_rate);

	ctrl = v4l2_ctrl_new_std(hdl, &gc2375h_ctrl_ops, V4L2_CID_HBLANK,
				 hblank, hblank, 1, hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &gc2375h_ctrl_ops, V4L2_CID_VBLANK, vblank_min,
			  GC2375H_VTS_MAX - GC2375H_NATIVE_HEIGHT, 1,
			  vblank_def);

	gc2375h->exposure =
		v4l2_ctrl_new_std(hdl, &gc2375h_ctrl_ops, V4L2_CID_EXPOSURE,
				  GC2375H_EXPOSURE_MIN, exposure_max,
				  GC2375H_EXPOSURE_STEP, cfg->exposure_def);

	v4l2_ctrl_new_std(hdl, &gc2375h_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC2375H_GAIN_MIN, GC2375H_GAIN_MAX,
			  GC2375H_GAIN_STEP, GC2375H_GAIN_DEFAULT);

	ret = v4l2_fwnode_device_parse(gc2375h->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &gc2375h_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	gc2375h->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int gc2375h_identify(struct gc2375h *gc2375h)
{
	u64 id_h, id_l;
	int ret = 0;

	cci_read(gc2375h->regmap, GC2375H_REG_CHIP_ID_H, &id_h, &ret);
	cci_read(gc2375h->regmap, GC2375H_REG_CHIP_ID_L, &id_l, &ret);
	if (ret)
		return dev_err_probe(gc2375h->dev, ret,
				     "failed to read the chip id\n");

	if (((id_h << 8) | id_l) != GC2375H_CHIP_ID)
		return dev_err_probe(gc2375h->dev, -ENXIO,
				     "chip id mismatch: 0x%02llx%02llx\n",
				     id_h, id_l);

	return 0;
}

static int gc2375h_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc2375h *gc2375h;
	unsigned long rate;
	unsigned int i;
	int ret;

	gc2375h = devm_kzalloc(dev, sizeof(*gc2375h), GFP_KERNEL);
	if (!gc2375h)
		return -ENOMEM;

	gc2375h->dev = dev;

	gc2375h->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc2375h->xclk))
		return dev_err_probe(dev, PTR_ERR(gc2375h->xclk),
				     "failed to get the clock\n");

	rate = clk_get_rate(gc2375h->xclk);
	for (i = 0; i < ARRAY_SIZE(gc2375h_clk_cfgs); i++) {
		if (gc2375h_clk_cfgs[i].xclk_freq == rate)
			gc2375h->clk_cfg = &gc2375h_clk_cfgs[i];
	}

	if (!gc2375h->clk_cfg)
		return dev_err_probe(dev, -EINVAL,
				     "clock rate %lu Hz not supported\n", rate);

	ret = gc2375h_parse_fwnode(gc2375h);
	if (ret)
		return ret;

	gc2375h->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc2375h->regmap))
		return dev_err_probe(dev, PTR_ERR(gc2375h->regmap),
				     "failed to init CCI\n");

	for (i = 0; i < ARRAY_SIZE(gc2375h_supplies); i++) {
		gc2375h->supplies[i].supply = gc2375h_supplies[i].name;
		gc2375h->supplies[i].init_load_uA =
			gc2375h_supplies[i].load_ua;
	}

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc2375h_supplies),
				      gc2375h->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	gc2375h->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc2375h->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc2375h->reset_gpio),
				     "failed to get the reset GPIO\n");

	gc2375h->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							  GPIOD_OUT_HIGH);
	if (IS_ERR(gc2375h->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(gc2375h->powerdown_gpio),
				     "failed to get the powerdown GPIO\n");

	v4l2_i2c_subdev_init(&gc2375h->sd, client, &gc2375h_subdev_ops);
	gc2375h->sd.internal_ops = &gc2375h_internal_ops;

	ret = gc2375h_power_on(dev);
	if (ret)
		return ret;

	ret = gc2375h_identify(gc2375h);
	if (ret)
		goto err_power_off;

	ret = gc2375h_init_controls(gc2375h);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init controls\n");
		goto err_power_off;
	}

	gc2375h->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc2375h->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc2375h->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc2375h->sd.entity, 1, &gc2375h->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init the media entity\n");
		goto err_ctrls_free;
	}

	gc2375h->sd.state_lock = gc2375h->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc2375h->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init the subdev state\n");
		goto err_entity_cleanup;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&gc2375h->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register the subdev\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_idle(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&gc2375h->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_entity_cleanup:
	media_entity_cleanup(&gc2375h->sd.entity);
err_ctrls_free:
	v4l2_ctrl_handler_free(&gc2375h->ctrls);
err_power_off:
	gc2375h_power_off(dev);
	return ret;
}

static void gc2375h_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2375h *gc2375h = to_gc2375h(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc2375h->ctrls);

	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc2375h_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id gc2375h_of_match[] = {
	{ .compatible = "galaxycore,gc2375h" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc2375h_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(gc2375h_pm_ops, gc2375h_power_off,
				 gc2375h_power_on, NULL);

static struct i2c_driver gc2375h_i2c_driver = {
	.driver = {
		.name = "gc2375h",
		.of_match_table = gc2375h_of_match,
		.pm = pm_ptr(&gc2375h_pm_ops),
	},
	.probe = gc2375h_probe,
	.remove = gc2375h_remove,
};
module_i2c_driver(gc2375h_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC2375H image sensor driver");
MODULE_AUTHOR("Yaron Shahrabani <yarons@users.noreply.github.com>");
MODULE_LICENSE("GPL");
