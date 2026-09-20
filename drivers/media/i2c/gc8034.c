// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the GalaxyCore GC8034 image sensor
 *
 * Register tables and the gain table follow the GC8034 driver of the
 * Rockchip BSP kernel:
 *   Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 * with the four lane settings of the Samsung Galaxy Tab A 8.0 (2019)
 * camera module.
 *
 * Copyright (C) 2026 Yaron Shahrabani
 */
#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math.h>
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
 * The registers are 8 bit wide and banked: GC8034_REG_PAGE selects the bank
 * and carries the block resets in its upper bits. Registers 0xf0 to 0xff are
 * reachable from every page. The register tables switch pages themselves,
 * everything the driver touches at run time is on page 0.
 */
#define GC8034_REG_PAGE			CCI_REG8(0xfe)
#define GC8034_PAGE_0			0x00
#define GC8034_PAGE_1			0x01

#define GC8034_REG_CHIP_ID_H		CCI_REG8(0xf0)
#define GC8034_REG_CHIP_ID_L		CCI_REG8(0xf1)
#define GC8034_CHIP_ID			0x8044

/* Page 0 */
#define GC8034_REG_EXPOSURE_H		CCI_REG8(0x03)
#define GC8034_REG_EXPOSURE_L		CCI_REG8(0x04)
#define GC8034_EXPOSURE_MIN		4
/* The sensor only takes even exposure values */
#define GC8034_EXPOSURE_STEP		2
#define GC8034_EXPOSURE_MARGIN		16
#define GC8034_EXPOSURE_DEFAULT		0x08c6

#define GC8034_REG_VB_H			CCI_REG8(0x07)
#define GC8034_REG_VB_L			CCI_REG8(0x08)
#define GC8034_VB_MIN			32
#define GC8034_VB_MAX			0x1fff

#define GC8034_REG_STREAM		CCI_REG8(0x3f)
#define GC8034_STREAM_ON_4LANE		0xd0
#define GC8034_STREAM_OFF		0x00

#define GC8034_REG_TEST_PATTERN		CCI_REG8(0x8c)
#define GC8034_TEST_PATTERN_EN		BIT(0)

#define GC8034_REG_PREGAIN_H		CCI_REG8(0xb1)
#define GC8034_REG_PREGAIN_L		CCI_REG8(0xb2)
#define GC8034_REG_AGAIN_INDEX		CCI_REG8(0xb6)

/* Total gain in 1/64 steps */
#define GC8034_GAIN_MIN			64
#define GC8034_GAIN_MAX			(16 * 64)
#define GC8034_GAIN_STEP		1
#define GC8034_GAIN_DEFAULT		64

#define GC8034_NATIVE_WIDTH		3264
#define GC8034_NATIVE_HEIGHT		2448

/*
 * All modes read the same 2464 sensor rows, line length and frame length are
 * counted in periods of the (unbinned) pixel clock. The frame length is
 * window height + 20 + the vertical blanking register.
 */
#define GC8034_HTS			4272
#define GC8034_VTS_OFFSET		2484
#define GC8034_PIXEL_RATE		(320 * HZ_PER_MHZ)

#define GC8034_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define GC8034_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10
#define GC8034_DATA_LANES		4

static const char * const gc8034_test_pattern_menu[] = {
	"Disabled",
	"Sensor Test Image",
};

static const s64 gc8034_link_freq_menu[] = {
	336 * HZ_PER_MHZ,
	168 * HZ_PER_MHZ,
};

/* Supply name and the load the vendor software requests for it, in uA */
static const struct {
	const char *name;
	int load_ua;
} gc8034_supplies[] = {
	{ "dovdd", 0 },
	{ "avdd", 80000 },
	{ "dvdd", 200000 },
	{ "vaf", 100000 },
};

struct gc8034 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc8034_supplies)];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	struct regmap *regmap;
	unsigned long link_freq_bitmap;
};

struct gc8034_mode {
	u32 width;
	u32 height;
	u32 link_freq_index;
	u32 vts_def;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

/*
 * Written before every mode table: PLL for a 24 MHz clock, sensor timing, analog,
 * ISP blocks, black level and MIPI D-PHY on four lanes, with the full size
 * window. The mode tables repeat what differs between the modes.
 */
static const struct cci_reg_sequence gc8034_common_regs[] = {
	/* System, PLL */
	{ CCI_REG8(0xf2), 0x00 },
	{ CCI_REG8(0xf4), 0x80 },
	{ CCI_REG8(0xf5), 0x19 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x63 },
	{ CCI_REG8(0xfa), 0x45 },
	{ CCI_REG8(0xf9), 0x00 },
	{ CCI_REG8(0xf7), 0x95 },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0xea },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x03), 0x9a },
	{ CCI_REG8(0x18), 0x07 },
	{ CCI_REG8(0x01), 0x07 },
	{ CCI_REG8(0xfc), 0xee },
	/* Sensor timing and analog */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x88), 0x03 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x3f), 0x00 },
	{ CCI_REG8(0x03), 0x08 },
	{ CCI_REG8(0x04), 0xc6 },
	{ CCI_REG8(0x05), 0x02 },
	{ CCI_REG8(0x06), 0x16 },
	{ CCI_REG8(0x07), 0x00 },
	{ CCI_REG8(0x08), 0x20 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x3a },
	{ CCI_REG8(0x0b), 0x00 },
	{ CCI_REG8(0x0c), 0x04 },
	{ CCI_REG8(0x0d), 0x09 },
	{ CCI_REG8(0x0e), 0xa0 },
	{ CCI_REG8(0x0f), 0x0c },
	{ CCI_REG8(0x10), 0xd4 },
	{ CCI_REG8(0x17), 0xc0 },
	{ CCI_REG8(0x18), 0x02 },
	{ CCI_REG8(0x19), 0x17 },
	{ CCI_REG8(0x1e), 0x50 },
	{ CCI_REG8(0x1f), 0x80 },
	{ CCI_REG8(0x21), 0x4c },
	{ CCI_REG8(0x25), 0x00 },
	{ CCI_REG8(0x28), 0x4a },
	{ CCI_REG8(0x2d), 0x89 },
	{ CCI_REG8(0xca), 0x02 },
	{ CCI_REG8(0xcb), 0x00 },
	{ CCI_REG8(0xcc), 0x39 },
	{ CCI_REG8(0xce), 0xd0 },
	{ CCI_REG8(0xcf), 0x93 },
	{ CCI_REG8(0xd0), 0x1b },
	{ CCI_REG8(0xd1), 0xaa },
	{ CCI_REG8(0xd2), 0xc3 },
	{ CCI_REG8(0xd8), 0x40 },
	{ CCI_REG8(0xd9), 0xff },
	{ CCI_REG8(0xda), 0x0e },
	{ CCI_REG8(0xdb), 0xb0 },
	{ CCI_REG8(0xdc), 0x0e },
	{ CCI_REG8(0xde), 0x08 },
	{ CCI_REG8(0xe4), 0xc6 },
	{ CCI_REG8(0xe5), 0x08 },
	{ CCI_REG8(0xe6), 0x10 },
	{ CCI_REG8(0xed), 0x2a },
	{ CCI_REG8(0xfe), 0x02 },
	{ CCI_REG8(0x59), 0x02 },
	{ CCI_REG8(0x5a), 0x04 },
	{ CCI_REG8(0x5b), 0x08 },
	{ CCI_REG8(0x5c), 0x20 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x1a), 0x09 },
	{ CCI_REG8(0x1d), 0x13 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	/* Analog, gain dependent part for 1x */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x20), 0x55 },
	{ CCI_REG8(0x33), 0x83 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xdf), 0x06 },
	{ CCI_REG8(0xe7), 0x18 },
	{ CCI_REG8(0xe8), 0x20 },
	{ CCI_REG8(0xe9), 0x16 },
	{ CCI_REG8(0xea), 0x17 },
	{ CCI_REG8(0xeb), 0x50 },
	{ CCI_REG8(0xec), 0x6c },
	{ CCI_REG8(0xed), 0x9b },
	{ CCI_REG8(0xee), 0xd8 },
	/* ISP blocks and output window */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x80), 0x13 },
	{ CCI_REG8(0x84), 0x01 },
	{ CCI_REG8(0x89), 0x03 },
	{ CCI_REG8(0x8d), 0x03 },
	{ CCI_REG8(0x8f), 0x14 },
	{ CCI_REG8(0xad), 0x00 },
	{ CCI_REG8(0xc2), 0x7f },
	{ CCI_REG8(0xc3), 0xff },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x92), 0x08 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x09 },
	{ CCI_REG8(0x95), 0x09 },
	{ CCI_REG8(0x96), 0x90 },
	{ CCI_REG8(0x97), 0x0c },
	{ CCI_REG8(0x98), 0xc0 },
	/* Gain */
	{ CCI_REG8(0xb0), 0x90 },
	{ CCI_REG8(0xb1), 0x01 },
	{ CCI_REG8(0xb2), 0x00 },
	{ CCI_REG8(0xb6), 0x00 },
	/* Black level */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x40), 0x22 },
	{ CCI_REG8(0x41), 0x20 },
	{ CCI_REG8(0x42), 0x02 },
	{ CCI_REG8(0x43), 0x08 },
	{ CCI_REG8(0x4e), 0x0f },
	{ CCI_REG8(0x4f), 0xf0 },
	{ CCI_REG8(0x58), 0x80 },
	{ CCI_REG8(0x59), 0x80 },
	{ CCI_REG8(0x5a), 0x80 },
	{ CCI_REG8(0x5b), 0x80 },
	{ CCI_REG8(0x5c), 0x00 },
	{ CCI_REG8(0x5d), 0x00 },
	{ CCI_REG8(0x5e), 0x00 },
	{ CCI_REG8(0x5f), 0x00 },
	{ CCI_REG8(0x6b), 0x01 },
	{ CCI_REG8(0x6c), 0x00 },
	{ CCI_REG8(0x6d), 0x0c },
	/* White balance offset, dark sun, defect correction */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xbf), 0x40 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x68), 0x77 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x60), 0x00 },
	{ CCI_REG8(0x61), 0x10 },
	{ CCI_REG8(0x62), 0x60 },
	{ CCI_REG8(0x63), 0x30 },
	{ CCI_REG8(0x64), 0x00 },
	/* Lens shading correction */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xa8), 0x60 },
	{ CCI_REG8(0xa2), 0xd1 },
	{ CCI_REG8(0xc8), 0x57 },
	{ CCI_REG8(0xa1), 0xb8 },
	{ CCI_REG8(0xa3), 0x91 },
	{ CCI_REG8(0xc0), 0x50 },
	{ CCI_REG8(0xd0), 0x05 },
	{ CCI_REG8(0xd1), 0xb2 },
	{ CCI_REG8(0xd2), 0x1f },
	{ CCI_REG8(0xd3), 0x00 },
	{ CCI_REG8(0xd4), 0x00 },
	{ CCI_REG8(0xd5), 0x00 },
	{ CCI_REG8(0xd6), 0x00 },
	{ CCI_REG8(0xd7), 0x00 },
	{ CCI_REG8(0xd8), 0x00 },
	{ CCI_REG8(0xd9), 0x00 },
	{ CCI_REG8(0xa4), 0x10 },
	{ CCI_REG8(0xa5), 0x20 },
	{ CCI_REG8(0xa6), 0x60 },
	{ CCI_REG8(0xa7), 0x80 },
	{ CCI_REG8(0xab), 0x18 },
	{ CCI_REG8(0xc7), 0xc0 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xdc), 0x00 },
	{ CCI_REG8(0xdd), 0x00 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x20), 0x02 },
	{ CCI_REG8(0x21), 0x02 },
	{ CCI_REG8(0x23), 0x42 },
	/* MIPI */
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x02), 0x03 },
	{ CCI_REG8(0x04), 0x80 },
	{ CCI_REG8(0x11), 0x2b },
	{ CCI_REG8(0x12), 0xf0 },
	{ CCI_REG8(0x13), 0x0f },
	{ CCI_REG8(0x15), 0x10 },
	{ CCI_REG8(0x16), 0x29 },
	{ CCI_REG8(0x17), 0xff },
	{ CCI_REG8(0x19), 0xaa },
	{ CCI_REG8(0x1a), 0x02 },
	{ CCI_REG8(0x21), 0x05 },
	{ CCI_REG8(0x22), 0x06 },
	{ CCI_REG8(0x23), 0x2b },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x25), 0x12 },
	{ CCI_REG8(0x26), 0x07 },
	{ CCI_REG8(0x29), 0x07 },
	{ CCI_REG8(0x2a), 0x12 },
	{ CCI_REG8(0x2b), 0x07 },
	{ CCI_REG8(0xfe), 0x00 },
};

static const struct cci_reg_sequence gc8034_3264x2448_regs[] = {
	{ CCI_REG8(0xf2), 0x00 },
	{ CCI_REG8(0xf4), 0x80 },
	{ CCI_REG8(0xf5), 0x19 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x63 },
	{ CCI_REG8(0xfa), 0x45 },
	{ CCI_REG8(0xf9), 0x00 },
	{ CCI_REG8(0xf7), 0x95 },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0xea },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x03), 0x9a },
	{ CCI_REG8(0x18), 0x07 },
	{ CCI_REG8(0x01), 0x07 },
	{ CCI_REG8(0xfc), 0xee },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x3f), 0x00 },
	{ CCI_REG8(0x08), 0x20 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x3a },
	{ CCI_REG8(0x0d), 0x09 },
	{ CCI_REG8(0x0e), 0xa0 },
	{ CCI_REG8(0x17), 0xc0 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x80), 0x13 },
	{ CCI_REG8(0xad), 0x00 },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x92), 0x08 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x09 },
	{ CCI_REG8(0x95), 0x09 },
	{ CCI_REG8(0x96), 0x90 },
	{ CCI_REG8(0x97), 0x0c },
	{ CCI_REG8(0x98), 0xc0 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xd3), 0x00 },
	{ CCI_REG8(0xd4), 0x00 },
	{ CCI_REG8(0xd7), 0x00 },
	{ CCI_REG8(0xdc), 0x00 },
	{ CCI_REG8(0xdd), 0x00 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x02), 0x03 },
	{ CCI_REG8(0x04), 0x80 },
	{ CCI_REG8(0x11), 0x2b },
	{ CCI_REG8(0x12), 0xf0 },
	{ CCI_REG8(0x13), 0x0f },
	{ CCI_REG8(0x15), 0x10 },
	{ CCI_REG8(0x16), 0x29 },
	{ CCI_REG8(0x17), 0xff },
	{ CCI_REG8(0x19), 0xaa },
	{ CCI_REG8(0x1a), 0x02 },
	{ CCI_REG8(0x21), 0x05 },
	{ CCI_REG8(0x22), 0x06 },
	{ CCI_REG8(0x23), 0x16 },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x25), 0x12 },
	{ CCI_REG8(0x26), 0x07 },
	{ CCI_REG8(0x29), 0x07 },
	{ CCI_REG8(0x2a), 0x12 },
	{ CCI_REG8(0x2b), 0x07 },
	{ CCI_REG8(0xfe), 0x00 },
};

static const struct cci_reg_sequence gc8034_1632x1224_regs[] = {
	{ CCI_REG8(0xf2), 0x00 },
	{ CCI_REG8(0xf4), 0x80 },
	{ CCI_REG8(0xf5), 0x19 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x63 },
	{ CCI_REG8(0xfa), 0x45 },
	{ CCI_REG8(0xf9), 0x00 },
	{ CCI_REG8(0xf7), 0x9d },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0x00 },
	{ CCI_REG8(0xfc), 0xea },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x03), 0x9a },
	{ CCI_REG8(0x18), 0x07 },
	{ CCI_REG8(0x01), 0x07 },
	{ CCI_REG8(0xfc), 0xee },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x3f), 0x00 },
	{ CCI_REG8(0x08), 0x20 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x3a },
	{ CCI_REG8(0x0d), 0x09 },
	{ CCI_REG8(0x0e), 0xa0 },
	{ CCI_REG8(0x17), 0xc0 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x80), 0x10 },
	{ CCI_REG8(0xad), 0x30 },
	{ CCI_REG8(0x66), 0x2c },
	{ CCI_REG8(0xbc), 0x49 },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x92), 0x04 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x05 },
	{ CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xc8 },
	{ CCI_REG8(0x97), 0x06 },
	{ CCI_REG8(0x98), 0x60 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0xd3), 0x00 },
	{ CCI_REG8(0xd4), 0x00 },
	{ CCI_REG8(0xd7), 0x00 },
	{ CCI_REG8(0xdc), 0x00 },
	{ CCI_REG8(0xdd), 0x00 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x02), 0x03 },
	{ CCI_REG8(0x04), 0x80 },
	{ CCI_REG8(0x11), 0x2b },
	{ CCI_REG8(0x12), 0xf8 },
	{ CCI_REG8(0x13), 0x07 },
	{ CCI_REG8(0x15), 0x10 },
	{ CCI_REG8(0x16), 0x29 },
	{ CCI_REG8(0x17), 0xff },
	{ CCI_REG8(0x19), 0xaa },
	{ CCI_REG8(0x1a), 0x02 },
	{ CCI_REG8(0x21), 0x02 },
	{ CCI_REG8(0x22), 0x03 },
	{ CCI_REG8(0x23), 0x0a },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x25), 0x12 },
	{ CCI_REG8(0x26), 0x04 },
	{ CCI_REG8(0x29), 0x04 },
	{ CCI_REG8(0x2a), 0x02 },
	{ CCI_REG8(0x2b), 0x04 },
	{ CCI_REG8(0xfe), 0x00 },
};

/* Declared from the biggest to the smallest size */
static const struct gc8034_mode gc8034_modes[] = {
	{
		/* 3264x2448, 29.8 fps, 672 Mbit/s per lane */
		.width = GC8034_NATIVE_WIDTH,
		.height = GC8034_NATIVE_HEIGHT,
		.link_freq_index = 0,
		.vts_def = GC8034_VTS_OFFSET + GC8034_VB_MIN,
		.regs = gc8034_3264x2448_regs,
		.num_regs = ARRAY_SIZE(gc8034_3264x2448_regs),
	},
	{
		/* 1632x1224, 2x2 binned, 29.8 fps, 336 Mbit/s per lane */
		.width = 1632,
		.height = 1224,
		.link_freq_index = 1,
		.vts_def = GC8034_VTS_OFFSET + GC8034_VB_MIN,
		.regs = gc8034_1632x1224_regs,
		.num_regs = ARRAY_SIZE(gc8034_1632x1224_regs),
	},
};

/*
 * Analog gain steps in 1/64 and the analog settings that go with each step.
 * What a step does not cover is made up with the digital pre-gain.
 */
static const u16 gc8034_again_level[] = {
	0x0040,	/*  1.000 */
	0x0058,	/*  1.375 */
	0x007d,	/*  1.950 */
	0x00ad,	/*  2.700 */
	0x00f3,	/*  3.800 */
	0x0159,	/*  5.400 */
	0x01ea,	/*  7.660 */
};

#define GC8034_AGAIN_ROW(r20, r33, rdf, re7, re8, re9, rea, reb, rec, red, ree) { \
	{ GC8034_REG_PAGE, GC8034_PAGE_0 },		\
	{ CCI_REG8(0x20), r20 }, { CCI_REG8(0x33), r33 },	\
	{ GC8034_REG_PAGE, GC8034_PAGE_1 },		\
	{ CCI_REG8(0xdf), rdf }, { CCI_REG8(0xe7), re7 },	\
	{ CCI_REG8(0xe8), re8 }, { CCI_REG8(0xe9), re9 },	\
	{ CCI_REG8(0xea), rea }, { CCI_REG8(0xeb), reb },	\
	{ CCI_REG8(0xec), rec }, { CCI_REG8(0xed), red },	\
	{ CCI_REG8(0xee), ree },				\
	{ GC8034_REG_PAGE, GC8034_PAGE_0 },		\
}

static const struct cci_reg_sequence gc8034_again_regs[][14] = {
	GC8034_AGAIN_ROW(0x55, 0x83, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8),
	GC8034_AGAIN_ROW(0x55, 0x83, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8),
	GC8034_AGAIN_ROW(0x4e, 0x84, 0x0c, 0x2e, 0x2d, 0x15, 0x19, 0x47, 0x70, 0x9f, 0xd8),
	GC8034_AGAIN_ROW(0x51, 0x80, 0x07, 0x28, 0x32, 0x22, 0x20, 0x49, 0x70, 0x91, 0xd9),
	GC8034_AGAIN_ROW(0x4d, 0x83, 0x0f, 0x3b, 0x3b, 0x1c, 0x1f, 0x47, 0x6f, 0x9b, 0xd3),
	GC8034_AGAIN_ROW(0x50, 0x83, 0x08, 0x35, 0x46, 0x1e, 0x22, 0x4c, 0x70, 0x9a, 0xd2),
	GC8034_AGAIN_ROW(0x52, 0x80, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0),
};

#undef GC8034_AGAIN_ROW

static inline struct gc8034 *to_gc8034(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc8034, sd);
}

static bool gc8034_mode_supported(struct gc8034 *gc8034,
				  const struct gc8034_mode *mode)
{
	return gc8034->link_freq_bitmap & BIT(mode->link_freq_index);
}

/* Nearest mode the link frequencies of the board allow; there always is one */
static const struct gc8034_mode *
gc8034_find_mode(struct gc8034 *gc8034, u32 width, u32 height)
{
	const struct gc8034_mode *best = NULL;
	u32 best_err = U32_MAX;
	unsigned int i;

	width = min(width, GC8034_NATIVE_WIDTH);
	height = min(height, GC8034_NATIVE_HEIGHT);

	for (i = 0; i < ARRAY_SIZE(gc8034_modes); i++) {
		const struct gc8034_mode *mode = &gc8034_modes[i];
		u32 err = abs_diff(mode->width, width) +
			  abs_diff(mode->height, height);

		if (!gc8034_mode_supported(gc8034, mode))
			continue;

		if (err < best_err) {
			best_err = err;
			best = mode;
		}
	}

	return best;
}

static int gc8034_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc8034 *gc8034 = to_gc8034(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(gc8034_supplies),
				    gc8034->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	fsleep(1 * USEC_PER_MSEC);

	ret = clk_prepare_enable(gc8034->xclk);
	if (ret) {
		dev_err(dev, "failed to enable the clock: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(gc8034_supplies),
				       gc8034->supplies);
		return ret;
	}

	fsleep(5 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc8034->powerdown_gpio, 0);
	fsleep(5 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc8034->reset_gpio, 0);
	fsleep(10 * USEC_PER_MSEC);

	return 0;
}

static int gc8034_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc8034 *gc8034 = to_gc8034(sd);

	gpiod_set_value_cansleep(gc8034->reset_gpio, 1);
	fsleep(5 * USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc8034->powerdown_gpio, 1);
	fsleep(5 * USEC_PER_MSEC);
	clk_disable_unprepare(gc8034->xclk);
	regulator_bulk_disable(ARRAY_SIZE(gc8034_supplies), gc8034->supplies);

	return 0;
}

static int gc8034_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = GC8034_MBUS_CODE;

	return 0;
}

static int gc8034_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	unsigned int index = fse->index;
	unsigned int i;

	if (fse->code != GC8034_MBUS_CODE)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(gc8034_modes); i++) {
		const struct gc8034_mode *mode = &gc8034_modes[i];

		if (!gc8034_mode_supported(gc8034, mode))
			continue;

		if (index--)
			continue;

		fse->min_width = mode->width;
		fse->max_width = mode->width;
		fse->min_height = mode->height;
		fse->max_height = mode->height;

		return 0;
	}

	return -EINVAL;
}

static int gc8034_update_mode_controls(struct gc8034 *gc8034,
				       const struct gc8034_mode *mode)
{
	s64 vblank_min = GC8034_VTS_OFFSET + GC8034_VB_MIN - mode->height;
	s64 vblank_max = GC8034_VTS_OFFSET + GC8034_VB_MAX - mode->height;
	s64 hblank = GC8034_HTS - mode->width;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(gc8034->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(gc8034->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(gc8034->vblank, vblank_min, vblank_max,
				       1, mode->vts_def - mode->height);
	if (ret)
		return ret;

	/* The exposure limits follow from the vblank control handler */
	return __v4l2_ctrl_s_ctrl(gc8034->vblank,
				  mode->vts_def - mode->height);
}

static int gc8034_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	const struct gc8034_mode *mode;
	struct v4l2_rect *crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = gc8034_find_mode(gc8034, fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = GC8034_MBUS_CODE;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc =
		V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->format.colorspace);
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	/* Both modes show the whole pixel array, the small one binned 2x2 */
	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = GC8034_NATIVE_WIDTH;
	crop->height = GC8034_NATIVE_HEIGHT;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	return gc8034_update_mode_controls(gc8034, mode);
}

static int gc8034_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = GC8034_NATIVE_WIDTH;
		sel->r.height = GC8034_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc8034_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.code = GC8034_MBUS_CODE,
			.width = GC8034_NATIVE_WIDTH,
			.height = GC8034_NATIVE_HEIGHT,
		},
	};

	return gc8034_set_format(sd, state, &fmt);
}

static int gc8034_set_exposure(struct gc8034 *gc8034, u32 exposure)
{
	int ret = 0;

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_EXPOSURE_H,
		  (exposure >> 8) & 0x7f, &ret);
	cci_write(gc8034->regmap, GC8034_REG_EXPOSURE_L, exposure & 0xfe, &ret);

	return ret;
}

static int gc8034_set_gain(struct gc8034 *gc8034, u32 gain)
{
	unsigned int i = ARRAY_SIZE(gc8034_again_level) - 1;
	u32 pregain;
	int ret = 0;

	while (i && gain < gc8034_again_level[i])
		i--;

	/* Digital pre-gain, 8.8 fixed point, covers the rest */
	pregain = 256 * gain / gc8034_again_level[i];

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_AGAIN_INDEX, i, &ret);
	cci_write(gc8034->regmap, GC8034_REG_PREGAIN_H, pregain >> 8, &ret);
	cci_write(gc8034->regmap, GC8034_REG_PREGAIN_L, pregain & 0xff, &ret);
	if (ret)
		return ret;

	return cci_multi_reg_write(gc8034->regmap, gc8034_again_regs[i],
				   ARRAY_SIZE(gc8034_again_regs[i]), NULL);
}

static int gc8034_set_vblank(struct gc8034 *gc8034, u32 vts)
{
	u32 vb = vts - GC8034_VTS_OFFSET;
	int ret = 0;

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_VB_H, (vb >> 8) & 0x1f, &ret);
	cci_write(gc8034->regmap, GC8034_REG_VB_L, vb & 0xff, &ret);

	return ret;
}

static int gc8034_set_test_pattern(struct gc8034 *gc8034, u32 pattern)
{
	int ret = 0;

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_TEST_PATTERN,
		  pattern ? GC8034_TEST_PATTERN_EN : 0, &ret);

	return ret;
}

static int gc8034_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc8034 *gc8034 = container_of(ctrl->handler, struct gc8034,
					     ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&gc8034->sd);
	format = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		/* The exposure has to stay below the new frame length */
		s64 exposure_max = format->height + ctrl->val -
				   GC8034_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(gc8034->exposure,
					       GC8034_EXPOSURE_MIN,
					       exposure_max,
					       GC8034_EXPOSURE_STEP,
					       min_t(s64, exposure_max,
						     GC8034_EXPOSURE_DEFAULT));
		if (ret)
			return ret;
	}

	/* The registers are written when streaming starts */
	if (!pm_runtime_get_if_active(gc8034->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc8034_set_exposure(gc8034, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc8034_set_gain(gc8034, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = gc8034_set_vblank(gc8034, format->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = gc8034_set_test_pattern(gc8034, ctrl->val);
		break;
	default:
		break;
	}

	pm_runtime_put(gc8034->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc8034_ctrl_ops = {
	.s_ctrl = gc8034_set_ctrl,
};

static int gc8034_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	const struct v4l2_mbus_framefmt *format;
	const struct gc8034_mode *mode;
	int ret;

	format = v4l2_subdev_state_get_format(state, 0);
	mode = gc8034_find_mode(gc8034, format->width, format->height);

	ret = pm_runtime_resume_and_get(gc8034->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(gc8034->regmap, gc8034_common_regs,
				  ARRAY_SIZE(gc8034_common_regs), NULL);
	if (ret)
		goto err_rpm_put;

	ret = cci_multi_reg_write(gc8034->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_rpm_put;

	ret = __v4l2_ctrl_handler_setup(&gc8034->ctrls);
	if (ret)
		goto err_rpm_put;

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_STREAM, GC8034_STREAM_ON_4LANE,
		  &ret);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	dev_err(gc8034->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(gc8034->dev);
	return ret;
}

static int gc8034_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	int ret = 0;

	cci_write(gc8034->regmap, GC8034_REG_PAGE, GC8034_PAGE_0, &ret);
	cci_write(gc8034->regmap, GC8034_REG_STREAM, GC8034_STREAM_OFF, &ret);
	if (ret)
		dev_err(gc8034->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(gc8034->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops gc8034_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc8034_pad_ops = {
	.enum_mbus_code = gc8034_enum_mbus_code,
	.enum_frame_size = gc8034_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc8034_set_format,
	.get_selection = gc8034_get_selection,
	.enable_streams = gc8034_enable_streams,
	.disable_streams = gc8034_disable_streams,
};

static const struct v4l2_subdev_ops gc8034_subdev_ops = {
	.video = &gc8034_video_ops,
	.pad = &gc8034_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc8034_internal_ops = {
	.init_state = gc8034_init_state,
};

static int gc8034_parse_fwnode(struct gc8034 *gc8034)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device *dev = gc8034->dev;
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC8034_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "only %u data lanes are supported\n",
				    GC8034_DATA_LANES);
		goto done;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc8034_link_freq_menu,
				       ARRAY_SIZE(gc8034_link_freq_menu),
				       &gc8034->link_freq_bitmap);

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc8034_init_controls(struct gc8034 *gc8034)
{
	const struct gc8034_mode *mode =
		gc8034_find_mode(gc8034, GC8034_NATIVE_WIDTH,
				 GC8034_NATIVE_HEIGHT);
	struct v4l2_ctrl_handler *hdl = &gc8034->ctrls;
	struct v4l2_fwnode_device_properties props;
	s64 hblank = GC8034_HTS - mode->width;
	s64 exposure_max = mode->vts_def - GC8034_EXPOSURE_MARGIN;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	gc8034->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &gc8034_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(gc8034_link_freq_menu) - 1,
				       mode->link_freq_index,
				       gc8034_link_freq_menu);
	if (gc8034->link_freq)
		gc8034->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &gc8034_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  GC8034_PIXEL_RATE, GC8034_PIXEL_RATE, 1,
			  GC8034_PIXEL_RATE);

	gc8034->hblank = v4l2_ctrl_new_std(hdl, &gc8034_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (gc8034->hblank)
		gc8034->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc8034->vblank =
		v4l2_ctrl_new_std(hdl, &gc8034_ctrl_ops, V4L2_CID_VBLANK,
				  GC8034_VTS_OFFSET + GC8034_VB_MIN -
				  mode->height,
				  GC8034_VTS_OFFSET + GC8034_VB_MAX -
				  mode->height, 1,
				  mode->vts_def - mode->height);

	gc8034->exposure =
		v4l2_ctrl_new_std(hdl, &gc8034_ctrl_ops, V4L2_CID_EXPOSURE,
				  GC8034_EXPOSURE_MIN, exposure_max,
				  GC8034_EXPOSURE_STEP,
				  GC8034_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &gc8034_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC8034_GAIN_MIN, GC8034_GAIN_MAX, GC8034_GAIN_STEP,
			  GC8034_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &gc8034_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc8034_test_pattern_menu) - 1,
				     0, 0, gc8034_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(gc8034->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &gc8034_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	gc8034->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int gc8034_identify(struct gc8034 *gc8034)
{
	u64 id_h, id_l;
	int ret = 0;

	cci_read(gc8034->regmap, GC8034_REG_CHIP_ID_H, &id_h, &ret);
	cci_read(gc8034->regmap, GC8034_REG_CHIP_ID_L, &id_l, &ret);
	if (ret)
		return dev_err_probe(gc8034->dev, ret,
				     "failed to read the chip id\n");

	if (((id_h << 8) | id_l) != GC8034_CHIP_ID)
		return dev_err_probe(gc8034->dev, -ENXIO,
				     "chip id mismatch: 0x%02llx%02llx\n",
				     id_h, id_l);

	return 0;
}

static int gc8034_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc8034 *gc8034;
	unsigned int i;
	int ret;

	gc8034 = devm_kzalloc(dev, sizeof(*gc8034), GFP_KERNEL);
	if (!gc8034)
		return -ENOMEM;

	gc8034->dev = dev;

	ret = gc8034_parse_fwnode(gc8034);
	if (ret)
		return ret;

	gc8034->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc8034->regmap))
		return dev_err_probe(dev, PTR_ERR(gc8034->regmap),
				     "failed to init CCI\n");

	gc8034->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc8034->xclk))
		return dev_err_probe(dev, PTR_ERR(gc8034->xclk),
				     "failed to get the clock\n");

	if (clk_get_rate(gc8034->xclk) != GC8034_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "clock rate %lu Hz not supported, need %lu Hz\n",
				     clk_get_rate(gc8034->xclk),
				     GC8034_XCLK_FREQ);

	for (i = 0; i < ARRAY_SIZE(gc8034_supplies); i++) {
		gc8034->supplies[i].supply = gc8034_supplies[i].name;
		gc8034->supplies[i].init_load_uA = gc8034_supplies[i].load_ua;
	}

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc8034_supplies),
				      gc8034->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	gc8034->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc8034->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc8034->reset_gpio),
				     "failed to get the reset GPIO\n");

	gc8034->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(gc8034->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(gc8034->powerdown_gpio),
				     "failed to get the powerdown GPIO\n");

	v4l2_i2c_subdev_init(&gc8034->sd, client, &gc8034_subdev_ops);
	gc8034->sd.internal_ops = &gc8034_internal_ops;

	ret = gc8034_power_on(dev);
	if (ret)
		return ret;

	ret = gc8034_identify(gc8034);
	if (ret)
		goto err_power_off;

	ret = gc8034_init_controls(gc8034);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init controls\n");
		goto err_power_off;
	}

	gc8034->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc8034->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc8034->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc8034->sd.entity, 1, &gc8034->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init the media entity\n");
		goto err_ctrls_free;
	}

	gc8034->sd.state_lock = gc8034->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc8034->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init the subdev state\n");
		goto err_entity_cleanup;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&gc8034->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register the subdev\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_idle(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&gc8034->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_entity_cleanup:
	media_entity_cleanup(&gc8034->sd.entity);
err_ctrls_free:
	v4l2_ctrl_handler_free(&gc8034->ctrls);
err_power_off:
	gc8034_power_off(dev);
	return ret;
}

static void gc8034_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc8034->ctrls);

	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc8034_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id gc8034_of_match[] = {
	{ .compatible = "galaxycore,gc8034" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc8034_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(gc8034_pm_ops, gc8034_power_off,
				 gc8034_power_on, NULL);

static struct i2c_driver gc8034_i2c_driver = {
	.driver = {
		.name = "gc8034",
		.of_match_table = gc8034_of_match,
		.pm = pm_ptr(&gc8034_pm_ops),
	},
	.probe = gc8034_probe,
	.remove = gc8034_remove,
};
module_i2c_driver(gc8034_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC8034 image sensor driver");
MODULE_AUTHOR("Yaron Shahrabani <yarons@users.noreply.github.com>");
MODULE_LICENSE("GPL");
