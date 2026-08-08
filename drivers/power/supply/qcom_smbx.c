// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd.
 * Author: Casey Connolly <casey.connolly@linaro.org>
 *
 * This driver is for the switch-mode battery charger and boost
 * hardware found in pmi8998 and related PMICs.
 */

#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* clang-format off */
#define BATTERY_CHARGER_STATUS_1			0x06
#define BVR_INITIAL_RAMP_BIT				BIT(7)
#define CC_SOFT_TERMINATE_BIT				BIT(6)
#define STEP_CHARGING_STATUS_SHIFT			3
#define STEP_CHARGING_STATUS_MASK			GENMASK(5, 3)
#define BATTERY_CHARGER_STATUS_MASK			GENMASK(2, 0)

#define BATTERY_CHARGER_STATUS_2			0x07
#define INPUT_CURRENT_LIMITED_BIT			BIT(7)
#define CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT		BIT(6)
#define CHARGER_ERROR_STATUS_BAT_OV_BIT			BIT(5)
#define SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT		BIT(1)
#define CHARGER_ERROR_STATUS_BAT_TERM_MISSING_BIT	BIT(4)
#define BAT_TEMP_STATUS_MASK				GENMASK(3, 0)
#define BAT_TEMP_STATUS_SOFT_LIMIT_MASK			GENMASK(3, 2)
#define BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT		BIT(3)
#define BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT		BIT(2)
#define BAT_TEMP_STATUS_TOO_HOT_BIT			BIT(1)
#define BAT_TEMP_STATUS_TOO_COLD_BIT			BIT(0)

#define BATTERY_CHARGER_STATUS_4			0x0A
#define CHARGE_CURRENT_POST_JEITA_MASK			GENMASK(7, 0)

#define BATTERY_CHARGER_STATUS_7			0x0D
#define ENABLE_TRICKLE_BIT				BIT(7)
#define ENABLE_PRE_CHARGING_BIT				BIT(6)
#define ENABLE_FAST_CHARGING_BIT			BIT(5)
#define ENABLE_FULLON_MODE_BIT				BIT(4)
#define TOO_COLD_ADC_BIT				BIT(3)
#define TOO_HOT_ADC_BIT					BIT(2)
#define HOT_SL_ADC_BIT					BIT(1)
#define COLD_SL_ADC_BIT					BIT(0)

#define CHARGING_ENABLE_CMD				0x42
#define CHARGING_ENABLE_CMD_BIT				BIT(0)

#define CHGR_CFG2					0x51
#define CHG_EN_SRC_BIT					BIT(7)
#define CHG_EN_POLARITY_BIT				BIT(6)
#define PRETOFAST_TRANSITION_CFG_BIT			BIT(5)
#define BAT_OV_ECC_BIT					BIT(4)
#define I_TERM_BIT					BIT(3)
#define AUTO_RECHG_BIT					BIT(2)
#define EN_ANALOG_DROP_IN_VBATT_BIT			BIT(1)
#define CHARGER_INHIBIT_BIT				BIT(0)

#define PRE_CHARGE_CURRENT_CFG				0x60
#define PRE_CHARGE_CURRENT_SETTING_MASK			GENMASK(5, 0)

#define FAST_CHARGE_CURRENT_CFG				0x61
#define FAST_CHARGE_CURRENT_SETTING_MASK		GENMASK(7, 0)

#define FLOAT_VOLTAGE_CFG				0x70
#define FLOAT_VOLTAGE_SETTING_MASK			GENMASK(7, 0)

#define FG_UPDATE_CFG_2_SEL				0x7D
#define SOC_LT_OTG_THRESH_SEL_BIT			BIT(3)
#define SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT		BIT(2)
#define VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT		BIT(1)
#define IBT_LT_CHG_TERM_THRESH_SEL_BIT			BIT(0)

#define JEITA_EN_CFG					0x90
#define JEITA_EN_HARDLIMIT_BIT				BIT(4)
#define JEITA_EN_HOT_SL_FCV_BIT				BIT(3)
#define JEITA_EN_COLD_SL_FCV_BIT			BIT(2)
#define JEITA_EN_HOT_SL_CCC_BIT				BIT(1)
#define JEITA_EN_COLD_SL_CCC_BIT			BIT(0)

#define JEITA_CCCOMP_CFG_HOT				0x92
#define JEITA_CCCOMP_CFG_COLD				0x93
#define JEITA_CCCOMP_MASK				GENMASK(5, 0)
#define JEITA_CCCOMP_STEP_UA				25000

/*
 * Two four-byte blocks of comparator thresholds, each holding the hot
 * threshold followed by the cold one, big-endian, as raw BAT_THERM ADC codes.
 * A higher code is a colder battery, so the hot threshold is the smaller
 * number of the pair.
 */
#define JEITA_SOFT_THRESHOLDS				0x94
#define JEITA_HARD_THRESHOLDS				0x98
#define JEITA_THRESHOLDS_LEN				4

/* The BAT_ID divider is biased from the ADC's own 1.875 V reference. */
#define BATT_ID_VREF_UV					1875000
#define BATT_ID_DEFAULT_TOLERANCE_PCT			15

#define INT_RT_STS					0x310
#define TYPE_C_CHANGE_RT_STS_BIT			BIT(7)
#define USBIN_ICL_CHANGE_RT_STS_BIT			BIT(6)
#define USBIN_SOURCE_CHANGE_RT_STS_BIT			BIT(5)
#define USBIN_PLUGIN_RT_STS_BIT				BIT(4)
#define USBIN_OV_RT_STS_BIT				BIT(3)
#define USBIN_UV_RT_STS_BIT				BIT(2)
#define USBIN_LT_3P6V_RT_STS_BIT			BIT(1)
#define USBIN_COLLAPSE_RT_STS_BIT			BIT(0)

#define OTG_CFG						0x153
#define OTG_RESERVED_MASK				GENMASK(7, 6)
#define DIS_OTG_ON_TLIM_BIT				BIT(5)
#define QUICKSTART_OTG_FASTROLESWAP_BIT			BIT(4)
#define INCREASE_DFP_TIME_BIT				BIT(3)
#define ENABLE_OTG_IN_DEBUG_MODE_BIT			BIT(2)
#define OTG_EN_SRC_CFG_BIT				BIT(1)
#define CONCURRENT_MODE_CFG_BIT				BIT(0)

#define OTG_ENG_OTG_CFG					0x1C0
#define ENG_BUCKBOOST_HALT1_8_MODE_BIT			BIT(0)

#define APSD_STATUS					0x307
#define APSD_STATUS_7_BIT				BIT(7)
#define HVDCP_CHECK_TIMEOUT_BIT				BIT(6)
#define SLOW_PLUGIN_TIMEOUT_BIT				BIT(5)
#define ENUMERATION_DONE_BIT				BIT(4)
#define VADP_CHANGE_DONE_AFTER_AUTH_BIT			BIT(3)
#define QC_AUTH_DONE_STATUS_BIT				BIT(2)
#define QC_CHARGER_BIT					BIT(1)
#define APSD_DTC_STATUS_DONE_BIT			BIT(0)

#define APSD_RESULT_STATUS				0x308
#define ICL_OVERRIDE_LATCH_BIT				BIT(7)
#define APSD_RESULT_STATUS_MASK				GENMASK(6, 0)
#define QC_3P0_BIT					BIT(6)
#define QC_2P0_BIT					BIT(5)
#define FLOAT_CHARGER_BIT				BIT(4)
#define DCP_CHARGER_BIT					BIT(3)
#define CDP_CHARGER_BIT					BIT(2)
#define OCP_CHARGER_BIT					BIT(1)
#define SDP_CHARGER_BIT					BIT(0)

#define USBIN_CMD_IL					0x340
#define USBIN_SUSPEND_BIT				BIT(0)

#define TYPE_C_STATUS_1					0x30B
#define UFP_TYPEC_MASK					GENMASK(7, 5)
#define UFP_TYPEC_RDSTD_BIT				BIT(7)
#define UFP_TYPEC_RD1P5_BIT				BIT(6)
#define UFP_TYPEC_RD3P0_BIT				BIT(5)
#define UFP_TYPEC_FMB_255K_BIT				BIT(4)
#define UFP_TYPEC_FMB_301K_BIT				BIT(3)
#define UFP_TYPEC_FMB_523K_BIT				BIT(2)
#define UFP_TYPEC_FMB_619K_BIT				BIT(1)
#define UFP_TYPEC_OPEN_OPEN_BIT				BIT(0)

#define TYPE_C_STATUS_2					0x30C
#define DFP_RA_OPEN_BIT					BIT(7)
#define TIMER_STAGE_BIT					BIT(6)
#define EXIT_UFP_MODE_BIT				BIT(5)
#define EXIT_DFP_MODE_BIT				BIT(4)
#define DFP_TYPEC_MASK					GENMASK(3, 0)
#define DFP_RD_OPEN_BIT					BIT(3)
#define DFP_RD_RA_VCONN_BIT				BIT(2)
#define DFP_RD_RD_BIT					BIT(1)
#define DFP_RA_RA_BIT					BIT(0)

#define TYPE_C_STATUS_3					0x30D
#define ENABLE_BANDGAP_BIT				BIT(7)
#define U_USB_GND_NOVBUS_BIT				BIT(6)
#define U_USB_FLOAT_NOVBUS_BIT				BIT(5)
#define U_USB_GND_BIT					BIT(4)
#define U_USB_FMB1_BIT					BIT(3)
#define U_USB_FLOAT1_BIT				BIT(2)
#define U_USB_FMB2_BIT					BIT(1)
#define U_USB_FLOAT2_BIT				BIT(0)

#define TYPE_C_STATUS_4					0x30E
#define UFP_DFP_MODE_STATUS_BIT				BIT(7)
#define TYPEC_VBUS_STATUS_BIT				BIT(6)
#define TYPEC_VBUS_ERROR_STATUS_BIT			BIT(5)
#define TYPEC_DEBOUNCE_DONE_STATUS_BIT			BIT(4)
#define TYPEC_UFP_AUDIO_ADAPT_STATUS_BIT		BIT(3)
#define TYPEC_VCONN_OVERCURR_STATUS_BIT			BIT(2)
#define CC_ORIENTATION_BIT				BIT(1)
#define CC_ATTACHED_BIT					BIT(0)

#define TYPE_C_STATUS_5					0x30F
#define TRY_SOURCE_FAILED_BIT				BIT(6)
#define TRY_SINK_FAILED_BIT				BIT(5)
#define TIMER_STAGE_2_BIT				BIT(4)
#define TYPEC_LEGACY_CABLE_STATUS_BIT			BIT(3)
#define TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT		BIT(2)
#define TYPEC_TRYSOURCE_DETECT_STATUS_BIT		BIT(1)
#define TYPEC_TRYSINK_DETECT_STATUS_BIT			BIT(0)

#define CMD_APSD					0x341
#define ICL_OVERRIDE_BIT				BIT(1)
#define APSD_RERUN_BIT					BIT(0)

#define TYPE_C_CFG					0x358
#define APSD_START_ON_CC_BIT				BIT(7)
#define WAIT_FOR_APSD_BIT				BIT(6)
#define FACTORY_MODE_DETECTION_EN_BIT			BIT(5)
#define FACTORY_MODE_ICL_3A_4A_BIT			BIT(4)
#define FACTORY_MODE_DIS_CHGING_CFG_BIT			BIT(3)
#define SUSPEND_NON_COMPLIANT_CFG_BIT			BIT(2)
#define VCONN_OC_CFG_BIT				BIT(1)
#define TYPE_C_OR_U_USB_BIT				BIT(0)

#define TYPE_C_CFG_2					0x359
#define TYPE_C_DFP_CURRSRC_MODE_BIT			BIT(7)
#define DFP_CC_1P4V_OR_1P6V_BIT				BIT(6)
#define VCONN_SOFTSTART_CFG_MASK			GENMASK(5, 4)
#define EN_TRY_SOURCE_MODE_BIT				BIT(3)
#define USB_FACTORY_MODE_ENABLE_BIT			BIT(2)
#define TYPE_C_UFP_MODE_BIT				BIT(1)
#define EN_80UA_180UA_CUR_SOURCE_BIT			BIT(0)

#define TYPE_C_CFG_3					0x35A
#define TVBUS_DEBOUNCE_BIT				BIT(7)
#define TYPEC_LEGACY_CABLE_INT_EN_BIT			BIT(6)
#define TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN_B	BIT(5)
#define TYPEC_TRYSOURCE_DETECT_INT_EN_BIT		BIT(4)
#define TYPEC_TRYSINK_DETECT_INT_EN_BIT			BIT(3)
#define EN_TRYSINK_MODE_BIT				BIT(2)
#define EN_LEGACY_CABLE_DETECTION_BIT			BIT(1)
#define ALLOW_PD_DRING_UFP_TCCDB_BIT			BIT(0)

#define USBIN_OPTIONS_1_CFG				0x362
#define CABLE_R_SEL_BIT					BIT(7)
#define HVDCP_AUTH_ALG_EN_CFG_BIT			BIT(6)
#define HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT		BIT(5)
#define INPUT_PRIORITY_BIT				BIT(4)
#define AUTO_SRC_DETECT_BIT				BIT(3)
#define HVDCP_EN_BIT					BIT(2)
#define VADP_INCREMENT_VOLTAGE_LIMIT_BIT		BIT(1)
#define VADP_TAPER_TIMER_EN_BIT				BIT(0)

#define USBIN_OPTIONS_2_CFG				0x363
#define WIPWR_RST_EUD_CFG_BIT				BIT(7)
#define SWITCHER_START_CFG_BIT				BIT(6)
#define DCD_TIMEOUT_SEL_BIT				BIT(5)
#define OCD_CURRENT_SEL_BIT				BIT(4)
#define SLOW_PLUGIN_TIMER_EN_CFG_BIT			BIT(3)
#define FLOAT_OPTIONS_MASK				GENMASK(2, 0)
#define FLOAT_DIS_CHGING_CFG_BIT			BIT(2)
#define SUSPEND_FLOAT_CFG_BIT				BIT(1)
#define FORCE_FLOAT_SDP_CFG_BIT				BIT(0)

#define TAPER_TIMER_SEL_CFG				0x364
#define TYPEC_SPARE_CFG_BIT				BIT(7)
#define TYPEC_DRP_DFP_TIME_CFG_BIT			BIT(5)
#define TAPER_TIMER_SEL_MASK				GENMASK(1, 0)

#define USBIN_LOAD_CFG					0x365
#define USBIN_OV_CH_LOAD_OPTION_BIT			BIT(7)
#define ICL_OVERRIDE_AFTER_APSD_BIT			BIT(4)

#define USBIN_ICL_OPTIONS				0x366
#define CFG_USB3P0_SEL_BIT				BIT(2)
#define USB51_MODE_BIT					BIT(1)
#define USBIN_MODE_CHG_BIT				BIT(0)

#define TYPE_C_INTRPT_ENB_SOFTWARE_CTRL			0x368
#define EXIT_SNK_BASED_ON_CC_BIT			BIT(7)
#define VCONN_EN_ORIENTATION_BIT			BIT(6)
#define TYPEC_VCONN_OVERCURR_INT_EN_BIT			BIT(5)
#define VCONN_EN_SRC_BIT				BIT(4)
#define VCONN_EN_VALUE_BIT				BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK			GENMASK(2, 0)
#define UFP_EN_CMD_BIT					BIT(2)
#define DFP_EN_CMD_BIT					BIT(1)
#define TYPEC_DISABLE_CMD_BIT				BIT(0)

#define USBIN_CURRENT_LIMIT_CFG				0x370
#define USBIN_CURRENT_LIMIT_MASK			GENMASK(7, 0)

#define USBIN_AICL_OPTIONS_CFG				0x380
#define SUSPEND_ON_COLLAPSE_USBIN_BIT			BIT(7)
#define USBIN_AICL_HDC_EN_BIT				BIT(6)
#define USBIN_AICL_START_AT_MAX_BIT			BIT(5)
#define USBIN_AICL_RERUN_EN_BIT				BIT(4)
#define USBIN_AICL_ADC_EN_BIT				BIT(3)
#define USBIN_AICL_EN_BIT				BIT(2)
#define USBIN_HV_COLLAPSE_RESPONSE_BIT			BIT(1)
#define USBIN_LV_COLLAPSE_RESPONSE_BIT			BIT(0)

#define USBIN_5V_AICL_THRESHOLD_CFG			0x381
#define USBIN_5V_AICL_THRESHOLD_CFG_MASK		GENMASK(2, 0)

#define USBIN_CONT_AICL_THRESHOLD_CFG			0x384
#define USBIN_CONT_AICL_THRESHOLD_CFG_MASK		GENMASK(5, 0)

#define DC_ENG_SSUPPLY_CFG2				0x4C1
#define ENG_SSUPPLY_IVREF_OTG_SS_MASK			GENMASK(2, 0)
#define OTG_SS_SLOW					0x3

#define DCIN_AICL_REF_SEL_CFG				0x481
#define DCIN_CONT_AICL_THRESHOLD_CFG_MASK		GENMASK(5, 0)

#define WI_PWR_OPTIONS					0x495
#define CHG_OK_BIT					BIT(7)
#define WIPWR_UVLO_IRQ_OPT_BIT				BIT(6)
#define BUCK_HOLDOFF_ENABLE_BIT				BIT(5)
#define CHG_OK_HW_SW_SELECT_BIT				BIT(4)
#define WIPWR_RST_ENABLE_BIT				BIT(3)
#define DCIN_WIPWR_IRQ_SELECT_BIT			BIT(2)
#define AICL_SWITCH_ENABLE_BIT				BIT(1)
#define ZIN_ICL_ENABLE_BIT				BIT(0)

/*
 * ICL_STATUS and POWER_PATH_STATUS live in the MISC peripheral (base offset
 * 0x600) on SMB2 (pmi8998/pm660) but in the DCDC peripheral (0x100) on SMB5
 * (pmi632). The generation-dependent prefix is held in smb_variant::status_base
 * and added to chip->base at the call sites, so these are in-peripheral offsets.
 */
#define ICL_STATUS					0x07
#define INPUT_CURRENT_LIMIT_MASK			GENMASK(7, 0)

#define POWER_PATH_STATUS				0x0B
#define P_PATH_INPUT_SS_DONE_BIT			BIT(7)
#define P_PATH_USBIN_SUSPEND_STS_BIT			BIT(6)
#define P_PATH_DCIN_SUSPEND_STS_BIT			BIT(5)
#define P_PATH_USE_USBIN_BIT				BIT(4)
#define P_PATH_USE_DCIN_BIT				BIT(3)
#define P_PATH_POWER_PATH_MASK				GENMASK(2, 1)
#define P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT		BIT(0)

#define BARK_BITE_WDOG_PET				0x643
#define BARK_BITE_WDOG_PET_BIT				BIT(0)

#define WD_CFG						0x651
#define WATCHDOG_TRIGGER_AFP_EN_BIT			BIT(7)
#define BARK_WDOG_INT_EN_BIT				BIT(6)
#define BITE_WDOG_INT_EN_BIT				BIT(5)
#define SFT_AFTER_WDOG_IRQ_MASK				GENMASK(4, 3)
#define WDOG_IRQ_SFT_BIT				BIT(2)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT			BIT(1)
#define WDOG_TIMER_EN_BIT				BIT(0)

#define SNARL_BARK_BITE_WD_CFG				0x653
#define BITE_WDOG_DISABLE_CHARGING_CFG_BIT		BIT(7)
#define SNARL_WDOG_TIMEOUT_MASK				GENMASK(6, 4)
#define BARK_WDOG_TIMEOUT_MASK				GENMASK(3, 2)
#define BITE_WDOG_TIMEOUT_MASK				GENMASK(1, 0)

#define AICL_RERUN_TIME_CFG				0x661
#define AICL_RERUN_TIME_MASK				GENMASK(1, 0)

#define STAT_CFG					0x690
#define STAT_SW_OVERRIDE_VALUE_BIT			BIT(7)
#define STAT_SW_OVERRIDE_CFG_BIT			BIT(6)
#define STAT_PARALLEL_OFF_DG_CFG_MASK			GENMASK(5, 4)
#define STAT_POLARITY_CFG_BIT				BIT(3)
#define STAT_PARALLEL_CFG_BIT				BIT(2)
#define STAT_FUNCTION_CFG_BIT				BIT(1)
#define STAT_IRQ_PULSING_EN_BIT				BIT(0)

/*
 * Fuel-gauge (QG) peripheral, present on SMB5 PMICs. Its base is absolute
 * inside the PMIC rather than relative to the charger's own base, and is
 * carried in smb_variant::qg_base. The offsets and the LSB sizes below are
 * the downstream qpnp-qg driver's (qg-reg.h, qg-defs.h).
 *
 * This driver only ever reads the peripheral: the PMIC's own boot sequence
 * starts the gauge, so the sample registers are live without anything here
 * configuring them. Verified on a Fairphone 3 running this driver, where the
 * ADC registers track a load step within one sample period.
 */
#define QG_S7_PON_OCV_V_DATA0				0x70
#define QG_S7_PON_OCV_I_DATA0				0x72
#define QG_S3_GOOD_OCV_V_DATA0				0x74
#define QG_S3_GOOD_OCV_I_DATA0				0x76
#define QG_LAST_ADC_V_DATA0				0xc0
#define QG_LAST_ADC_I_DATA0				0xc2

/* Both sample registers read back as 0x8000 while they hold no measurement */
#define QG_ADC_INVALID					0x8000
/* nV per LSB of the voltage ADC, nA per LSB of the current ADC */
#define QG_V_LSB_NV					194637
#define QG_I_LSB_NA					152588

/*
 * How often the gauge integrates, and the currents at which an open-circuit
 * voltage reading is trusted enough to correct the integral. The wider band
 * still carries an IR error of a few millivolts, which is why it only nudges;
 * the narrow one is small enough to steer with.
 */
#define SMB_FG_POLL_MS					10000
#define SMB_FG_OCV_QUIET_UA				50000
#define SMB_FG_OCV_SETTLED_UA				150000
#define SMB_FG_OCV_WEIGHT_QUIET				25
#define SMB_FG_OCV_WEIGHT_SETTLED			3
/*
 * A poll this late means the CPU was suspended rather than merely busy, so
 * the last current sample says nothing about the interval. Re-anchor on the
 * open-circuit voltage instead, which a rested battery reports accurately.
 */
#define SMB_FG_STALE_MS					60000

#define SDP_CURRENT_UA					500000
#define CDP_CURRENT_UA					1500000
#define DCP_CURRENT_UA					1500000
#define CURRENT_MAX_UA					DCP_CURRENT_UA

/* pmi8998 registers represent current in increments of 1/40th of an amp */
#define CURRENT_SCALE_FACTOR				25000
/* clang-format on */

/*
 * What the eight BATTERY_CHARGER_STATUS_1 codes mean, per generation. SMB5
 * renumbered them: INHIBIT moved from 6 down to 0, the three codes below it
 * shifted up by one, and PAUSE - which SMB2 does not have - took the 6 that
 * INHIBIT left behind. Codes 3, 4, 5 and 7 mean the same thing on both, which
 * is why reading an SMB5 PMIC through the SMB2 table looks right in the middle
 * of a charge and is wrong at either end of it.
 *
 * Inhibit is reported as full because that is what inhibits: the cell is above
 * the recharge threshold. Pause is not - it is a charge that stopped for a
 * reason of its own.
 */
/*
 * Charge completion, the one code both generations put at the same value and
 * the only one that says the charger stopped because the battery is full.
 */
#define CHARGE_STATUS_TERMINATE				5

static const u8 smb2_charge_status[8] = {
	POWER_SUPPLY_STATUS_CHARGING,		/* 0 trickle */
	POWER_SUPPLY_STATUS_CHARGING,		/* 1 pre */
	POWER_SUPPLY_STATUS_CHARGING,		/* 2 fast */
	POWER_SUPPLY_STATUS_CHARGING,		/* 3 full-on */
	POWER_SUPPLY_STATUS_CHARGING,		/* 4 taper */
	POWER_SUPPLY_STATUS_FULL,		/* 5 terminate */
	POWER_SUPPLY_STATUS_FULL,		/* 6 inhibit */
	POWER_SUPPLY_STATUS_NOT_CHARGING,	/* 7 disable */
};

static const u8 smb5_charge_status[8] = {
	/*
	 * Inhibit is where SMB2 reports full, but on SMB5 it is also where a
	 * charger sits whenever it is being held off - including with a cell
	 * that was already above the recharge threshold when the cable went
	 * in and so was never charged at all. Not charging is the most the
	 * code alone supports saying; smb_get_prop_status() upgrades it to
	 * full where the gauge watched the charge that led into it, and
	 * charge completion has its own code below.
	 */
	POWER_SUPPLY_STATUS_NOT_CHARGING,	/* 0 inhibit */
	POWER_SUPPLY_STATUS_CHARGING,		/* 1 trickle */
	POWER_SUPPLY_STATUS_CHARGING,		/* 2 pre */
	POWER_SUPPLY_STATUS_CHARGING,		/* 3 full-on */
	POWER_SUPPLY_STATUS_CHARGING,		/* 4 taper */
	POWER_SUPPLY_STATUS_FULL,		/* 5 terminate */
	POWER_SUPPLY_STATUS_NOT_CHARGING,	/* 6 pause */
	POWER_SUPPLY_STATUS_NOT_CHARGING,	/* 7 disable */
};

struct smb_init_register {
	u16 addr;
	u8 mask;
	u8 val;
};

/**
 * struct smb_variant - per-PMIC-generation parameters
 * @name:		Model name, used for the power_supply name
 * @status_base:	Peripheral prefix for ICL_STATUS / POWER_PATH_STATUS
 *			(0x600 = MISC on SMB2, 0x100 = DCDC on SMB5)
 * @current_scale_ua:	uA per LSB of the FCC / ICL registers
 *			(25000 on SMB2/pmi8998, 50000 on SMB5/pmi632)
 * @fcc_max_ua:		Highest fast-charge current this PMIC generation can
 *			deliver, from the datasheet values Qualcomm's own
 *			drivers carry (qpnp-smb2 / qpnp-smb5 smb_params.fcc.max_u).
 *			A hardware bound, not a policy one - how much of it a
 *			board may use is the board's business, not this file's
 * @float_base_uv:	Float-voltage register value 0 corresponds to this voltage
 * @float_step_uv:	uV per LSB of FLOAT_VOLTAGE_CFG
 * @ov_bit:		BAT_OV (overvoltage) bit within BATTERY_CHARGER_STATUS_2
 *			(BIT(5) on SMB2, BIT(1) on SMB5)
 * @charge_status:	What each of the eight BATTERY_CHARGER_STATUS_1 codes
 *			means on this generation, see smb2_charge_status
 * @inhibit_code:	Which of those codes is charge inhibit - the state a
 *			finished charge settles into once the cell is above the
 *			recharge threshold (6 on SMB2, 0 on SMB5)
 * @temp_status_reg:	Register holding the JEITA temperature-status bits,
 *			relative to @base (BATTERY_CHARGER_STATUS_2 = 0x07 on
 *			SMB2, BATTERY_CHARGER_STATUS_7 = 0x0D on SMB5)
 * @temp_status_shift:	Left-shift of the HOT_SOFT/COLD_SOFT/TOO_HOT/TOO_COLD
 *			bits versus the SMB2 baseline (0 on SMB2, 2 on SMB5)
 * @init_seq:		HW init register write sequence for this generation
 * @init_seq_len:	Number of entries in @init_seq
 * @qg_base:		Absolute base of the QG fuel-gauge peripheral, or 0 on
 *			a PMIC where this driver has no gauge to read
 *
 * All values are taken from the Qualcomm downstream qpnp-smb2 / qpnp-smb5
 * drivers (smb_chg_param tables) so the current/voltage scaling is exact.
 */
struct smb_variant {
	const char *name;
	u16 status_base;
	u32 current_scale_ua;
	u32 fcc_max_ua;
	u32 float_base_uv;
	u32 float_step_uv;
	u8 ov_bit;
	const u8 *charge_status;
	u8 inhibit_code;
	u16 temp_status_reg;
	u8 temp_status_shift;
	const struct smb_init_register *init_seq;
	int init_seq_len;
	u16 qg_base;
};

/**
 * struct smb_chip - smb chip structure
 * @dev:		Device reference for power_supply
 * @name:		The platform device name
 * @base:		Base address for smb registers
 * @regmap:		Register map
 * @batt_info:		Battery data from DT
 * @status_change_work: Worker to handle plug/unplug events
 * @cable_irq:		USB plugin IRQ
 * @wakeup_enabled:	If the cable IRQ will cause a wakeup
 * @usb_in_i_chan:	USB_IN current measurement channel
 * @usb_in_v_chan:	USB_IN voltage measurement channel
 * @vbat_chan:		Battery voltage (VBAT_SNS) measurement channel
 * @bat_therm_chan:	Battery thermistor (BAT_THERM) measurement channel
 * @bat_id_chan:	Battery-ID resistor measurement channel, if the board
 *			routes one
 * @chg_psy:		Charger power supply instance
 * @batt_psy:		Battery (fuel-gauge) power supply instance
 * @thermal_mitigation_ua: Fast-charge current for each thermal cooling state,
 *			from qcom,thermal-mitigation, most permissive first
 * @thermal_levels:	Number of entries in @thermal_mitigation_ua
 * @thermal_level:	Cooling state currently in effect
 * @fg_work:		Worker that integrates the fuel gauge, see smb_fg_work()
 * @fg_lock:		Serialises @soc_permyriad and @fg_last against readers
 * @soc_permyriad:	State of charge in hundredths of a percent
 * @fg_residue:		Charge counted but too small to have moved
 *			@soc_permyriad yet, in microamp-milliseconds
 * @fg_last:		When the gauge last integrated
 * @fg_ready:		Set once @soc_permyriad holds a real estimate
 * @fg_full:		The charger has finished a charge on the input that is
 *			still attached, so the pack is full whatever the curve
 *			says, see smb_fg_track_completion()
 * @fg_charge_seen:	A charge has actually been in progress since that input
 *			appeared, which is what makes a later inhibit mean
 *			completion rather than a top-up that was never needed
 */
struct smb_chip {
	struct device *dev;
	const char *name;
	unsigned int base;
	const struct smb_variant *var;
	struct regmap *regmap;
	struct power_supply_battery_info *batt_info;

	struct delayed_work status_change_work;
	int cable_irq;
	bool wakeup_enabled;

	struct iio_channel *usb_in_i_chan;
	struct iio_channel *usb_in_v_chan;
	struct iio_channel *vbat_chan;
	struct iio_channel *bat_therm_chan;
	struct iio_channel *bat_id_chan;

	struct power_supply *chg_psy;
	struct power_supply *batt_psy;

	u32 *thermal_mitigation_ua;
	unsigned int thermal_levels;
	unsigned long thermal_level;

	struct delayed_work fg_work;
	/* Serialises the gauge state below against readers of it */
	struct mutex fg_lock;
	int soc_permyriad;
	s64 fg_residue;
	ktime_t fg_last;
	bool fg_ready;
	bool fg_full;
	bool fg_charge_seen;
};

static enum power_supply_property smb_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static int smb_get_prop_usb_online(struct smb_chip *chip, int *val)
{
	unsigned int stat;
	int rc;

	rc = regmap_read(chip->regmap,
			 chip->base + chip->var->status_base + POWER_PATH_STATUS,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read power path status: %d\n", rc);
		return rc;
	}

	*val = (stat & P_PATH_USE_USBIN_BIT) &&
	       (stat & P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT);
	return 0;
}

/*
 * Qualcomm "automatic power source detection" aka APSD
 * tells us what type of charger we're connected to.
 */
static int smb_apsd_get_charger_type(struct smb_chip *chip, int *val)
{
	unsigned int apsd_stat, stat;
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return rc;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_STATUS, &apsd_stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read apsd status, rc = %d", rc);
		return rc;
	}
	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT)) {
		dev_dbg(chip->dev, "Apsd not ready");
		return -EAGAIN;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_RESULT_STATUS, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read apsd result, rc = %d", rc);
		return rc;
	}

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat & CDP_CHARGER_BIT)
		*val = POWER_SUPPLY_USB_TYPE_CDP;
	else if (stat & (DCP_CHARGER_BIT | OCP_CHARGER_BIT | FLOAT_CHARGER_BIT))
		*val = POWER_SUPPLY_USB_TYPE_DCP;
	else /* SDP_CHARGER_BIT (or others) */
		*val = POWER_SUPPLY_USB_TYPE_SDP;

	return 0;
}

static int smb_get_prop_status(struct smb_chip *chip, int *val)
{
	unsigned char stat[2];
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = regmap_bulk_read(chip->regmap,
			      chip->base + BATTERY_CHARGER_STATUS_1, &stat, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read charging status ret=%d\n",
			rc);
		return rc;
	}

	/*
	 * Which bit of STATUS_2 carries BAT_OV also moved between the
	 * generations, so it has to come from the variant here as it does
	 * everywhere else this register is read.
	 */
	if (stat[1] & chip->var->ov_bit) {
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	*val = chip->var->charge_status[stat[0] & BATTERY_CHARGER_STATUS_MASK];

	/*
	 * Inhibit is where a finished charge comes to rest, and by itself it
	 * cannot be told apart from a charge that never needed to start - so
	 * the table has to call it not-charging. Where the gauge has watched
	 * the charge that led into it, say so.
	 */
	if ((stat[0] & BATTERY_CHARGER_STATUS_MASK) == chip->var->inhibit_code) {
		guard(mutex)(&chip->fg_lock);

		if (chip->fg_full)
			*val = POWER_SUPPLY_STATUS_FULL;
	}

	return 0;
}

static inline int smb_get_current_limit(struct smb_chip *chip,
					 unsigned int *val)
{
	int rc = regmap_read(chip->regmap,
			     chip->base + chip->var->status_base + ICL_STATUS,
			     val);

	if (rc >= 0)
		*val *= chip->var->current_scale_ua;
	return rc;
}

static int smb_set_current_limit(struct smb_chip *chip, unsigned int val)
{
	unsigned char val_raw;

	if (val > 4800000) {
		dev_err(chip->dev,
			"Can't set current limit higher than 4800000uA");
		return -EINVAL;
	}
	val_raw = val / chip->var->current_scale_ua;

	return regmap_write(chip->regmap, chip->base + USBIN_CURRENT_LIMIT_CFG,
			    val_raw);
}

static int smb_get_fast_charge_current(struct smb_chip *chip, unsigned int *val)
{
	unsigned int val_raw;
	int rc;

	rc = regmap_read(chip->regmap, chip->base + FAST_CHARGE_CURRENT_CFG,
			 &val_raw);
	if (rc < 0)
		return rc;

	*val = (val_raw & FAST_CHARGE_CURRENT_SETTING_MASK) *
	       chip->var->current_scale_ua;

	return 0;
}

static int smb_set_fast_charge_current(struct smb_chip *chip, unsigned int val)
{
	unsigned int val_raw = val / chip->var->current_scale_ua;

	if (val_raw > FAST_CHARGE_CURRENT_SETTING_MASK)
		return -EINVAL;

	return regmap_update_bits(chip->regmap,
				  chip->base + FAST_CHARGE_CURRENT_CFG,
				  FAST_CHARGE_CURRENT_SETTING_MASK, val_raw);
}

static void smb_status_change_work(struct work_struct *work)
{
	unsigned int charger_type, current_ua;
	int usb_online = 0;
	int count, rc;
	struct smb_chip *chip;

	chip = container_of(work, struct smb_chip, status_change_work.work);

	smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online)
		return;

	for (count = 0; count < 3; count++) {
		dev_dbg(chip->dev, "get charger type retry %d\n", count);
		rc = smb_apsd_get_charger_type(chip, &charger_type);
		if (rc != -EAGAIN)
			break;
		msleep(100);
	}

	if (rc < 0 && rc != -EAGAIN) {
		dev_err(chip->dev, "get charger type failed: %d\n", rc);
		return;
	}

	if (rc < 0) {
		rc = regmap_update_bits(chip->regmap, chip->base + CMD_APSD,
					APSD_RERUN_BIT, APSD_RERUN_BIT);
		schedule_delayed_work(&chip->status_change_work,
				      msecs_to_jiffies(1000));
		dev_dbg(chip->dev, "get charger type failed, rerun apsd\n");
		return;
	}

	switch (charger_type) {
	case POWER_SUPPLY_USB_TYPE_CDP:
		current_ua = CDP_CURRENT_UA;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		current_ua = DCP_CURRENT_UA;
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
	default:
		current_ua = SDP_CURRENT_UA;
		break;
	}

	smb_set_current_limit(chip, current_ua);
	power_supply_changed(chip->chg_psy);
	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);
}

static int smb_get_iio_chan(struct smb_chip *chip, struct iio_channel *chan,
			     int *val)
{
	int rc;
	union power_supply_propval status;

	rc = power_supply_get_property(chip->chg_psy, POWER_SUPPLY_PROP_STATUS,
				       &status);
	if (rc < 0 || status.intval != POWER_SUPPLY_STATUS_CHARGING) {
		*val = 0;
		return 0;
	}

	if (IS_ERR(chan)) {
		dev_err(chip->dev, "Failed to chan, err = %li", PTR_ERR(chan));
		return PTR_ERR(chan);
	}

	return iio_read_channel_processed(chan, val);
}

static int smb_get_prop_health(struct smb_chip *chip, int *val)
{
	int rc;
	unsigned int stat;

	/*
	 * Battery overvoltage is reported in BATTERY_CHARGER_STATUS_2 on both
	 * generations, but at a different bit (smb_variant::ov_bit).
	 */
	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_2,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read charger status rc=%d\n", rc);
		return rc;
	}

	if (stat & chip->var->ov_bit) {
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	/*
	 * JEITA temperature status. On SMB2 it shares BATTERY_CHARGER_STATUS_2;
	 * on SMB5 it lives in BATTERY_CHARGER_STATUS_7 with the bits shifted up
	 * by two. The per-variant register and shift select the right layout.
	 * Test bits individually (a register may have several set) and let the
	 * hard limits take precedence over the soft ones.
	 */
	rc = regmap_read(chip->regmap, chip->base + chip->var->temp_status_reg,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read temp status rc=%d\n", rc);
		return rc;
	}

	if (stat & (BAT_TEMP_STATUS_TOO_HOT_BIT << chip->var->temp_status_shift))
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & (BAT_TEMP_STATUS_TOO_COLD_BIT << chip->var->temp_status_shift))
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & (BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT << chip->var->temp_status_shift))
		*val = POWER_SUPPLY_HEALTH_WARM;
	else if (stat & (BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT << chip->var->temp_status_shift))
		*val = POWER_SUPPLY_HEALTH_COOL;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb_get_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct smb_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Qualcomm";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->name;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return smb_get_current_limit(chip, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return smb_get_iio_chan(chip, chip->usb_in_i_chan,
					 &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return smb_get_iio_chan(chip, chip->usb_in_v_chan,
					 &val->intval);
	case POWER_SUPPLY_PROP_ONLINE:
		return smb_get_prop_usb_online(chip, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return smb_get_prop_status(chip, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return smb_get_prop_health(chip, &val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		return smb_apsd_get_charger_type(chip, &val->intval);
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
}

static int smb_set_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
	struct smb_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return regmap_update_bits(chip->regmap, chip->base + USBIN_CMD_IL,
					  USBIN_SUSPEND_BIT, !val->intval);
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return smb_set_current_limit(chip, val->intval);
	default:
		dev_err(chip->dev, "No setter for property: %d\n", psp);
		return -EINVAL;
	}
}

static int smb_property_is_writable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		return 0;
	}
}

static irqreturn_t smb_handle_batt_overvoltage(int irq, void *data)
{
	struct smb_chip *chip = data;
	unsigned int status;

	regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_2,
		    &status);

	if (status & chip->var->ov_bit) {
		/* The hardware stops charging automatically */
		dev_err(chip->dev, "battery overvoltage detected\n");
		power_supply_changed(chip->chg_psy);
	}

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_plugin(int irq, void *data)
{
	struct smb_chip *chip = data;

	power_supply_changed(chip->chg_psy);
	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);

	schedule_delayed_work(&chip->status_change_work,
			      msecs_to_jiffies(1500));

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_icl_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	power_supply_changed(chip->chg_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_wdog_bark(int irq, void *data)
{
	struct smb_chip *chip = data;
	int rc;

	power_supply_changed(chip->chg_psy);

	rc = regmap_write(chip->regmap, BARK_BITE_WDOG_PET,
			  BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

/*
 * Battery (fuel-gauge) power supply.
 *
 * The state of charge is carried in the OCV->capacity table of the
 * monitored-battery node, which is the downstream QG battery profile. That
 * table maps an *open-circuit* voltage, so what is fed into it decides how
 * good the answer is: the battery's terminal voltage moves by hundreds of
 * millivolts with load, while the same table spans eighteen points of charge
 * over the forty millivolts between 3.80 V and 3.84 V. Reading the terminal
 * voltage into it therefore does not give a poor estimate, it gives a meter
 * that tracks the CPU rather than the battery.
 *
 * So the terminal voltage is not what this gauge reports. On a PMIC with a QG
 * peripheral we have both the battery voltage and the battery current from
 * the same converter, and the two together support the usual arrangement:
 * integrate the current continuously, and correct that integral against the
 * OCV table only while the current is small enough for the IR term to be
 * worth trusting. The result no longer moves under load, because a load does
 * not change how much charge is in the cell.
 *
 * Reading is non-invasive: ADC channels and QG sample registers only.
 */
static enum power_supply_property smb_batt_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

/**
 * smb_qg_read_sample() - read one QG voltage/current sample register pair
 * @chip: charger
 * @v_off: in-peripheral offset of the voltage half of the pair
 * @v_uv: where to put the voltage, in microvolts
 * @i_ua: where to put the current, in microamps, positive while charging
 *
 * The pairs are always voltage then current two registers apart. The gauge
 * reports current negative into the battery; this returns it the way the
 * power-supply class states it, positive while charging.
 *
 * Return: 0, or -ENODATA when the pair holds no measurement.
 */
static int smb_qg_read_sample(struct smb_chip *chip, unsigned int v_off,
			      int *v_uv, int *i_ua)
{
	u8 buf[4];
	u16 v_raw;
	s16 i_raw;
	int rc;

	rc = regmap_bulk_read(chip->regmap, chip->var->qg_base + v_off, buf,
			      sizeof(buf));
	if (rc < 0)
		return rc;

	v_raw = get_unaligned_le16(&buf[0]);
	i_raw = get_unaligned_le16(&buf[2]);

	if (v_raw == QG_ADC_INVALID)
		return -ENODATA;

	/*
	 * Both products run past two billion at ordinary readings - a 3.9 V
	 * battery is already 4.0e9 nV - so neither may be evaluated in an int.
	 */
	*v_uv = (int)div_u64((u64)v_raw * QG_V_LSB_NV + 500, 1000);
	*i_ua = -(int)div_s64((s64)i_raw * QG_I_LSB_NA, 1000);

	return 0;
}

/*
 * The battery's internal resistance, so an open-circuit voltage can be
 * recovered from a loaded one. Both DT properties are optional; without them
 * the gauge simply confines itself to samples taken at a current low enough
 * for the correction not to matter.
 */
static int smb_batt_resistance_uohm(struct smb_chip *chip, int i_ua)
{
	int r = chip->batt_info->factory_internal_resistance_uohm;

	if (i_ua > 0 && chip->batt_info->factory_internal_resistance_charging_uohm > 0)
		r = chip->batt_info->factory_internal_resistance_charging_uohm;

	return r > 0 ? r : 0;
}

/* Open-circuit voltage behind a terminal voltage loaded with @i_ua */
static int smb_batt_ocv(struct smb_chip *chip, int v_uv, int i_ua)
{
	s64 ir_uv = (s64)i_ua * smb_batt_resistance_uohm(chip, i_ua);

	return v_uv - (int)div_s64(ir_uv, 1000000);
}

static int smb_get_vbat(struct smb_chip *chip, int *val)
{
	if (IS_ERR_OR_NULL(chip->vbat_chan))
		return -ENODATA;

	/* ADC5 processed voltage channels return microvolts */
	return iio_read_channel_processed(chip->vbat_chan, val);
}

static int smb_get_batt_temp(struct smb_chip *chip, int *val)
{
	int temp, rc;

	if (IS_ERR_OR_NULL(chip->bat_therm_chan))
		return -ENODATA;

	/* The ADC returns millidegrees C, the power supply class decidegrees */
	rc = iio_read_channel_processed(chip->bat_therm_chan, &temp);
	if (rc < 0)
		return rc;

	*val = temp / 100;
	return 0;
}

/*
 * Whether this board can be gauged by integration: a PMIC with the peripheral
 * to sample the current, and a pack size to integrate the current against. The
 * second is what the charge-to-percent division divides by, so it has to be a
 * real one.
 */
static bool smb_fg_available(struct smb_chip *chip)
{
	return chip->var->qg_base && chip->batt_info &&
	       chip->batt_info->charge_full_design_uah > 0;
}

/* Capacity, in hundredths of a percent, of an open-circuit voltage */
static int smb_ocv_to_permyriad(struct smb_chip *chip, int ocv_uv)
{
	/* The single table we carry characterises the cell at 25 degC */
	int cap = power_supply_batinfo_ocv2cap(chip->batt_info, ocv_uv, 25);

	if (cap < 0)
		return cap;

	return clamp(cap, 0, 100) * 100;
}

/**
 * smb_fg_track_completion() - follow the charger through the end of a charge
 * @chip: the charger
 *
 * Termination is the one point on the curve the charger knows better than any
 * gauge: it stopped because the cell reached the float voltage at below the
 * termination current, which is what full means. But it is also a state the
 * charger passes through rather than sits in - having terminated, the cell is
 * by definition above the recharge threshold, so the hardware moves on to
 * inhibit and stays there. A gauge that only recognises the instant of
 * termination therefore has to sample it inside that window, and then watches
 * the OCV correction walk its answer back down over the next few polls.
 *
 * So remember it instead. Inhibit is not evidence of a full pack on its own -
 * it is equally what a charger does when it is handed a cell that was already
 * above the threshold when the cable went in - but inhibit *after* a charge was
 * running is the tail of that charge. That pairing is what this tracks, and it
 * needs no knowledge of where the inhibit threshold happens to be set.
 *
 * Returns: true while the attached input has finished charging the pack.
 */
static bool smb_fg_track_completion(struct smb_chip *chip)
{
	unsigned int status, code;
	int online = 0;

	guard(mutex)(&chip->fg_lock);

	smb_get_prop_usb_online(chip, &online);
	if (!online) {
		chip->fg_full = false;
		chip->fg_charge_seen = false;
		return false;
	}

	if (regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_1,
			&status))
		return chip->fg_full;

	code = status & BATTERY_CHARGER_STATUS_MASK;
	if (code == CHARGE_STATUS_TERMINATE) {
		chip->fg_full = true;
	} else if (chip->var->charge_status[code] ==
		   POWER_SUPPLY_STATUS_CHARGING) {
		chip->fg_charge_seen = true;
		chip->fg_full = false;
	} else if (code == chip->var->inhibit_code && chip->fg_charge_seen) {
		chip->fg_full = true;
	}

	/*
	 * The remaining codes - pause, disable - say nothing either way about
	 * how much charge is in the pack, so they leave the answer alone.
	 */
	return chip->fg_full;
}

/**
 * smb_fg_work() - integrate the fuel gauge over one poll interval
 * @work: the delayed work in struct smb_chip
 *
 * Charge counting is what keeps the reported capacity still while the load
 * moves, but on its own it only ever accumulates error, so it is steered by
 * the OCV table whenever the current is low enough for the table to be worth
 * more than the integral. The two currents that gate that are deliberately
 * far below what this phone draws when it is awake and doing something: the
 * intent is to correct while it idles or charges gently, and to coast on the
 * integral the rest of the time.
 */
static bool smb_fg_update(struct smb_chip *chip)
{
	int v_uv, i_ua, ocv_uv, soc_ocv, was, weight;
	ktime_t now = ktime_get_boottime();
	s64 elapsed_ms;

	if (smb_qg_read_sample(chip, QG_LAST_ADC_V_DATA0, &v_uv, &i_ua) < 0)
		return false;

	/*
	 * Hold the pack at full for as long as the charger says it finished
	 * charging it. This is not a shortcut past the curve, it is the one
	 * place the curve cannot answer: the table's top entry is an OCV of
	 * over 4.37 V, which a cell charged to a 4.39 V float only shows while
	 * it is still being held there. Left to relax it settles some seventy
	 * millivolts lower, six percent down the table, and a full battery
	 * reads as not quite full for the rest of the time it stays on the
	 * cable.
	 */
	if (smb_fg_track_completion(chip)) {
		guard(mutex)(&chip->fg_lock);

		was = chip->fg_ready ? chip->soc_permyriad : -1;
		chip->soc_permyriad = 100 * 100;
		chip->fg_residue = 0;
		chip->fg_ready = true;
		chip->fg_last = now;

		return was != chip->soc_permyriad;
	}

	ocv_uv = smb_batt_ocv(chip, v_uv, i_ua);
	soc_ocv = smb_ocv_to_permyriad(chip, ocv_uv);
	if (soc_ocv < 0)
		return false;

	if (abs(i_ua) <= SMB_FG_OCV_QUIET_UA)
		weight = SMB_FG_OCV_WEIGHT_QUIET;
	else if (abs(i_ua) <= SMB_FG_OCV_SETTLED_UA)
		weight = SMB_FG_OCV_WEIGHT_SETTLED;
	else
		weight = 0;

	guard(mutex)(&chip->fg_lock);

	elapsed_ms = ktime_ms_delta(now, chip->fg_last);
	was = chip->fg_ready ? chip->soc_permyriad : -1;

	if (!chip->fg_ready || elapsed_ms > SMB_FG_STALE_MS) {
		/*
		 * A gap this long means the system was suspended, so the
		 * sample says nothing about what happened during it. A
		 * suspended phone is also a rested battery, which is the one
		 * condition where the OCV table needs no help at all - take it
		 * outright rather than integrating a current nobody drew.
		 */
		chip->soc_permyriad = soc_ocv;
		chip->fg_residue = 0;
		chip->fg_ready = true;
	} else {
		/*
		 * charge (uAh) = i_ua * elapsed / 3600, and one percent of the
		 * pack is charge_full_design_uah / 100, so a hundredth of a
		 * percent per millisecond is i_ua * elapsed_ms / (360 * full).
		 *
		 * At a tenth of an amp that quotient is under one per poll, so
		 * carry the undivided remainder rather than rounding it away:
		 * discarded it would be a standing error in the rate itself,
		 * always in the direction of counting too little.
		 */
		s64 per_permyriad = 360LL * chip->batt_info->charge_full_design_uah;
		s64 step;

		chip->fg_residue += (s64)i_ua * elapsed_ms;
		step = div_s64(chip->fg_residue, per_permyriad);
		chip->fg_residue -= step * per_permyriad;

		chip->soc_permyriad += step;
		chip->soc_permyriad += (soc_ocv - chip->soc_permyriad) * weight / 100;
		chip->soc_permyriad = clamp(chip->soc_permyriad, 0, 100 * 100);
	}

	chip->fg_last = now;

	dev_dbg(chip->dev, "fg: %d.%02d%% vbat=%duV ibat=%duA ocv=%duV w=%d\n",
		chip->soc_permyriad / 100, chip->soc_permyriad % 100, v_uv,
		i_ua, ocv_uv, weight);

	return DIV_ROUND_CLOSEST(chip->soc_permyriad, 100) !=
	       DIV_ROUND_CLOSEST(was, 100);
}

static void smb_fg_work(struct work_struct *work)
{
	struct smb_chip *chip = container_of(work, struct smb_chip,
					     fg_work.work);

	if (smb_fg_update(chip))
		power_supply_changed(chip->batt_psy);

	schedule_delayed_work(&chip->fg_work, msecs_to_jiffies(SMB_FG_POLL_MS));
}

/**
 * smb_fg_start() - seed the fuel gauge and start integrating it
 * @chip: charger
 *
 * An integrating gauge is only ever as good as what it started from, and the
 * live sample is a poor start: at probe the machine is booting, which is the
 * least rested the battery gets. The gauge itself has a better answer already
 * measured. It samples an open-circuit voltage twice under conditions we
 * cannot recreate here - once at power-on before anything draws, and again
 * whenever the PMIC has seen the current stay near zero for long enough - and
 * keeps both. Take the second in preference to the first, since the first can
 * be arbitrarily old, and fall back to the live sample if the PMIC holds
 * neither.
 */
static int smb_fg_start(struct smb_chip *chip)
{
	static const unsigned int rest_ocv[] = {
		QG_S3_GOOD_OCV_V_DATA0,
		QG_S7_PON_OCV_V_DATA0,
	};
	int v_uv, i_ua, soc, rc, i;

	rc = devm_mutex_init(chip->dev, &chip->fg_lock);
	if (rc)
		return rc;

	for (i = 0; i < ARRAY_SIZE(rest_ocv); i++) {
		if (smb_qg_read_sample(chip, rest_ocv[i], &v_uv, &i_ua) < 0)
			continue;

		soc = smb_ocv_to_permyriad(chip, smb_batt_ocv(chip, v_uv, i_ua));
		if (soc < 0)
			continue;

		chip->soc_permyriad = soc;
		chip->fg_ready = true;
		/* so the first poll integrates rather than re-anchoring */
		chip->fg_last = ktime_get_boottime();
		dev_dbg(chip->dev, "fg: seeded at %d.%02d%% from %duV\n",
			soc / 100, soc % 100, v_uv);
		break;
	}

	rc = devm_delayed_work_autocancel(chip->dev, &chip->fg_work,
					  smb_fg_work);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to init fuel-gauge work\n");

	schedule_delayed_work(&chip->fg_work, 0);
	return 0;
}

static int smb_get_batt_capacity(struct smb_chip *chip, int *val)
{
	int v_uv, cap, rc;

	if (!chip->batt_info)
		return -ENODATA;

	if (smb_fg_available(chip)) {
		guard(mutex)(&chip->fg_lock);

		if (!chip->fg_ready)
			return -EAGAIN;

		*val = DIV_ROUND_CLOSEST(chip->soc_permyriad, 100);
		return 0;
	}

	/*
	 * Without a gauge to integrate there is nothing to feed the OCV table
	 * but the terminal voltage, load and all - see the caveat above
	 * smb_batt_properties for what that costs.
	 */
	rc = smb_get_vbat(chip, &v_uv);
	if (rc < 0)
		return rc;

	cap = power_supply_batinfo_ocv2cap(chip->batt_info, v_uv, 25);
	if (cap < 0)
		return cap;

	*val = clamp(cap, 0, 100);
	return 0;
}

static int smb_get_batt_current(struct smb_chip *chip, int *val)
{
	int v_uv;

	if (!chip->var->qg_base)
		return -ENODATA;

	return smb_qg_read_sample(chip, QG_LAST_ADC_V_DATA0, &v_uv, val);
}

static int smb_get_batt_voltage(struct smb_chip *chip, bool open_circuit,
				int *val)
{
	int v_uv, i_ua, rc;

	/*
	 * Prefer the gauge's own converter over the ADC channel: it measures
	 * the voltage and the current at the same instant, which is what makes
	 * the pair of them usable together.
	 */
	if (chip->var->qg_base) {
		rc = smb_qg_read_sample(chip, QG_LAST_ADC_V_DATA0, &v_uv, &i_ua);
		if (!rc) {
			*val = open_circuit ? smb_batt_ocv(chip, v_uv, i_ua) : v_uv;
			return 0;
		}
	}

	if (open_circuit)
		return -ENODATA;

	return smb_get_vbat(chip, val);
}

static int smb_batt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct smb_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return smb_get_prop_status(chip, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return smb_get_prop_health(chip, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		return smb_get_batt_capacity(chip, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return smb_get_batt_voltage(chip, false, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return smb_get_batt_voltage(chip, true, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return smb_get_batt_current(chip, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return smb_get_batt_temp(chip, &val->intval);
	default:
		dev_err(chip->dev, "invalid battery property: %d\n", psp);
		return -EINVAL;
	}
}

static const struct power_supply_desc smb_batt_psy_desc = {
	.name = "pmi632-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb_batt_properties,
	.num_properties = ARRAY_SIZE(smb_batt_properties),
	.get_property = smb_batt_get_property,
};

static const struct power_supply_desc smb_psy_desc = {
	.name = "pmi8998_charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties = smb_properties,
	.num_properties = ARRAY_SIZE(smb_properties),
	.get_property = smb_get_property,
	.set_property = smb_set_property,
	.property_is_writeable = smb_property_is_writable,
};

/* Init sequence derived from vendor downstream driver (SMB2: pmi8998/pm660) */
static const struct smb_init_register smb2_init_seq[] = {
	{ .addr = AICL_RERUN_TIME_CFG, .mask = AICL_RERUN_TIME_MASK, .val = 0 },
	/*
	 * By default configure us as an upstream facing port
	 * FIXME: This will be handled by the type-c driver
	 */
	{ .addr = TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
	  .mask = TYPEC_POWER_ROLE_CMD_MASK | VCONN_EN_SRC_BIT |
		  VCONN_EN_VALUE_BIT,
	  .val = VCONN_EN_SRC_BIT },
	/*
	 * Disable Type-C factory mode and stay in Attached.SRC state when VCONN
	 * over-current happens
	 */
	{ .addr = TYPE_C_CFG,
	  .mask = FACTORY_MODE_DETECTION_EN_BIT | VCONN_OC_CFG_BIT,
	  .val = 0 },
	/* Configure VBUS for software control */
	{ .addr = OTG_CFG, .mask = OTG_EN_SRC_CFG_BIT, .val = 0 },
	/*
	 * Use VBAT to determine the recharge threshold when battery is full
	 * rather than the state of charge.
	 */
	{ .addr = FG_UPDATE_CFG_2_SEL,
	  .mask = SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT |
		  VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT,
	  .val = VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT },
	/* Enable charging */
	{ .addr = USBIN_OPTIONS_1_CFG, .mask = HVDCP_EN_BIT, .val = 0 },
	{ .addr = CHARGING_ENABLE_CMD,
	  .mask = CHARGING_ENABLE_CMD_BIT,
	  .val = CHARGING_ENABLE_CMD_BIT },
	/*
	 * Match downstream defaults
	 * CHG_EN_SRC_BIT - charger enable is controlled by software
	 * CHG_EN_POLARITY_BIT - polarity of charge enable pin when in HW control
	 *                       pulled low on OnePlus 6 and SHIFT6mq
	 * PRETOFAST_TRANSITION_CFG_BIT -
	 * BAT_OV_ECC_BIT -
	 * I_TERM_BIT - Current termination ?? 0 = enabled
	 * AUTO_RECHG_BIT - Enable automatic recharge when battery is full
	 *                  0 = enabled
	 * EN_ANALOG_DROP_IN_VBATT_BIT
	 * CHARGER_INHIBIT_BIT - Inhibit charging based on battery voltage
	 *                       instead of ??
	 */
	{ .addr = CHGR_CFG2,
	  .mask = CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT |
		  PRETOFAST_TRANSITION_CFG_BIT | BAT_OV_ECC_BIT | I_TERM_BIT |
		  AUTO_RECHG_BIT | EN_ANALOG_DROP_IN_VBATT_BIT |
		  CHARGER_INHIBIT_BIT,
	  .val = CHARGER_INHIBIT_BIT },
	/* STAT pin software override, match downstream. Parallel charging? */
	{ .addr = STAT_CFG,
	  .mask = STAT_SW_OVERRIDE_CFG_BIT,
	  .val = STAT_SW_OVERRIDE_CFG_BIT },
	/* Set the default SDP charger type to a 500ma USB 2.0 port */
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USB51_MODE_BIT | USBIN_MODE_CHG_BIT,
	  .val = USB51_MODE_BIT },
	/* Disable watchdog */
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = 0xff, .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	/* These bits aren't documented anywhere */
	{ .addr = USBIN_5V_AICL_THRESHOLD_CFG,
	  .mask = USBIN_5V_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	{ .addr = USBIN_CONT_AICL_THRESHOLD_CFG,
	  .mask = USBIN_CONT_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	/*
	 * Enable Automatic Input Current Limit, this will slowly ramp up the current
	 * When connected to a wall charger, and automatically stop when it detects
	 * the charger current limit (voltage drop?) or it reaches the programmed limit.
	 */
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_START_AT_MAX_BIT | USBIN_AICL_ADC_EN_BIT |
		  USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT |
		  USBIN_HV_COLLAPSE_RESPONSE_BIT |
		  USBIN_LV_COLLAPSE_RESPONSE_BIT,
	  .val = USBIN_HV_COLLAPSE_RESPONSE_BIT |
		 USBIN_LV_COLLAPSE_RESPONSE_BIT | USBIN_AICL_EN_BIT },
	/*
	 * Set pre charge current to default, the OnePlus 6 bootloader
	 * sets this very conservatively.
	 */
	{ .addr = PRE_CHARGE_CURRENT_CFG,
	  .mask = PRE_CHARGE_CURRENT_SETTING_MASK,
	  .val = 500000 / CURRENT_SCALE_FACTOR },
	/*
	 * This overrides all of the current limit options exposed to userspace
	 * and prevents the device from pulling more than ~1A. This is done
	 * to minimise potential fire hazard risks. A board that describes a
	 * monitored-battery gets that battery's current instead.
	 */
	{ .addr = FAST_CHARGE_CURRENT_CFG,
	  .mask = FAST_CHARGE_CURRENT_SETTING_MASK,
	  .val = 1000000 / CURRENT_SCALE_FACTOR },
};

/*
 * SMB5 (pmi632) current registers step in 50mA (not 25mA), so the FCC/pre-charge
 * raw values below use 50000 uA/LSB. Type-C, OTG and STAT/recharge-select writes
 * from the SMB2 sequence are intentionally omitted: on SMB5 the Type-C block lives
 * at its own base (0x1500, owned here by the qcom,pmi632-typec / tcpm driver) and
 * those SMB2 offsets would hit the wrong registers. Only CHGR, USBIN (AICL/ICL/
 * HVDCP) and MISC (watchdog) writes are kept — these share offsets with SMB2.
 */
#define SMB5_CURRENT_SCALE_FACTOR			50000
static const struct smb_init_register pmi632_init_seq[] = {
	/* Enable charging */
	{ .addr = USBIN_OPTIONS_1_CFG, .mask = HVDCP_EN_BIT, .val = 0 },
	{ .addr = CHARGING_ENABLE_CMD,
	  .mask = CHARGING_ENABLE_CMD_BIT,
	  .val = CHARGING_ENABLE_CMD_BIT },
	/* Charger enable controlled by software; inhibit based on battery voltage */
	{ .addr = CHGR_CFG2,
	  .mask = CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT |
		  PRETOFAST_TRANSITION_CFG_BIT | BAT_OV_ECC_BIT | I_TERM_BIT |
		  AUTO_RECHG_BIT | EN_ANALOG_DROP_IN_VBATT_BIT |
		  CHARGER_INHIBIT_BIT,
	  .val = CHARGER_INHIBIT_BIT },
	/* Default SDP charger to a 500mA USB 2.0 port */
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USB51_MODE_BIT | USBIN_MODE_CHG_BIT,
	  .val = USB51_MODE_BIT },
	/* Disable the charger watchdog */
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = 0xff, .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	{ .addr = USBIN_5V_AICL_THRESHOLD_CFG,
	  .mask = USBIN_5V_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	{ .addr = USBIN_CONT_AICL_THRESHOLD_CFG,
	  .mask = USBIN_CONT_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	/* Enable Automatic Input Current Limit */
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_START_AT_MAX_BIT | USBIN_AICL_ADC_EN_BIT |
		  USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT |
		  USBIN_HV_COLLAPSE_RESPONSE_BIT |
		  USBIN_LV_COLLAPSE_RESPONSE_BIT,
	  .val = USBIN_HV_COLLAPSE_RESPONSE_BIT |
		 USBIN_LV_COLLAPSE_RESPONSE_BIT | USBIN_AICL_EN_BIT },
	{ .addr = PRE_CHARGE_CURRENT_CFG,
	  .mask = PRE_CHARGE_CURRENT_SETTING_MASK,
	  .val = 500000 / SMB5_CURRENT_SCALE_FACTOR },
	/*
	 * Fast-charge at ~1A until the battery is known. A board that describes
	 * a monitored-battery gets its constant-charge-current-max-microamp
	 * instead.
	 */
	{ .addr = FAST_CHARGE_CURRENT_CFG,
	  .mask = FAST_CHARGE_CURRENT_SETTING_MASK,
	  .val = 1000000 / SMB5_CURRENT_SCALE_FACTOR },
};

static const struct smb_variant smb_variant_pmi8998 = {
	.name = "pmi8998",
	.status_base = 0x600,
	.current_scale_ua = CURRENT_SCALE_FACTOR,
	.fcc_max_ua = 4500000,
	.float_base_uv = 3480000,	/* (v - 3487500) / 7500 + 1 == (v - 3480000) / 7500 */
	.float_step_uv = 7500,
	.ov_bit = CHARGER_ERROR_STATUS_BAT_OV_BIT,
	.charge_status = smb2_charge_status,
	.inhibit_code = 6,
	.temp_status_reg = BATTERY_CHARGER_STATUS_2,
	.temp_status_shift = 0,
	.init_seq = smb2_init_seq,
	.init_seq_len = ARRAY_SIZE(smb2_init_seq),
};

static const struct smb_variant smb_variant_pm660 = {
	.name = "pm660",
	.status_base = 0x600,
	.current_scale_ua = CURRENT_SCALE_FACTOR,
	.fcc_max_ua = 4500000,
	.float_base_uv = 3480000,
	.float_step_uv = 7500,
	.ov_bit = CHARGER_ERROR_STATUS_BAT_OV_BIT,
	.charge_status = smb2_charge_status,
	.inhibit_code = 6,
	.temp_status_reg = BATTERY_CHARGER_STATUS_2,
	.temp_status_shift = 0,
	.init_seq = smb2_init_seq,
	.init_seq_len = ARRAY_SIZE(smb2_init_seq),
};

static const struct smb_variant smb_variant_pmi632 = {
	.name = "pmi632",
	.status_base = 0x100,		/* ICL/POWER_PATH_STATUS in DCDC, not MISC */
	.current_scale_ua = SMB5_CURRENT_SCALE_FACTOR,
	.fcc_max_ua = 3000000,
	.float_base_uv = 3600000,	/* qpnp-smb5 fv: min 3600000, step 10000 */
	.float_step_uv = 10000,
	.ov_bit = SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT,
	.charge_status = smb5_charge_status,
	.inhibit_code = 0,
	/*
	 * On SMB5 the JEITA temperature-status bits moved out of
	 * BATTERY_CHARGER_STATUS_2 into BATTERY_CHARGER_STATUS_7, and the
	 * HOT_SOFT/COLD_SOFT/TOO_HOT/TOO_COLD bits shifted up by two
	 * (verified against the downstream qpnp-smb5 smb5-reg.h and read back
	 * on a Fairphone 3: STATUS_2=0x28 is reserved data, STATUS_7=0x00).
	 */
	.temp_status_reg = BATTERY_CHARGER_STATUS_7,
	.temp_status_shift = 2,
	.init_seq = pmi632_init_seq,
	.init_seq_len = ARRAY_SIZE(pmi632_init_seq),
	.qg_base = 0x4800,
};

static int smb_init_hw(struct smb_chip *chip)
{
	int rc, i;

	for (i = 0; i < chip->var->init_seq_len; i++) {
		dev_dbg(chip->dev, "%d: Writing 0x%02x to 0x%02x\n", i,
			chip->var->init_seq[i].val, chip->var->init_seq[i].addr);
		rc = regmap_update_bits(chip->regmap,
					chip->base + chip->var->init_seq[i].addr,
					chip->var->init_seq[i].mask,
					chip->var->init_seq[i].val);
		if (rc < 0)
			return dev_err_probe(chip->dev, rc,
					     "%s: init command %d failed\n",
					     __func__, i);
	}

	return 0;
}

/*
 * Write one four-byte JEITA comparator block from a property on the battery
 * node holding the pair of raw BAT_THERM ADC codes { cold, hot }. Absent
 * property leaves the PMIC's power-on defaults in place.
 */
static int smb_set_jeita_thresholds(struct smb_chip *chip,
				    struct fwnode_handle *batt,
				    const char *prop, unsigned int reg)
{
	u32 thresh[2];
	u8 buf[JEITA_THRESHOLDS_LEN];
	int rc;

	rc = fwnode_property_read_u32_array(batt, prop, thresh,
					    ARRAY_SIZE(thresh));
	if (rc == -EINVAL)
		return 0;
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't read %s\n", prop);

	if (thresh[0] > U16_MAX || thresh[1] > U16_MAX || thresh[1] >= thresh[0])
		return dev_err_probe(chip->dev, -EINVAL,
				     "%s: expected { cold, hot } ADC codes with cold > hot, got { %u, %u }\n",
				     prop, thresh[0], thresh[1]);

	put_unaligned_be16(thresh[1], &buf[0]);
	put_unaligned_be16(thresh[0], &buf[2]);

	rc = regmap_bulk_write(chip->regmap, chip->base + reg, buf, sizeof(buf));
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't write %s\n", prop);

	return 0;
}

/*
 * A board that can be fitted with more than one battery tells them apart by a
 * resistor in the pack, read through a divider against the ADC's reference.
 *
 * This driver cannot *choose* between two batteries - a power supply has one
 * monitored-battery and there is no binding for more - but it can refuse to
 * apply one battery's limits to a different battery, which is the failure that
 * actually hurts: a cell charged to another cell's currents and temperature
 * limits, silently, because the device tree names only one.
 *
 * Returns 1 when the described battery is the one fitted, or when nothing here
 * says otherwise; 0 when it demonstrably is not; negative on error.
 */
static int smb_verify_battery_id(struct smb_chip *chip)
{
	struct fwnode_handle *batt __free(fwnode_handle) =
		fwnode_find_reference(dev_fwnode(chip->dev),
				      "monitored-battery", 0);
	u32 expect_ohm, pullup_ohm;
	u32 tol_pct = BATT_ID_DEFAULT_TOLERANCE_PCT;
	int rc, uv, ohm;

	if (IS_ERR(batt))
		return 1;

	/* All three have to be described before there is anything to check. */
	if (fwnode_property_read_u32(batt, "qcom,batt-id-ohm", &expect_ohm) ||
	    device_property_read_u32(chip->dev, "qcom,batt-id-pullup-ohm",
				     &pullup_ohm) ||
	    !chip->bat_id_chan)
		return 1;

	fwnode_property_read_u32(batt, "qcom,batt-id-tolerance-percent",
				 &tol_pct);

	rc = iio_read_channel_processed(chip->bat_id_chan, &uv);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read the battery ID\n");

	if (uv <= 0 || uv >= BATT_ID_VREF_UV) {
		dev_err(chip->dev,
			"Battery ID line reads %d uV: open or shorted\n", uv);
		return 0;
	}

	ohm = div_u64((u64)pullup_ohm * uv, BATT_ID_VREF_UV - uv);

	if (abs(ohm - (int)expect_ohm) * 100 > (int)expect_ohm * (int)tol_pct) {
		dev_err(chip->dev,
			"Battery ID is %d ohm, but the described battery is %u ohm +/-%u%%: not applying its charging limits\n",
			ohm, expect_ohm, tol_pct);
		return 0;
	}

	dev_dbg(chip->dev, "Battery ID %d ohm matches the described %u ohm\n",
		ohm, expect_ohm);

	return 1;
}

/*
 * Hardware JEITA. The hard thresholds stop charging outright and need no
 * enabling; the soft ones only do something once the corresponding
 * compensation bit is set, and then they subtract a fixed offset from the
 * fast-charge current. Express that offset as the current we want to be left
 * with in each soft zone, so the battery node carries a charge current rather
 * than a register delta.
 *
 * All of it is read from the monitored battery, not from this device: which
 * temperatures a cell may be charged at, and how hard, is a property of the
 * cell. A board that fits more than one pack describes each of them.
 */
static int smb_init_jeita(struct smb_chip *chip)
{
	struct fwnode_handle *batt __free(fwnode_handle) =
		fwnode_find_reference(dev_fwnode(chip->dev),
				      "monitored-battery", 0);
	unsigned int fcc_ua, comp_hot, comp_cold;
	u32 soft_fcc_ua[2];
	int rc;

	if (IS_ERR(batt))
		return 0;

	rc = smb_set_jeita_thresholds(chip, batt, "qcom,jeita-hard-thresholds",
				      JEITA_HARD_THRESHOLDS);
	if (rc < 0)
		return rc;

	rc = smb_set_jeita_thresholds(chip, batt, "qcom,jeita-soft-thresholds",
				      JEITA_SOFT_THRESHOLDS);
	if (rc < 0)
		return rc;

	rc = fwnode_property_read_u32_array(batt,
					    "qcom,jeita-soft-fcc-microamp",
					    soft_fcc_ua,
					    ARRAY_SIZE(soft_fcc_ua));
	if (rc == -EINVAL)
		return 0;
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read qcom,jeita-soft-fcc-microamp\n");

	/*
	 * Read the fast-charge current back out of the hardware rather than
	 * taking it from the device tree: the compensation is a subtraction
	 * from whatever is actually programmed.
	 */
	rc = smb_get_fast_charge_current(chip, &fcc_ua);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read the fast-charge current\n");

	if (soft_fcc_ua[0] > fcc_ua || soft_fcc_ua[1] > fcc_ua)
		return dev_err_probe(chip->dev, -EINVAL,
				     "JEITA soft-zone current { %u, %u } exceeds the fast-charge current %u\n",
				     soft_fcc_ua[0], soft_fcc_ua[1], fcc_ua);

	/* Round the reduction up, so the result is never above what was asked. */
	comp_cold = DIV_ROUND_UP(fcc_ua - soft_fcc_ua[0], JEITA_CCCOMP_STEP_UA);
	comp_hot = DIV_ROUND_UP(fcc_ua - soft_fcc_ua[1], JEITA_CCCOMP_STEP_UA);

	if (comp_cold > JEITA_CCCOMP_MASK || comp_hot > JEITA_CCCOMP_MASK)
		return dev_err_probe(chip->dev, -ERANGE,
				     "JEITA soft-zone current { %u, %u } is more than %u uA below the fast-charge current %u\n",
				     soft_fcc_ua[0], soft_fcc_ua[1],
				     (unsigned int)JEITA_CCCOMP_MASK *
					     JEITA_CCCOMP_STEP_UA,
				     fcc_ua);

	rc = regmap_update_bits(chip->regmap,
				chip->base + JEITA_CCCOMP_CFG_COLD,
				JEITA_CCCOMP_MASK, comp_cold);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't set the cold JEITA compensation\n");

	rc = regmap_update_bits(chip->regmap, chip->base + JEITA_CCCOMP_CFG_HOT,
				JEITA_CCCOMP_MASK, comp_hot);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't set the hot JEITA compensation\n");

	/*
	 * Only the charge-current halves are enabled here. The float-voltage
	 * ones are left as the PMIC defaults them: this driver has no way to
	 * describe the voltage reduction they apply.
	 */
	rc = regmap_update_bits(chip->regmap, chip->base + JEITA_EN_CFG,
				JEITA_EN_HOT_SL_CCC_BIT | JEITA_EN_COLD_SL_CCC_BIT,
				JEITA_EN_HOT_SL_CCC_BIT | JEITA_EN_COLD_SL_CCC_BIT);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't enable JEITA compensation\n");

	dev_dbg(chip->dev,
		"JEITA: fcc %u uA, soft-zone cold %u uA (-%u), hot %u uA (-%u)\n",
		fcc_ua, soft_fcc_ua[0], comp_cold * JEITA_CCCOMP_STEP_UA,
		soft_fcc_ua[1], comp_hot * JEITA_CCCOMP_STEP_UA);

	return 0;
}

static int smb_tcd_get_max_state(struct thermal_cooling_device *tcd,
				 unsigned long *state)
{
	struct smb_chip *chip = tcd->devdata;

	*state = chip->thermal_levels - 1;

	return 0;
}

static int smb_tcd_get_cur_state(struct thermal_cooling_device *tcd,
				 unsigned long *state)
{
	struct smb_chip *chip = tcd->devdata;

	*state = chip->thermal_level;

	return 0;
}

static int smb_tcd_set_cur_state(struct thermal_cooling_device *tcd,
				 unsigned long state)
{
	struct smb_chip *chip = tcd->devdata;
	int rc;

	if (state >= chip->thermal_levels)
		return -EINVAL;

	rc = smb_set_fast_charge_current(chip,
					 chip->thermal_mitigation_ua[state]);
	if (rc < 0)
		return rc;

	chip->thermal_level = state;

	return 0;
}

static const struct thermal_cooling_device_ops smb_tcd_ops = {
	.get_max_state = smb_tcd_get_max_state,
	.get_cur_state = smb_tcd_get_cur_state,
	.set_cur_state = smb_tcd_set_cur_state,
};

/*
 * Expose the fast-charge current as a cooling device, so a thermal zone can
 * throttle charging the way it throttles a CPU. State 0 is the unmitigated
 * current the board asked for and each further state is lower.
 *
 * The JEITA soft-zone compensation is a fixed subtraction from whatever is
 * programmed here, so a mitigated current stays mitigated in the soft zones
 * too.
 */
static int smb_init_cooling(struct smb_chip *chip)
{
	struct thermal_cooling_device *tcd;
	unsigned int fcc_ua;
	int count, i, rc;

	count = device_property_count_u32(chip->dev, "qcom,thermal-mitigation");
	if (count == -EINVAL)
		return 0;
	if (count < 0)
		return dev_err_probe(chip->dev, count,
				     "Couldn't read qcom,thermal-mitigation\n");
	if (count < 2)
		return dev_err_probe(chip->dev, -EINVAL,
				     "qcom,thermal-mitigation needs at least two states\n");

	chip->thermal_mitigation_ua = devm_kcalloc(chip->dev, count,
						   sizeof(*chip->thermal_mitigation_ua),
						   GFP_KERNEL);
	if (!chip->thermal_mitigation_ua)
		return -ENOMEM;

	rc = device_property_read_u32_array(chip->dev, "qcom,thermal-mitigation",
					    chip->thermal_mitigation_ua, count);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read qcom,thermal-mitigation\n");

	rc = smb_get_fast_charge_current(chip, &fcc_ua);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read the fast-charge current\n");

	for (i = 1; i < count; i++)
		if (chip->thermal_mitigation_ua[i] >
		    chip->thermal_mitigation_ua[i - 1])
			return dev_err_probe(chip->dev, -EINVAL,
					     "qcom,thermal-mitigation must not increase (state %d)\n",
					     i);

	/*
	 * Mitigation may only ever reduce. The table is written for the current
	 * the board expects to charge at, and the charger may be running below
	 * that - on the init-sequence default, because the fitted battery could
	 * not be identified - in which case a state must not raise it back up.
	 */
	for (i = 0; i < count; i++)
		chip->thermal_mitigation_ua[i] =
			min(chip->thermal_mitigation_ua[i], fcc_ua);

	chip->thermal_levels = count;

	tcd = devm_thermal_of_cooling_device_register(chip->dev,
						      dev_of_node(chip->dev),
						      "qcom-smbx-charger", chip,
						      &smb_tcd_ops);
	if (IS_ERR(tcd))
		return dev_err_probe(chip->dev, PTR_ERR(tcd),
				     "Couldn't register the cooling device\n");

	return 0;
}

static int smb_init_irq(struct smb_chip *chip, int *irq, const char *name,
			 irqreturn_t (*handler)(int irq, void *data))
{
	int irqnum;
	int rc;

	irqnum = platform_get_irq_byname(to_platform_device(chip->dev), name);
	if (irqnum < 0)
		return irqnum;

	rc = devm_request_threaded_irq(chip->dev, irqnum, NULL, handler,
				       IRQF_ONESHOT, name, chip);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't request irq %s\n",
				     name);

	if (irq)
		*irq = irqnum;

	return 0;
}

static int smb_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct power_supply_desc *desc;
	struct smb_chip *chip;
	int rc, irq;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->name = pdev->name;

	chip->var = device_get_match_data(&pdev->dev);
	if (!chip->var)
		return dev_err_probe(chip->dev, -ENODEV,
				     "no match data for device\n");

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "failed to locate the regmap\n");

	rc = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read base address\n");

	chip->usb_in_v_chan = devm_iio_channel_get(chip->dev, "usbin_v");
	if (IS_ERR(chip->usb_in_v_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_v_chan),
				     "Couldn't get usbin_v IIO channel\n");

	chip->usb_in_i_chan = devm_iio_channel_get(chip->dev, "usbin_i");
	if (IS_ERR(chip->usb_in_i_chan)) {
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_i_chan),
				     "Couldn't get usbin_i IIO channel\n");
	}

	/*
	 * VBAT_SNS is optional: it drives the voltage-based fuel gauge and is
	 * only present on variants that wire it up (PMI632). Defer if the ADC
	 * isn't ready yet, otherwise carry on without a battery gauge.
	 */
	chip->vbat_chan = devm_iio_channel_get(chip->dev, "vbat");
	if (IS_ERR(chip->vbat_chan)) {
		rc = PTR_ERR(chip->vbat_chan);
		if (rc == -EPROBE_DEFER)
			return rc;
		chip->vbat_chan = NULL;
	}

	/*
	 * BAT_THERM is optional in the same way: only boards that route the
	 * pack thermistor to the PMIC can report a battery temperature.
	 */
	chip->bat_therm_chan = devm_iio_channel_get(chip->dev, "bat_therm");
	if (IS_ERR(chip->bat_therm_chan)) {
		rc = PTR_ERR(chip->bat_therm_chan);
		if (rc == -EPROBE_DEFER)
			return rc;
		chip->bat_therm_chan = NULL;
	}

	/*
	 * BAT_ID likewise: only boards that can be fitted with more than one
	 * battery have a reason to route it.
	 */
	chip->bat_id_chan = devm_iio_channel_get(chip->dev, "bat_id");
	if (IS_ERR(chip->bat_id_chan)) {
		rc = PTR_ERR(chip->bat_id_chan);
		if (rc == -EPROBE_DEFER)
			return rc;
		chip->bat_id_chan = NULL;
	}

	rc = smb_init_hw(chip);
	if (rc < 0)
		return rc;

	supply_config.drv_data = chip;
	supply_config.fwnode = dev_fwnode(&pdev->dev);

	desc = devm_kzalloc(chip->dev, sizeof(smb_psy_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &smb_psy_desc, sizeof(smb_psy_desc));
	desc->name =
		devm_kasprintf(chip->dev, GFP_KERNEL, "%s-charger",
			       chip->var->name);
	if (!desc->name)
		return -ENOMEM;

	chip->chg_psy =
		devm_power_supply_register(chip->dev, desc, &supply_config);
	if (IS_ERR(chip->chg_psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->chg_psy),
				     "failed to register power supply\n");

	rc = power_supply_get_battery_info(chip->chg_psy, &chip->batt_info);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to get battery info\n");

	/*
	 * Register a battery (fuel-gauge) power supply only when we can
	 * actually estimate state of charge: a VBAT channel plus an
	 * OCV->capacity table from the monitored-battery node.
	 */
	if (chip->vbat_chan && chip->batt_info->ocv_table[0]) {
		chip->batt_psy = devm_power_supply_register(chip->dev,
							    &smb_batt_psy_desc,
							    &supply_config);
		if (IS_ERR(chip->batt_psy))
			return dev_err_probe(chip->dev, PTR_ERR(chip->batt_psy),
					     "failed to register battery power supply\n");

		if (smb_fg_available(chip)) {
			rc = smb_fg_start(chip);
			if (rc < 0)
				return rc;
		}
	}

	rc = devm_delayed_work_autocancel(chip->dev, &chip->status_change_work,
					  smb_status_change_work);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to init status change work\n");

	rc = (chip->batt_info->voltage_max_design_uv - chip->var->float_base_uv) /
	     chip->var->float_step_uv;
	rc = regmap_update_bits(chip->regmap, chip->base + FLOAT_VOLTAGE_CFG,
				FLOAT_VOLTAGE_SETTING_MASK, rc);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't set vbat max\n");

	/*
	 * Everything below describes the *battery*, so none of it may be
	 * applied to a battery that is not the one described. Where the board
	 * gives us a way to tell, check first; a mismatch leaves the charger on
	 * the init sequence's conservative defaults rather than on another
	 * cell's limits.
	 */
	rc = smb_verify_battery_id(chip);
	if (rc < 0)
		return rc;

	if (rc > 0) {
		/*
		 * Let the battery say what it will take, bounded only by what
		 * the PMIC can physically deliver. Whether that current is
		 * appropriate is a property of the pack and of the board's
		 * thermal design, both of which the device tree describes and
		 * this file does not know.
		 */
		if (chip->batt_info->constant_charge_current_max_ua > 0) {
			u32 batt_ua = chip->batt_info->constant_charge_current_max_ua;
			unsigned int fcc_ua = min(chip->var->fcc_max_ua, batt_ua);

			rc = smb_set_fast_charge_current(chip, fcc_ua);
			if (rc < 0)
				return dev_err_probe(chip->dev, rc,
						     "Couldn't set the fast-charge current\n");
		}

		rc = smb_init_jeita(chip);
		if (rc < 0)
			return rc;
	}

	rc = smb_init_cooling(chip);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &irq, "bat-ov", smb_handle_batt_overvoltage);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &chip->cable_irq, "usb-plugin",
			   smb_handle_usb_plugin);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &irq, "usbin-icl-change",
			   smb_handle_usb_icl_change);
	if (rc < 0)
		return rc;
	rc = smb_init_irq(chip, &irq, "wdog-bark", smb_handle_wdog_bark);
	if (rc < 0)
		return rc;

	devm_device_init_wakeup(chip->dev);

	rc = devm_pm_set_wake_irq(chip->dev, chip->cable_irq);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't set wake irq\n");

	platform_set_drvdata(pdev, chip);

	/* Initialise charger state */
	schedule_delayed_work(&chip->status_change_work, 0);

	return 0;
}

static const struct of_device_id smb_match_id_table[] = {
	{ .compatible = "qcom,pmi8998-charger", .data = &smb_variant_pmi8998 },
	{ .compatible = "qcom,pm660-charger", .data = &smb_variant_pm660 },
	{ .compatible = "qcom,pmi632-charger", .data = &smb_variant_pmi632 },
	{ /* sentinal */ }
};
MODULE_DEVICE_TABLE(of, smb_match_id_table);

static struct platform_driver qcom_spmi_smb = {
	.probe = smb_probe,
	.driver = {
		.name = "qcom-smbx-charger",
		.of_match_table = smb_match_id_table,
		},
};

module_platform_driver(qcom_spmi_smb);

MODULE_AUTHOR("Casey Connolly <casey.connolly@linaro.org>");
MODULE_DESCRIPTION("Qualcomm SMB2 Charger Driver");
MODULE_LICENSE("GPL");
