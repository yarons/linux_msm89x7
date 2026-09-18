// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW87319 class-D speaker amplifier with integrated boost converter
 *
 * The chip has an analog input and no digital audio interface, so it is
 * modelled like aw8738/simple-amplifier: IN -> DRV -> OUT, with the register
 * programming done from the DAPM event of the DRV widget.
 *
 * There is no public datasheet. Register names, bit meanings and the power
 * sequences are taken from the Awinic vendor driver v1.2.1 shipped in the
 * Samsung SM-T290 msm-4.9 kernel (techpack/audio/asoc/aw87319l_audio.c):
 *
 *   aw87319_hw_reset()         reset low 2 ms, high 2 ms
 *   aw87319_hw_on()            the same, then reg 0x64 = 0x2c
 *   aw87319l_audio_speaker()   reg 0x01 without CHIP_EN, reg 0x02..0x09,
 *                              then reg 0x01 with CHIP_EN
 *   aw87319l_audio_receiver()  reg 0x01 without CHIP_EN, reg 0x05 only,
 *                              then reg 0x01 with CHIP_EN
 *   aw87319l_audio_off()       reg 0x01 = 0x00, reset low 2 ms
 *   aw87319_read_chipid()      reg 0x64 = 0x2c, read reg 0x00, 5 tries
 *
 * The register values are the tables Samsung ships as aw87319{l,r}_spk.bin
 * and aw87319l_rcv.bin (10 bytes each, one byte per register 0x00..0x09).
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#define AW87319_REG_CHIPID		0x00
#define AW87319_REG_SYSCTRL		0x01
/*
 * 0x02..0x09 carry the names the vendor driver uses in its comments
 * ("BATSAFE", "BOV", "BP", "Gain", "AGC3_Po", "AGC3", "AGC2", "AGC1").
 * BOV and BP presumably are the boost output voltage and the boost peak
 * current limit; the bit layout of these registers is not known.
 */
#define AW87319_REG_BATSAFE		0x02
#define AW87319_REG_BOV			0x03
#define AW87319_REG_BP			0x04
#define AW87319_REG_GAIN		0x05
#define AW87319_REG_AGC3_PO		0x06
#define AW87319_REG_AGC3		0x07
#define AW87319_REG_AGC2		0x08
#define AW87319_REG_AGC1		0x09
/*
 * Not named by the vendor driver, which writes 0x2c to it after every
 * hardware reset and before every chip id read. Kept as is.
 */
#define AW87319_REG_INIT		0x64
#define AW87319_INIT_VALUE		0x2c

/*
 * SYSCTRL bits, from the vendor comments: 0x07 is "CHIP Enable; Class D
 * Enable; Boost Enable", 0x06 the same with "Boost Disable", and the value
 * masked with 0xfb is the one without "CHIP Enable". Bit 1 follows by
 * elimination. The chip resets to 0x03.
 */
#define AW87319_SYSCTRL_BOOST_EN	BIT(0)
#define AW87319_SYSCTRL_CLASSD_EN	BIT(1)
#define AW87319_SYSCTRL_CHIP_EN		BIT(2)

/*
 * The vendor driver only accepts 0x9b and its register tables carry 0x9b as
 * byte 0. The two amplifiers of the SM-T290 this driver was written for
 * answer with 0x9a instead when read right after releasing the reset line
 * (without the 0x64 write the vendor driver does first). Whether that is a
 * different silicon revision or an effect of the missing 0x64 write is not
 * known, so accept both.
 */
#define AW87319_CHIPID			0x9b
#define AW87319_CHIPID_ALT		0x9a

#define AW87319_CHIPID_RETRIES		5
#define AW87319_RESET_DELAY_US		2000

enum aw87319_mode {
	AW87319_MODE_SPEAKER,
	AW87319_MODE_RECEIVER,
};

struct aw87319_profile {
	u8 sysctrl;
	const struct reg_sequence *regs;
	unsigned int num_regs;
};

struct aw87319 {
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock; /* serializes the power sequences and mode changes */
	unsigned int mode;
};

/* aw87319l_spk.bin / aw87319r_spk.bin: 9b 07 38 05 02 0d 0b b2 a0 05 */
static const struct reg_sequence aw87319_speaker_regs[] = {
	{ AW87319_REG_BATSAFE, 0x38 },
	{ AW87319_REG_BOV, 0x05 },
	{ AW87319_REG_BP, 0x02 },
	{ AW87319_REG_GAIN, 0x0d },
	{ AW87319_REG_AGC3_PO, 0x0b },
	{ AW87319_REG_AGC3, 0xb2 },
	{ AW87319_REG_AGC2, 0xa0 },
	{ AW87319_REG_AGC1, 0x05 },
};

/*
 * aw87319l_rcv.bin: 9b 06 00 05 04 02 0b 52 a8 03. The vendor receiver
 * sequence only uses the SYSCTRL and GAIN bytes of it, the other registers
 * stay at their reset values.
 */
static const struct reg_sequence aw87319_receiver_regs[] = {
	{ AW87319_REG_GAIN, 0x02 },
};

static const struct aw87319_profile aw87319_profiles[] = {
	[AW87319_MODE_SPEAKER] = {
		.sysctrl = AW87319_SYSCTRL_CHIP_EN | AW87319_SYSCTRL_CLASSD_EN |
			   AW87319_SYSCTRL_BOOST_EN,
		.regs = aw87319_speaker_regs,
		.num_regs = ARRAY_SIZE(aw87319_speaker_regs),
	},
	[AW87319_MODE_RECEIVER] = {
		.sysctrl = AW87319_SYSCTRL_CHIP_EN | AW87319_SYSCTRL_CLASSD_EN,
		.regs = aw87319_receiver_regs,
		.num_regs = ARRAY_SIZE(aw87319_receiver_regs),
	},
};

static void aw87319_hw_reset(struct aw87319 *aw)
{
	if (!aw->reset_gpio)
		return;

	gpiod_set_value_cansleep(aw->reset_gpio, 1);
	usleep_range(AW87319_RESET_DELAY_US, AW87319_RESET_DELAY_US + 500);
	gpiod_set_value_cansleep(aw->reset_gpio, 0);
	usleep_range(AW87319_RESET_DELAY_US, AW87319_RESET_DELAY_US + 500);
}

static void aw87319_hw_on(struct aw87319 *aw)
{
	/*
	 * The vendor driver does not check this write and what it does is not
	 * known, so do not let it get in the way: a dead bus shows up in the
	 * SYSCTRL write that follows.
	 */
	aw87319_hw_reset(aw);
	regmap_write(aw->regmap, AW87319_REG_INIT, AW87319_INIT_VALUE);
}

static void aw87319_hw_off(struct aw87319 *aw)
{
	if (!aw->reset_gpio)
		return;

	gpiod_set_value_cansleep(aw->reset_gpio, 1);
	usleep_range(AW87319_RESET_DELAY_US, AW87319_RESET_DELAY_US + 500);
}

static int aw87319_power_up(struct aw87319 *aw)
{
	const struct aw87319_profile *profile = &aw87319_profiles[aw->mode];
	int ret;

	aw87319_hw_on(aw);

	/* Configure with the output stage still disabled, enable it last */
	ret = regmap_write(aw->regmap, AW87319_REG_SYSCTRL,
			   profile->sysctrl & ~AW87319_SYSCTRL_CHIP_EN);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(aw->regmap, profile->regs,
				     profile->num_regs);
	if (ret)
		return ret;

	return regmap_write(aw->regmap, AW87319_REG_SYSCTRL, profile->sysctrl);
}

static int aw87319_power_down(struct aw87319 *aw)
{
	int ret;

	ret = regmap_write(aw->regmap, AW87319_REG_SYSCTRL, 0x00);
	aw87319_hw_off(aw);

	return ret;
}

static int aw87319_drv_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct aw87319 *aw = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&aw->lock);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = aw87319_power_up(aw);
		if (ret) {
			dev_err(component->dev, "power up failed: %d\n", ret);
			aw87319_hw_off(aw);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = aw87319_power_down(aw);
		if (ret)
			dev_err(component->dev, "power down failed: %d\n", ret);
		break;
	default:
		WARN(1, "Unexpected event");
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&aw->lock);

	return ret;
}

static const char * const aw87319_mode_texts[] = {
	[AW87319_MODE_SPEAKER] = "Speaker",
	[AW87319_MODE_RECEIVER] = "Receiver",
};

static SOC_ENUM_SINGLE_EXT_DECL(aw87319_mode_enum, aw87319_mode_texts);

static int aw87319_mode_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct aw87319 *aw = snd_soc_component_get_drvdata(component);

	mutex_lock(&aw->lock);
	ucontrol->value.enumerated.item[0] = aw->mode;
	mutex_unlock(&aw->lock);

	return 0;
}

/* A new mode is used the next time the amplifier is powered up */
static int aw87319_mode_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct aw87319 *aw = snd_soc_component_get_drvdata(component);
	unsigned int mode = ucontrol->value.enumerated.item[0];
	int changed;

	if (mode >= ARRAY_SIZE(aw87319_profiles))
		return -EINVAL;

	mutex_lock(&aw->lock);
	changed = aw->mode != mode;
	aw->mode = mode;
	mutex_unlock(&aw->lock);

	return changed;
}

static const struct snd_kcontrol_new aw87319_controls[] = {
	SOC_ENUM_EXT("Mode", aw87319_mode_enum,
		     aw87319_mode_get, aw87319_mode_put),
};

static const struct snd_soc_dapm_widget aw87319_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_OUT_DRV_E("DRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       aw87319_drv_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route aw87319_dapm_routes[] = {
	{ "DRV", NULL, "IN" },
	{ "OUT", NULL, "DRV" },
};

static const struct snd_soc_component_driver aw87319_component_driver = {
	.controls = aw87319_controls,
	.num_controls = ARRAY_SIZE(aw87319_controls),
	.dapm_widgets = aw87319_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw87319_dapm_widgets),
	.dapm_routes = aw87319_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(aw87319_dapm_routes),
};

static int aw87319_read_chipid(struct aw87319 *aw, unsigned int *id)
{
	int ret = -EIO;
	int i;

	for (i = 0; i < AW87319_CHIPID_RETRIES; i++) {
		if (i)
			usleep_range(AW87319_RESET_DELAY_US,
				     AW87319_RESET_DELAY_US + 500);

		/* As in the vendor driver, only the read decides */
		regmap_write(aw->regmap, AW87319_REG_INIT, AW87319_INIT_VALUE);

		ret = regmap_read(aw->regmap, AW87319_REG_CHIPID, id);
		if (ret)
			continue;

		if (*id == AW87319_CHIPID || *id == AW87319_CHIPID_ALT)
			return 0;

		ret = -ENODEV;
	}

	return ret;
}

static const struct regmap_config aw87319_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW87319_REG_INIT,
	/* The chip loses its registers on every power down: nothing to cache */
	.cache_type = REGCACHE_NONE,
};

static int aw87319_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	unsigned int id = 0;
	struct aw87319 *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &aw->lock);
	if (ret)
		return ret;

	i2c_set_clientdata(i2c, aw);

	aw->regmap = devm_regmap_init_i2c(i2c, &aw87319_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap),
				     "Failed to init regmap\n");

	/* Start in hardware shutdown, like the vendor driver */
	aw->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(aw->reset_gpio),
				     "Failed to get reset gpio\n");

	aw87319_hw_reset(aw);
	ret = aw87319_read_chipid(aw, &id);
	aw87319_hw_off(aw);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to identify chip (id 0x%02x)\n", id);

	dev_info(dev, "AW87319 chip id 0x%02x\n", id);

	return devm_snd_soc_register_component(dev, &aw87319_component_driver,
					       NULL, 0);
}

static const struct of_device_id aw87319_of_match[] = {
	{ .compatible = "awinic,aw87319" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw87319_of_match);

static const struct i2c_device_id aw87319_i2c_id[] = {
	{ "aw87319" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw87319_i2c_id);

static struct i2c_driver aw87319_i2c_driver = {
	.driver = {
		.name = "aw87319",
		.of_match_table = aw87319_of_match,
	},
	.probe = aw87319_i2c_probe,
	.id_table = aw87319_i2c_id,
};
module_i2c_driver(aw87319_i2c_driver);

MODULE_DESCRIPTION("Awinic AW87319 Amplifier Driver");
MODULE_LICENSE("GPL");
