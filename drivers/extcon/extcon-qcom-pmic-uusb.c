// SPDX-License-Identifier: GPL-2.0-only
/*
 * Micro-USB ID detection with the Type-C block of Qualcomm SMB5 PMICs
 *
 * PMI632 and its relatives detect Type-C connections with the block at 0x1500.
 * On boards with a micro-USB receptacle the same block runs in "micro USB
 * mode" (set up by the boot loader, or strapped) and reports the state of the
 * ID pin instead: floating, grounded (an OTG adapter), or one of the factory
 * resistors. This driver turns that into extcon cable states so that a dual
 * role USB controller can switch to host mode. VBUS for the host side comes
 * from the boost converter of the same PMIC, which has its own regulator
 * driver (qcom_usb_vbus-regulator).
 *
 * Register use follows the vendor smb5 charger driver
 * (smblib_uusb_otg_work() and the micro USB setup in smb5-lib.c).
 */

#include <linux/devm-helpers.h>
#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define TYPEC_U_USB_STATUS		0x0f
#define U_USB_GROUND_NOVBUS		BIT(6)
#define U_USB_GROUND			BIT(4)

#define TYPE_C_INTERRUPT_EN_CFG_2	0x60
#define MICRO_USB_STATE_CHANGE_INT_EN	BIT(5)

#define TYPEC_U_USB_CFG			0x70
#define EN_MICRO_USB_MODE		BIT(0)

/* The vendor driver looks at the status 100 ms after the interrupt */
#define UUSB_DEBOUNCE_MS		100

struct qcom_uusb {
	struct device *dev;
	struct regmap *regmap;
	struct extcon_dev *edev;
	struct delayed_work work;
	u32 base;
};

static const unsigned int qcom_uusb_cables[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void qcom_uusb_detect(struct work_struct *work)
{
	struct qcom_uusb *uusb = container_of(to_delayed_work(work),
					      struct qcom_uusb, work);
	unsigned int stat;
	bool host;
	int ret;

	ret = regmap_read(uusb->regmap, uusb->base + TYPEC_U_USB_STATUS, &stat);
	if (ret) {
		dev_err(uusb->dev, "failed to read the ID state: %d\n", ret);
		return;
	}

	host = stat & (U_USB_GROUND_NOVBUS | U_USB_GROUND);

	dev_dbg(uusb->dev, "ID status 0x%02x: %s\n", stat,
		host ? "host" : "device");

	/*
	 * There is no VBUS detection in this block. Anything that is not an
	 * OTG adapter is handed to the controller as a possible device
	 * connection; without a host on the other end nothing happens.
	 */
	extcon_set_state_sync(uusb->edev, EXTCON_USB_HOST, host);
	extcon_set_state_sync(uusb->edev, EXTCON_USB, !host);
}

static irqreturn_t qcom_uusb_irq(int irq, void *data)
{
	struct qcom_uusb *uusb = data;

	mod_delayed_work(system_power_efficient_wq, &uusb->work,
			 msecs_to_jiffies(UUSB_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static int qcom_uusb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_uusb *uusb;
	unsigned int cfg;
	int irq, ret;

	uusb = devm_kzalloc(dev, sizeof(*uusb), GFP_KERNEL);
	if (!uusb)
		return -ENOMEM;

	uusb->dev = dev;

	uusb->regmap = dev_get_regmap(dev->parent, NULL);
	if (!uusb->regmap)
		return dev_err_probe(dev, -ENODEV, "no regmap of the PMIC\n");

	ret = device_property_read_u32(dev, "reg", &uusb->base);
	if (ret)
		return dev_err_probe(dev, ret, "no register base\n");

	ret = regmap_read(uusb->regmap, uusb->base + TYPEC_U_USB_CFG, &cfg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the mode\n");

	if (!(cfg & EN_MICRO_USB_MODE))
		return dev_err_probe(dev, -ENODEV,
				     "the Type-C block is not in micro USB mode (0x%02x)\n",
				     cfg);

	uusb->edev = devm_extcon_dev_allocate(dev, qcom_uusb_cables);
	if (IS_ERR(uusb->edev))
		return PTR_ERR(uusb->edev);

	ret = devm_extcon_dev_register(dev, uusb->edev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register extcon\n");

	ret = devm_delayed_work_autocancel(dev, &uusb->work, qcom_uusb_detect);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, qcom_uusb_irq,
					IRQF_ONESHOT, dev_name(dev), uusb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the IRQ\n");

	/* The boot loader leaves the ID state change interrupt disabled */
	ret = regmap_update_bits(uusb->regmap,
				 uusb->base + TYPE_C_INTERRUPT_EN_CFG_2,
				 MICRO_USB_STATE_CHANGE_INT_EN,
				 MICRO_USB_STATE_CHANGE_INT_EN);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable the interrupt\n");

	platform_set_drvdata(pdev, uusb);
	device_init_wakeup(dev, true);

	/* Pick up an adapter that is already plugged in */
	mod_delayed_work(system_power_efficient_wq, &uusb->work, 0);

	return 0;
}

static const struct of_device_id qcom_uusb_of_match[] = {
	{ .compatible = "qcom,pmi632-uusb-extcon" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_uusb_of_match);

static struct platform_driver qcom_uusb_driver = {
	.probe = qcom_uusb_probe,
	.driver = {
		.name = "extcon-qcom-pmic-uusb",
		.of_match_table = qcom_uusb_of_match,
	},
};
module_platform_driver(qcom_uusb_driver);

MODULE_DESCRIPTION("Qualcomm SMB5 PMIC micro-USB ID extcon driver");
MODULE_LICENSE("GPL");
