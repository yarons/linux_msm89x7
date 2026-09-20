// SPDX-License-Identifier: GPL-2.0
/*
 * Giantec GT9772 voice coil motor (VCM) lens driver
 *
 * No datasheet is public. The register use below is what the vendor camera
 * stack of the Samsung Galaxy Tab A 8.0 2019 (SM-T290) does with the part
 * (libactuator_gt9772.so: i2c address 0x0c, 10 bit position written as one
 * big endian word to 0x03, init table of six writes). The layout matches the
 * Dongwoon DW9768: control at 0x02, position at 0x03/0x04, ringing control
 * setup at 0x06/0x07. The meaning of 0xed and 0x08 is not known.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define GT9772_MAX_FOCUS_POS		(1024 - 1)
#define GT9772_FOCUS_STEPS		1

#define GT9772_REG_CONTROL		0x02
#define GT9772_CONTROL_PD		BIT(0)
#define GT9772_REG_POS_MSB		0x03
#define GT9772_REG_RING_MODE		0x06
#define GT9772_REG_RING_TIME		0x07
#define GT9772_REG_VENDOR_08		0x08
#define GT9772_REG_UNLOCK		0xed

/* Time from power to the first access, and after leaving power down */
#define GT9772_T_POWER_US		5000
#define GT9772_T_OPR_US			1000

/*
 * Move in steps so that the lens does not hit its end stops: the vendor
 * tuning data damps with 13 to 15 ms per step; this is the same idea as in
 * dw9768, with its step size.
 */
#define GT9772_MOVE_STEPS		16
#define GT9772_MOVE_DELAY_US		3000

struct gt9772 {
	struct regulator *vdd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
	struct v4l2_subdev sd;
};

/* The vendor init table, in its order and with its delays */
static const struct {
	u8 reg;
	u8 val;
	unsigned int delay_us;
} gt9772_init_regs[] = {
	{ GT9772_REG_UNLOCK,	0xab, 0 },
	{ GT9772_REG_CONTROL,	GT9772_CONTROL_PD, 0 },
	{ GT9772_REG_CONTROL,	0x00, GT9772_T_OPR_US },
	{ GT9772_REG_RING_MODE,	0x88, 0 },
	{ GT9772_REG_RING_TIME,	0x01, GT9772_T_OPR_US },
	{ GT9772_REG_VENDOR_08,	0x4a, 0 },
};

static inline struct gt9772 *sd_to_gt9772(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct gt9772, sd);
}

static int gt9772_set_pos(struct gt9772 *gt9772, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772->sd);

	return i2c_smbus_write_word_swapped(client, GT9772_REG_POS_MSB, val);
}

static int gt9772_move(struct gt9772 *gt9772, int from, int to)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772->sd);
	int step = to > from ? GT9772_MOVE_STEPS : -GT9772_MOVE_STEPS;
	int val = from;
	int ret;

	while (val != to) {
		val = abs(to - val) > GT9772_MOVE_STEPS ? val + step : to;

		ret = gt9772_set_pos(gt9772, val);
		if (ret) {
			dev_err(&client->dev, "failed to set position: %d\n",
				ret);
			return ret;
		}

		usleep_range(GT9772_MOVE_DELAY_US, GT9772_MOVE_DELAY_US + 500);
	}

	return 0;
}

static int gt9772_init(struct gt9772 *gt9772)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(gt9772_init_regs); i++) {
		ret = i2c_smbus_write_byte_data(client, gt9772_init_regs[i].reg,
						gt9772_init_regs[i].val);
		if (ret < 0) {
			dev_err(&client->dev, "init write 0x%02x failed: %d\n",
				gt9772_init_regs[i].reg, ret);
			return ret;
		}

		if (gt9772_init_regs[i].delay_us)
			usleep_range(gt9772_init_regs[i].delay_us,
				     gt9772_init_regs[i].delay_us + 100);
	}

	return gt9772_move(gt9772, 0, gt9772->focus->val);
}

static int gt9772_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gt9772 *gt9772 = sd_to_gt9772(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Let the lens down gently, then power down */
	gt9772_move(gt9772, gt9772->focus->val, 0);
	i2c_smbus_write_byte_data(client, GT9772_REG_CONTROL,
				  GT9772_CONTROL_PD);
	usleep_range(GT9772_T_OPR_US, GT9772_T_OPR_US + 100);

	regulator_disable(gt9772->vdd);

	return 0;
}

static int gt9772_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gt9772 *gt9772 = sd_to_gt9772(sd);
	int ret;

	ret = regulator_enable(gt9772->vdd);
	if (ret < 0) {
		dev_err(dev, "failed to enable the supply: %d\n", ret);
		return ret;
	}

	usleep_range(GT9772_T_POWER_US, GT9772_T_POWER_US + 500);

	ret = gt9772_init(gt9772);
	if (ret < 0) {
		regulator_disable(gt9772->vdd);
		return ret;
	}

	return 0;
}

static int gt9772_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gt9772 *gt9772 = container_of(ctrl->handler, struct gt9772,
					     ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772->sd);
	int ret = 0;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return 0;

	/* The position is applied by the resume handler otherwise */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	ret = gt9772_set_pos(gt9772, ctrl->val);

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gt9772_ctrl_ops = {
	.s_ctrl = gt9772_set_ctrl,
};

static int gt9772_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int gt9772_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops gt9772_int_ops = {
	.open = gt9772_open,
	.close = gt9772_close,
};

static const struct v4l2_subdev_ops gt9772_ops = { };

static int gt9772_init_controls(struct gt9772 *gt9772)
{
	struct v4l2_ctrl_handler *hdl = &gt9772->ctrls;

	v4l2_ctrl_handler_init(hdl, 1);

	gt9772->focus = v4l2_ctrl_new_std(hdl, &gt9772_ctrl_ops,
					  V4L2_CID_FOCUS_ABSOLUTE, 0,
					  GT9772_MAX_FOCUS_POS,
					  GT9772_FOCUS_STEPS, 0);
	if (hdl->error)
		return hdl->error;

	gt9772->sd.ctrl_handler = hdl;

	return 0;
}

static int gt9772_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gt9772 *gt9772;
	int ret;

	gt9772 = devm_kzalloc(dev, sizeof(*gt9772), GFP_KERNEL);
	if (!gt9772)
		return -ENOMEM;

	gt9772->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(gt9772->vdd))
		return dev_err_probe(dev, PTR_ERR(gt9772->vdd),
				     "failed to get the supply\n");

	v4l2_i2c_subdev_init(&gt9772->sd, client, &gt9772_ops);
	gt9772->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gt9772->sd.internal_ops = &gt9772_int_ops;

	ret = gt9772_init_controls(gt9772);
	if (ret)
		goto err_free_handler;

	ret = media_entity_pads_init(&gt9772->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_handler;

	gt9772->sd.entity.function = MEDIA_ENT_F_LENS;

	/*
	 * Power the part once to see that it answers, the way the first
	 * open() will. The device is left suspended afterwards.
	 */
	ret = gt9772_runtime_resume(dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to power up and initialise\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&gt9772->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register the subdevice: %d\n", ret);
		goto err_power_off;
	}

	pm_runtime_idle(dev);

	return 0;

err_power_off:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	gt9772_runtime_suspend(dev);
err_clean_entity:
	media_entity_cleanup(&gt9772->sd.entity);
err_free_handler:
	v4l2_ctrl_handler_free(&gt9772->ctrls);

	return ret;
}

static void gt9772_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt9772 *gt9772 = sd_to_gt9772(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&gt9772->sd);
	v4l2_ctrl_handler_free(&gt9772->ctrls);
	media_entity_cleanup(&gt9772->sd.entity);
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		gt9772_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id gt9772_of_table[] = {
	{ .compatible = "giantec,gt9772" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gt9772_of_table);

static DEFINE_RUNTIME_DEV_PM_OPS(gt9772_pm_ops, gt9772_runtime_suspend,
				 gt9772_runtime_resume, NULL);

static struct i2c_driver gt9772_i2c_driver = {
	.driver = {
		.name = "gt9772",
		.pm = pm_ptr(&gt9772_pm_ops),
		.of_match_table = gt9772_of_table,
	},
	.probe = gt9772_probe,
	.remove = gt9772_remove,
};
module_i2c_driver(gt9772_i2c_driver);

MODULE_DESCRIPTION("Giantec GT9772 VCM driver");
MODULE_LICENSE("GPL");
