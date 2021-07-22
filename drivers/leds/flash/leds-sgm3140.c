// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Luca Weiss <luca@z3ntu.xyz>

#include <linux/gpio/consumer.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/delay.h>

#include <media/v4l2-flash-led-class.h>

#define FLASH_TIMEOUT_DEFAULT		250000U /* 250ms */
#define FLASH_MAX_TIMEOUT_DEFAULT	300000U /* 300ms */

enum chip_id {
	SGM3140,
	SGM3785,
};

struct sgm3140 {
	struct led_classdev_flash fled_cdev;
	struct v4l2_flash *v4l2_flash;

	struct timer_list powerdown_timer;

	struct gpio_desc *flash_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *vin_regulator;

	struct pwm_device *enable_pwm;
	struct pwm_state pwmstate;

	enum chip_id chip_id;

	unsigned int brightness;

	/* current timeout in us */
	u32 timeout;
	/* maximum timeout in us */
	u32 max_timeout;
};

static struct sgm3140 *flcdev_to_sgm3140(struct led_classdev_flash *flcdev)
{
	return container_of(flcdev, struct sgm3140, fled_cdev);
}

static int sgm3140_set_enable(struct sgm3140 *priv, unsigned int brightness)
{
	unsigned long long duty;
	int ret;

	switch (priv->chip_id) {
	case SGM3140:
		gpiod_set_value_cansleep(priv->enable_gpio, !!brightness);
		return 0;

	case SGM3785:
		duty = priv->pwmstate.period;
		duty *= brightness;
		do_div(duty, LED_FULL);
		priv->pwmstate.enabled = duty > 0;
		if (brightness != LED_OFF && brightness != LED_FULL && !priv->brightness) {
			priv->pwmstate.duty_cycle = priv->pwmstate.period;
			ret = pwm_apply_state(priv->enable_pwm, &priv->pwmstate);
			if (ret)
				return ret;
			usleep_range(5000, 6000);
		}
		priv->pwmstate.duty_cycle = duty;
		return pwm_apply_state(priv->enable_pwm, &priv->pwmstate);
	}

	return -EINVAL;
}

static int sgm3140_brightness_set(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);
	int ret;

	if (priv->brightness == brightness)
		return 0;

	sgm3140_set_enable(priv, brightness);

	if (brightness && !priv->brightness)
		ret = regulator_enable(priv->vin_regulator);
	else if (!brightness && priv->brightness)
		ret = regulator_disable(priv->vin_regulator);

	if (ret) {
		dev_err(led_cdev->dev, "failed to %s regulator: %d\n",
			(brightness ? "enable" : "disable"), ret);
		return ret;
	}

	priv->brightness = brightness;

	return 0;
}

static int sgm3140_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);
	int ret;

	//FIXME it ignores strobe if torch mode was enabled?
	if (!!priv->brightness == state)
		return 0;

	ret = sgm3140_brightness_set(&fled_cdev->led_cdev, LED_FULL * state);
	if (ret)
		return ret;

	if (state) {
		gpiod_set_value_cansleep(priv->flash_gpio, 1);
		mod_timer(&priv->powerdown_timer,
			  jiffies + usecs_to_jiffies(priv->timeout));
	} else {
		del_timer_sync(&priv->powerdown_timer);
		gpiod_set_value_cansleep(priv->flash_gpio, 0);
	}

	priv->brightness = state;

	return 0;
}

static int sgm3140_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);

	*state = timer_pending(&priv->powerdown_timer);

	return 0;
}

static int sgm3140_timeout_set(struct led_classdev_flash *fled_cdev,
			       u32 timeout)
{
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);

	priv->timeout = timeout;

	return 0;
}

static const struct led_flash_ops sgm3140_flash_ops = {
	.strobe_set = sgm3140_strobe_set,
	.strobe_get = sgm3140_strobe_get,
	.timeout_set = sgm3140_timeout_set,
};

static void sgm3140_powerdown_timer(struct timer_list *t)
{
	struct sgm3140 *priv = from_timer(priv, t, powerdown_timer);

	gpiod_set_value(priv->flash_gpio, 0);
	sgm3140_brightness_set(&priv->fled_cdev.led_cdev, LED_OFF);
}

static void sgm3140_init_flash_timeout(struct sgm3140 *priv)
{
	struct led_classdev_flash *fled_cdev = &priv->fled_cdev;
	struct led_flash_setting *s;

	/* Init flash timeout setting */
	s = &fled_cdev->timeout;
	s->min = 1;
	s->max = priv->max_timeout;
	s->step = 1;
	s->val = FLASH_TIMEOUT_DEFAULT;
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
static void sgm3140_init_v4l2_flash_config(struct sgm3140 *priv,
					struct v4l2_flash_config *v4l2_sd_cfg)
{
	struct led_classdev *led_cdev = &priv->fled_cdev.led_cdev;
	struct led_flash_setting *s;

	strscpy(v4l2_sd_cfg->dev_name, led_cdev->dev->kobj.name,
		sizeof(v4l2_sd_cfg->dev_name));

	/* Init flash intensity setting */
	s = &v4l2_sd_cfg->intensity;
	s->min = 0;
	s->max = 1;
	s->step = 1;
	s->val = 1;
}

#else
static void sgm3140_init_v4l2_flash_config(struct sgm3140 *priv,
					struct v4l2_flash_config *v4l2_sd_cfg)
{
}
#endif

static int sgm3140_probe_dt(struct platform_device *pdev, struct sgm3140 *priv)
{
	int ret;

	priv->flash_gpio = devm_gpiod_get(&pdev->dev, "flash", GPIOD_OUT_LOW);
	ret = PTR_ERR_OR_ZERO(priv->flash_gpio);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request flash gpio\n");

	priv->enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	ret = PTR_ERR_OR_ZERO(priv->enable_gpio);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request enable gpio\n");

	priv->vin_regulator = devm_regulator_get(&pdev->dev, "vin");
	ret = PTR_ERR_OR_ZERO(priv->vin_regulator);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request regulator\n");

	return 0;
}

static int sgm3785_probe_dt(struct platform_device *pdev, struct sgm3140 *priv)
{
	int ret;

	priv->flash_gpio = devm_gpiod_get(&pdev->dev, "flash", GPIOD_OUT_LOW);
	ret = PTR_ERR_OR_ZERO(priv->flash_gpio);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request flash gpio\n");

	priv->enable_pwm = devm_pwm_get(&pdev->dev, NULL);
	ret = PTR_ERR_OR_ZERO(priv->enable_pwm);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request pwm signal\n");

	priv->vin_regulator = devm_regulator_get(&pdev->dev, "vin");
	ret = PTR_ERR_OR_ZERO(priv->vin_regulator);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request regulator\n");

	return 0;
}

static int sgm3140_probe(struct platform_device *pdev)
{
	struct sgm3140 *priv;
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
	struct led_init_data init_data = {};
	struct fwnode_handle *child_node;
	struct v4l2_flash_config v4l2_sd_cfg = {};
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip_id = (enum chip_id)device_get_match_data(&pdev->dev);

	switch (priv->chip_id) {
	case SGM3140:
		ret = sgm3140_probe_dt(pdev, priv);
		dev_err(&pdev->dev, "==== 3140");
		break;
	case SGM3785:
		ret = sgm3785_probe_dt(pdev, priv);
		dev_err(&pdev->dev, "==== 3785");
		break;
	}
	if (ret)
		return ret;

	child_node = fwnode_get_next_available_child_node(pdev->dev.fwnode,
							  NULL);
	if (!child_node) {
		dev_err(&pdev->dev,
			"No fwnode child node found for connected LED.\n");
		return -EINVAL;
	}

	ret = fwnode_property_read_u32(child_node, "flash-max-timeout-us",
				       &priv->max_timeout);
	if (ret) {
		priv->max_timeout = FLASH_MAX_TIMEOUT_DEFAULT;
		dev_warn(&pdev->dev,
			 "flash-max-timeout-us property missing\n");
	}

	/*
	 * Set default timeout to FLASH_DEFAULT_TIMEOUT except if max_timeout
	 * from DT is lower.
	 */
	priv->timeout = min(priv->max_timeout, FLASH_TIMEOUT_DEFAULT);

	timer_setup(&priv->powerdown_timer, sgm3140_powerdown_timer, 0);

	fled_cdev = &priv->fled_cdev;
	led_cdev = &fled_cdev->led_cdev;

	fled_cdev->ops = &sgm3140_flash_ops;

	led_cdev->brightness_set_blocking = sgm3140_brightness_set;
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	switch (priv->chip_id) {
	case SGM3140:
		led_cdev->max_brightness = LED_ON;
		break;
	case SGM3785:
		led_cdev->max_brightness = LED_FULL;
		pwm_init_state(priv->enable_pwm, &priv->pwmstate);
		break;
	}

	sgm3140_init_flash_timeout(priv);

	init_data.fwnode = child_node;

	platform_set_drvdata(pdev, priv);

	/* Register in the LED subsystem */
	ret = devm_led_classdev_flash_register_ext(&pdev->dev,
						   fled_cdev, &init_data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register flash device: %d\n",
			ret);
		goto err;
	}

	sgm3140_init_v4l2_flash_config(priv, &v4l2_sd_cfg);

	/* Create V4L2 Flash subdev */
	priv->v4l2_flash = v4l2_flash_init(&pdev->dev,
					   child_node,
					   fled_cdev, NULL,
					   &v4l2_sd_cfg);
	if (IS_ERR(priv->v4l2_flash)) {
		ret = PTR_ERR(priv->v4l2_flash);
		goto err;
	}

	return ret;

err:
	fwnode_handle_put(child_node);
	return ret;
}

static int sgm3140_remove(struct platform_device *pdev)
{
	struct sgm3140 *priv = platform_get_drvdata(pdev);

	del_timer_sync(&priv->powerdown_timer);

	v4l2_flash_release(priv->v4l2_flash);

	return 0;
}

static const struct of_device_id sgm3140_dt_match[] = {
	{ .compatible = "ocs,ocp8110", .data = (void *)SGM3140 },
	{ .compatible = "richtek,rt5033-led", .data = (void *)SGM3140 },
	{ .compatible = "sgmicro,sgm3140", .data = (void *)SGM3140 },
	{ .compatible = "sgmicro,sgm3785", .data = (void *)SGM3785 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sgm3140_dt_match);

static struct platform_driver sgm3140_driver = {
	.probe	= sgm3140_probe,
	.remove	= sgm3140_remove,
	.driver	= {
		.name	= "sgm3140",
		.of_match_table = sgm3140_dt_match,
	},
};

module_platform_driver(sgm3140_driver);

MODULE_AUTHOR("Luca Weiss <luca@z3ntu.xyz>");
MODULE_DESCRIPTION("SG Micro SGM3140 charge pump LED driver");
MODULE_LICENSE("GPL v2");
