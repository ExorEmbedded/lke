/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <video/displayconfig.h>
#include <linux/init.h> 

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	bool			enabled;
	int			enable_gpio;
	unsigned long		enable_gpio_flags;
	unsigned int		scale;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
};

/*----------------------------------------------------------------------------------------------------------------*
Exported function which allows to get the actual backlight enable status from kernel.
It is needed for example by the working hours driver to correctly compute the effective backlight on time.
*----------------------------------------------------------------------------------------------------------------*/
bool pwm_backlight_is_enabled(struct backlight_device* bl)
{
  struct pwm_bl_data *pb = bl_get_data(bl);
  return pb->enabled;
}
EXPORT_SYMBOL(pwm_backlight_is_enabled);

/*----------------------------------------------------------------------------------------------------------------*
Helper functions to retrieve the display id value, when passed from the cmdline, and use it to set the
backlight parameters (the contents of the DTB file are overridden, if a valid dispaly id is passed from
cmdline.)
*----------------------------------------------------------------------------------------------------------------*/
extern int hw_dispid; //Exported variable which holds the display id value, when passed from the cmdline

/*
* Writes the pwm_bl_data structure according with the contents of the displayconfig.h file and the passed dispid parameter.
* Returns 0 if success, -1 if failure (ie: no match found)
*/
int dispid_get_backlight(struct pwm_bl_data* pb, int dispid, int maxlevels)
{
  int i=0;
  int j;
  int step = 0;
  
  // Scan the display array to search for the required dispid
  if(dispid == NODISPLAY)
    return -1;
	
  while((displayconfig[i].dispid != NODISPLAY) && (displayconfig[i].dispid != dispid))
    i++;
  
  if(displayconfig[i].dispid == NODISPLAY)
    return -1;
      
  // If we are here, we have a valid array index pointing to the desired display
  // Configure the backlight controller, based on a 0-100% dutycycle range.
  pb->lth_brightness = 0;
  pb->scale = 100;
  
  if(displayconfig[i].pwmfreq == 0)
    displayconfig[i].pwmfreq = 1000;
    
  pb->period = 1000000000l / displayconfig[i].pwmfreq;
  
  //Now override the levels based on linear interpolation between brightness_min and brightness_max
  if(displayconfig[i].brightness_max > 100)
    displayconfig[i].brightness_max = 100;
  
  if((displayconfig[i].brightness_min & 0xff) > displayconfig[i].brightness_max)
    displayconfig[i].brightness_min = displayconfig[i].brightness_max;
    
  if(maxlevels > 1) 
    step = 100 * (displayconfig[i].brightness_max - (displayconfig[i].brightness_min & 0xff)) / (maxlevels-1);
  for(j=1; j<maxlevels; j++)
    pb->levels[j] = (displayconfig[i].brightness_min & 0xff) + (step *(j-1))/100;
  
  pb->levels[1] = displayconfig[i].brightness_min;
  pb->levels[maxlevels] = displayconfig[i].brightness_max;
  
  /* BSP-1559: Create brightness curve for zero dimming feature (3 segments envelope) */
  if((displayconfig[i].brightness_min & 0xff00) && (maxlevels > 16)) 
  {
    int lev;

    /* 1st segment m=brightness_min */
    step = (displayconfig[i].brightness_min >> 8) & 0xff;
    for(j=2; j<7; j++)
    {
      lev = j*step;
      if(lev < 255)
	pb->levels[j] = (lev << 8) & 0xff00;
      else
	pb->levels[j] = lev / 100;
    }
    
    /* 2nd segment m=1 */
    lev = (lev/100) + 1;
    for(j=7; j<10; j++)
      pb->levels[j] = lev++;
    
    lev++;
    pb->levels[j] = lev;
    
    /* 3rd segment: interpolation till brightness_max */
    step = 100 * (displayconfig[i].brightness_max - lev) / (maxlevels-j);
    for(j=11;j<maxlevels; j++)
      pb->levels[j] = lev + (step *(j-10))/100;
    
    pb->levels[maxlevels] = displayconfig[i].brightness_max;
  }
  
  for(j=1; j<(maxlevels+1); j++)
    printk("j=%d lev=%d \n",j,pb->levels[j]);

  return 0;
}

static void pwm_backlight_power_on(struct pwm_bl_data *pb, int brightness)
{
	if (pb->enabled)
		return;

	if (gpio_is_valid(pb->enable_gpio)) {
		if (pb->enable_gpio_flags & PWM_BACKLIGHT_GPIO_ACTIVE_LOW)
			gpio_set_value(pb->enable_gpio, 0);
		else
			gpio_set_value(pb->enable_gpio, 1);
	}

	pwm_enable(pb->pwm);
	pb->enabled = true;
}

static void pwm_backlight_power_off(struct pwm_bl_data *pb)
{
	if (!pb->enabled)
		return;

	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);

	if (gpio_is_valid(pb->enable_gpio)) {
		if (pb->enable_gpio_flags & PWM_BACKLIGHT_GPIO_ACTIVE_LOW)
			gpio_set_value(pb->enable_gpio, 1);
		else
			gpio_set_value(pb->enable_gpio, 0);
	}

	pb->enabled = false;
}

static int compute_duty_cycle(struct pwm_bl_data *pb, int brightness)
{
	unsigned int lth = pb->lth_brightness;
	int duty_cycle;

	if (pb->levels)
		duty_cycle = pb->levels[brightness];
	else
		duty_cycle = brightness;
	
	/* BSP-1559: handle special brightness dimming values */
	if(pb->levels)
	  if(duty_cycle & 0xff00)
	    {
	      duty_cycle = ((duty_cycle >> 8) & 0xff);
	      return ((duty_cycle * pb->period) / (pb->scale * 100));
	    }

	return (duty_cycle * (pb->period - lth) / pb->scale) + lth;
}

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int duty_cycle;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness > 0) {
		duty_cycle = compute_duty_cycle(pb, brightness);
		pwm_config(pb->pwm, duty_cycle, pb->period);
		pwm_backlight_power_on(pb, brightness);
	} else
		pwm_backlight_power_off(pb);

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	struct property *prop;
	int length;
	u32 value;
	int ret;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;

		data->dft_brightness = value;
		data->max_brightness--;
	}

	data->enable_gpio = of_get_named_gpio_flags(node, "enable-gpios", 0,
						    &flags);
	if (data->enable_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(data->enable_gpio) && (flags & OF_GPIO_ACTIVE_LOW))
		data->enable_gpio_flags |= PWM_BACKLIGHT_GPIO_ACTIVE_LOW;

	return 0;
}

static const struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = dev_get_platdata(&pdev->dev);
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		unsigned int i;

		for (i = 0; i <= data->max_brightness; i++)
			if (data->levels[i] > pb->scale)
				pb->scale = data->levels[i];

		pb->levels = data->levels;
	} else
		pb->scale = data->max_brightness;

	pb->enable_gpio = data->enable_gpio;
	pb->enable_gpio_flags = data->enable_gpio_flags;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;
	pb->enabled = false;

	if (gpio_is_valid(pb->enable_gpio)) {
		unsigned long flags;

		if (pb->enable_gpio_flags & PWM_BACKLIGHT_GPIO_ACTIVE_LOW)
			flags = GPIOF_OUT_INIT_HIGH;
		else
			flags = GPIOF_OUT_INIT_LOW;

		ret = gpio_request_one(pb->enable_gpio, flags, "enable");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request GPIO#%d: %d\n",
				pb->enable_gpio, ret);
			goto err_alloc;
		}
	}

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
		if (IS_ERR(pb->pwm)) {
			dev_err(&pdev->dev, "unable to request legacy PWM\n");
			ret = PTR_ERR(pb->pwm);
			goto err_gpio;
		}
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data if it has not already been set
	 * via the PWM lookup table.
	 */
	if (data->pwm_period_ns > 0)
		pwm_set_period(pb->pwm, data->pwm_period_ns);

	pb->period = pwm_get_period(pb->pwm);
	pb->lth_brightness = data->lth_brightness * (pb->period / pb->scale);
	
	dispid_get_backlight(pb, hw_dispid, data->max_brightness);
	
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_gpio;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_gpio:
	if (gpio_is_valid(pb->enable_gpio))
		gpio_free(pb->enable_gpio);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_backlight_power_off(pb);

	if (pb->exit)
		pb->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	if (pb->notify)
		pb->notify(pb->dev, 0);

	pwm_backlight_power_off(pb);

	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);
	return 0;
}
#endif

static const struct dev_pm_ops pwm_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = pwm_backlight_suspend,
	.resume = pwm_backlight_resume,
	.poweroff = pwm_backlight_suspend,
	.restore = pwm_backlight_resume,
#endif
};

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.pm		= &pwm_backlight_pm_ops,
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");
