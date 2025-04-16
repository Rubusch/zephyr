/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adxl345

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "adxl345.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ADXL345, CONFIG_SENSOR_LOG_LEVEL);

// TODO instead of "pad" use an array based on enum types for gpio numbers
static int adxl345_set_int_pad_state(const struct device *dev, uint8_t pad,
				     bool en)
{
	const struct adxl345_dev_config *cfg = dev->config;
	int state = en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

LOG_INF("INTERRUPT INT_%d -> %s", pad, en ? "ON" : "OFF"); // TODO rm

	/* in case of neither INT_1 nor INT_2 being defined */
	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		LOG_INF("neither INT1, nor INT2 have port - no interrupts defined!");
		return -ENOTSUP;
	}

	if (pad == 1) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int1, state);
		LOG_INF("INT_1 %s", en ? "enabled" : "disabled");
	} else if (pad == 2) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int2, state);
		LOG_INF("INT_2 %s", en ? "enabled" : "disabled");
	} else {
		LOG_INF("no pad to configure (-1)");
		/* pad may be -1, e.g. if no INT line defined in DT */
		return -EINVAL;
	}
}

int adxl345_set_gpios_en(const struct device *dev, bool en)
{
	const struct adxl345_dev_config *cfg = dev->config;

	return adxl345_set_int_pad_state(dev, cfg->drdy_pad, en);
}

// // TODO fix ctx [st] for [analog]
// static int adxl345_enable_int(const struct device *dev,
// 			      const struct sensor_trigger *trig,
// 			      int enable)
// {
// 	const struct adxl345_dev_config *cfg = dev->config;
// // TODO ctx
// 
// 	switch (trig->type) {
// 	case SENSOR_TRIG_DATA_READY:
// 		if (cfg->drdy_pad == 1) {
// 			/* route DRDY to PAD1 */
// 			if (adxl345_pin_int1_route_set(ctx, ADXL345_PAD1_DRDY) != 0) {
// 				return -EIO;
// 			}
// 		} else if (cfg->drdy_pad == 2) {
// 			if  (adxl345_pin_int2_route_set(ctx, ADXL345_PAD2_DRDY) != 0) {
// 				return -EIO;
// 			}
// 		} else {
// 			LOG_ERR("Failed, no interrupt pin configured for data ready (DRDY) in DT");
// 			return -ENOTSUP;
// 		}
// 		return adxl345_set_int_pad_state(dev, cfg->drdy_pad, enable);
// 	default:
// 		LOG_ERR("Failed, unsupported trigger interrupt route %d", trig->type);
// 		return -ENOTSUP;
// 	}
// 
// 	return 0;
// }

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD) || defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)

/** adxl345_handle_interrupt - Interrupt service routine for the sensor.
 * @dev: The device instance.
 * Handle and reset the sensor interrupt events.
 */
static void adxl345_handle_interrupt(const struct device *dev)
{
	struct adxl345_dev_data *drv_data = dev->data;
	const struct adxl345_dev_config *cfg = dev->config;
	uint8_t status;
	int rc;

LOG_INF("TRIGGER: interrupt caught!!!"); // TODO rm

	/* clear the status */
	rc = adxl345_get_status(dev, &status);
	__ASSERT(rc == 0, "Interrupt configuration failed");

	/* handle FIFO: data ready */
	if (FIELD_GET(ADXL345_INT_DATA_RDY, status)) {
		if (drv_data->drdy_handler) {
			drv_data->drdy_handler(dev, drv_data->drdy_trigger);
		}

		rc = adxl345_set_int_pad_state(dev, cfg->drdy_pad, true);
		__ASSERT(rc == 0, "Interrupt configuration failed");
	}

	/* handle FIFO: watermark */
	if (FIELD_GET(ADXL345_INT_WATERMARK, status)) {
		if (drv_data->wm_handler) {
			drv_data->wm_handler(dev, drv_data->wm_trigger);
		}
	}

	/* handle FIFO: overrun */
	if (FIELD_GET(ADXL345_INT_OVERRUN, status)) {
		if (drv_data->overrun_handler) {
			drv_data->overrun_handler(dev, drv_data->overrun_trigger);
		}

		rc = adxl345_set_int_pad_state(dev, cfg->drdy_pad, true);
		__ASSERT(rc == 0, "Interrupt configuration failed");
	}

	/*
	 * The sensor won't generate new interrupts if watermark, data ready
	 * and/or overrun is still set. To unset the status flags, either
	 * consume FIFO content in a provided handler, or fallback to this
	 * reset here.
	 */
	adxl345_reset_events(dev);
}
#endif

static void adxl345_int1_gpio_callback(const struct device *dev,
				       struct gpio_callback *cb,
				       uint32_t pins)
{
	struct adxl345_dev_data *drv_data =
		CONTAINER_OF(cb, struct adxl345_dev_data, int1_cb);

	ARG_UNUSED(pins);

	adxl345_set_int_pad_state(dev, 1, false);

#ifdef CONFIG_ADXL345_STREAM
	LOG_INF("STREAM: call adxl345_stream_irq_handler()"); // TODO rm
	adxl345_stream_irq_handler(dev);
#endif /* CONFIG_ADXL345_STREAM */

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void adxl345_int2_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct adxl345_dev_data *drv_data =
		CONTAINER_OF(cb, struct adxl345_dev_data, int2_cb);

	ARG_UNUSED(pins);

	adxl345_set_int_pad_state(dev, 2, false);

#ifdef CONFIG_ADXL345_STREAM
	LOG_INF("STREAM: call adxl345_stream_irq_handler()"); // TODO rm
	adxl345_stream_irq_handler(dev);
#endif /* CONFIG_ADXL345_STREAM */

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
static void adxl345_thread(struct adxl345_dev_data *drv_data)
{
	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
// TODO disable TRIGGER ISR if STREAM ISR is enabled
		adxl345_handle_interrupt(drv_data->dev);
	}
}

#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
static void adxl345_work_cb(struct k_work *work)
{
	struct adxl345_dev_data *drv_data =
		CONTAINER_OF(work, struct adxl345_dev_data, work);

LOG_INF("called"); // TODO rm

#if !defined CONFIG_ADXL345_STREAM
// TODO verify this is working
	/*
	 * Make sure, STREAM ISR w/ RTIO is handling the interrupt, and not
	 * cleaned up afterwards by the TRIGGER handler, if STREAM is enabled.
	 * So, disable TRIGGER ISR if STREAM is defined.
	 */
	adxl345_handle_interrupt(drv_data->dev);
#endif /* !defined CONFIG_ADXL345_STREAM */

}
#endif

/**
 * adxl345_trigger_set - Register a handler for a sensor trigger from the app.
 * @dev: The device instance.
 * @trig: The interrupt event type of the sensor.
 * @handler: A handler for the sensor event to be registered.
 * Map sensor events to the interrupt lines.
 * return: 0 for success, or error.
 */
int adxl345_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	int rc;

LOG_INF("called"); // TODO rm

	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		/* might be in FIFO BYPASS mode */
		goto done;
	}

	/* generally turn off interrupts */
	rc = adxl345_set_measure_en(dev, false);
	if (rc) {
		return rc;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = trig;
		rc = adxl345_reg_update_bits(dev, ADXL345_REG_INT_ENABLE,
					      ADXL345_INT_DATA_RDY,
					      0xff);
		if (rc) {
			return rc;
		}

		break;
	case SENSOR_TRIG_FIFO_WATERMARK:
		drv_data->wm_handler = handler;
		drv_data->wm_trigger = trig;
		rc = adxl345_reg_update_bits(dev, ADXL345_REG_INT_ENABLE,
					      ADXL345_INT_WATERMARK,
					      0xff);
		if (rc) {
			return rc;
		}
		break;
	case SENSOR_TRIG_FIFO_FULL:
		drv_data->overrun_handler = handler;
		drv_data->overrun_trigger = trig;
		rc = adxl345_reg_update_bits(dev, ADXL345_REG_INT_ENABLE,
					      ADXL345_INT_OVERRUN,
					      0xff);
		if (rc) {
			return rc;
		}
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

	rc = adxl345_set_gpios_en(dev, true);
	if (rc) {
		return rc;
	}

	debug_regs(dev);

done:
	/* clear status and fifo status, enable measurement */
	adxl345_reset_events(dev);

	return adxl345_set_measure_en(dev, handler != NULL);
}

int adxl345_init_interrupt(const struct device *dev)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	int ret;

LOG_INF("called"); // TODO rm

	/* TRIGGER is set, but not INT line was defined in DT */
	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		LOG_WRN("no interrupt gpios configured"); // TODO rm
		return -ENOTSUP;
	}

	if (cfg->gpio_int1.port) {
		if (!gpio_is_ready_dt(&cfg->gpio_int1)) {
			LOG_ERR("INT_1 line defined, but not ready");
			return -ENODEV;
		}
	}

	if (cfg->gpio_int2.port) {
		if (!gpio_is_ready_dt(&cfg->gpio_int2)) {
			LOG_ERR("INT_2 line defined, but not ready");
			return -ENODEV;
		}
	}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	LOG_INF("CONFIG_ADXL345_TRIGGER_OWN_THREAD set"); // TODO rm
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ADXL345_THREAD_STACK_SIZE,
			adxl345_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_ADXL345_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	LOG_INF("CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD set"); // TODO rm
	drv_data->work.handler = adxl345_work_cb;
#endif

	if (cfg->gpio_int1.port) {
		LOG_INF("cfg->gpio_int1.port is set"); // TODO rm
		ret = gpio_pin_configure_dt(&cfg->gpio_int1, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
		gpio_init_callback(&drv_data->int1_cb,
				   adxl345_int1_gpio_callback,
				   BIT(cfg->gpio_int1.pin));
		if (gpio_add_callback(cfg->gpio_int1.port, &drv_data->int1_cb) < 0) {
			LOG_ERR("Failed to set INT_1 gpio callback!");
			return -EIO;
		}
	}

	if (cfg->gpio_int2.port) {
		LOG_INF("cfg->gpio_int2.port is set"); // TODO rm
		ret = gpio_pin_configure_dt(&cfg->gpio_int2, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
		gpio_init_callback(&drv_data->int2_cb,
				   adxl345_int2_gpio_callback,
				   BIT(cfg->gpio_int2.pin));
		if (gpio_add_callback(cfg->gpio_int2.port, &drv_data->int2_cb) < 0) {
			LOG_ERR("Failed to set INT_2 gpio callback!");
			return -EIO;
		}
	}

// TODO verify: dev is needed for threads, verify dev is already initialized when threads start
	drv_data->dev = dev;

	LOG_INF("done"); // TODO rm
	return 0;
}
