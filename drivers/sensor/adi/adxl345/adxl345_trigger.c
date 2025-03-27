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

static int adxl345_set_int_pad_state(const struct device *dev,
				     uint8_t pad,
				     bool enable)
{
	const struct adxl345_dev_config *cfg = dev->config;
	int state = enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

	if (pad == 1) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int1, state);
	} else if (pad == 2) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int2, state);
	} else {
		return -EINVAL;
	}
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
static void adxl345_handle_interrupt(const struct device *dev)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	uint8_t status;
	int ret;

	/* Clear the status */
// TODO  fetch status(int) and status2(fifo)
// TODO clear fifo status as well
//	if (adxl345_get_status(dev, &status, NULL) < 0) {
	if (adxl345_get_status(dev, &status) < 0) {
		return;
	}

	if ((drv_data->drdy_handler != NULL) &&
		ADXL345_STATUS_DATA_RDY(status)) {
		drv_data->drdy_handler(dev, drv_data->drdy_trigger);
	}

	ret = adxl345_set_int_pad_state(drv_data->dev, cfg->drdy_pad, true);
//	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, //  TODO rm
//					      GPIO_INT_EDGE_TO_ACTIVE); // TODO rm
	__ASSERT(ret == 0, "Interrupt configuration failed");
}
#endif

static void adxl345_int1_gpio_callback(const struct device *dev,
				       struct gpio_callback *cb,
				       uint32_t pins)
{
	struct adxl345_dev_data *drv_data =
		CONTAINER_OF(cb, struct adxl345_dev_data, int1_cb);
//	const struct adxl345_dev_config *cfg = drv_data->dev->config; // TODO rm
//	gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_DISABLE); // TODO rm

	ARG_UNUSED(pins);

	adxl345_set_int_pad_state(drv_data->dev, 1, true);

//	if (IS_ENABLED(CONFIG_ADXL345_STREAM)) { // TODO rm
//		adxl345_stream_irq_handler(drv_data->dev);
//	} // TODO rm

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

	adxl345_set_int_pad_state(drv_dta->dev, 2, true);

//	const struct adxl345_dev_config *cfg = drv_data->dev->config; // TODO rm
//	gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_DISABLE); // TODO rm

//	if (IS_ENABLED(CONFIG_ADXL345_STREAM)) { // TODO rm
//		adxl345_stream_irq_handler(drv_data->dev);
//	} // TODO rm

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
static void adxl345_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct adxl345_dev_data *drv_data = p1;

	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		adxl345_handle_interrupt(drv_data->dev);
	}
}

#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
static void adxl345_work_cb(struct k_work *work)
{
	struct adxl345_dev_data *drv_data =
		CONTAINER_OF(work, struct adxl345_dev_data, work);

	adxl345_handle_interrupt(drv_data->dev);
}
#endif

/**
 * adxl345_trigger_set - link the data ready event
 */
int adxl345_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	uint8_t int_mask, int_en, status;
	int enable = (handler != NULL) ? PROPERTY_ENABLE : PROPERTY_DISABLE;
	int ret;

	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		return -ENOTSUP;
	}

// TODO rm
//	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_DISABLE);
//	if (ret < 0) {
//		return ret;
//	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = trig;
		int_mask = ADXL345_INT_MAP_DATA_RDY_MSK;
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}
	
	/* map requested interrupt events for this interrupt gpio line */
	if (handler) {
		int_en = int_mask;
	} else {
		int_en = 0U;
	}
// TODO int_en -> active high, then invert depending on intnumber of data_rdy gpio int
	ret = adxl345_reg_write_mask(dev, ADXL345_INT_MAP, int_mask, int_en);
	if (ret < 0) {
		return ret;
	}

	/* Clear status */
// TODO move to init: clear int status
// TODO also clear fifo status
//	ret = adxl345_get_status(dev, &status, NULL);
	ret = adxl345_get_status(dev, &status);
	if (ret < 0) {
		return ret;
	}

	ret = adxl345_set_int_pad_state(drv_data->dev, cfg->threshold_pad, enable); // TODO
//	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
//					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int adxl345_init_interrupt(const struct device *dev)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	int ret;

// TODO in case use return value: <0: fail, 0: bypassed, 1/2: int1 or int2 line?
// TODO cfg->interrupt replaced by cfg->gpio_int1 and cfg->gpio_int2

	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		return -ENOTSUP;
	}

	if (cfg->gpio_int1.port) {
		if (!gpio_is_ready_dt(&cfg->gpio_int1)) {
			LOG_ERR("INT_1 line is not ready");
			return -ENODEV;
		}
	}

	if (cfg->gpio_int2.port) {
		if (!gpio_is_ready_dt(&cfg->gpio_int2)) {
			LOG_ERR("INT_2 line is not ready");
			return -ENODEV;
		}
	}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ADXL345_THREAD_STACK_SIZE,
			adxl345_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_ADXL345_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = adxl345_work_cb;
#endif

	if (cfg->gpio_int1.port) {
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

//	drv_data->dev = dev; // TODO rm, needed?

// TODO check? adxl345_int1_notification_set(ctx, ADXL345_INT1_PULSED) != 0
// TODO check? adxl345_int1_notification_set(ctx, ADXL345_INT2_PULSED) != 0

	return 0;
}
