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
				     bool enable)
{
	const struct adxl345_dev_config *cfg = dev->config;
	int state = enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

	/* in case trigger mode, but neither INT_1 nor INT_2 defined */
	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		return -ENOTSUP;
	}

	if (pad == 1) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int1, state);
	} else if (pad == 2) {
		return gpio_pin_interrupt_configure_dt(&cfg->gpio_int2, state);
	} else {
		/* pad may be -1, e.g. if no INT line defined in DT */
		return -EINVAL;
	}
}

int adxl345_set_gpios_en(const struct device *dev, bool enable)
{
	const struct adxl345_dev_config *cfg = dev->config;

	return adxl345_set_int_pad_state(dev, cfg->drdy_pad, enable);
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

/** adxl345_handle_interrupt - Interrupt handler for the sensor.
 * @dev: The device instance.
 * Handle and reset the sensor interrupt events.
 */
static void adxl345_handle_interrupt(const struct device *dev)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	uint8_t status;
	int ret;

	/* clear the status */
	if (adxl345_get_status(dev, &status) < 0) {
		return;
	}

// TODO overrun, drdy and watermark on the same gpio INT!!!

	/* clear FIFO status */
	// TODO here or check if handler clears FIFO

	/* handle FIFO: watermark */
	// TODO

	/* handle FIFO: data ready */
	if (FIELD_GET(ADXL345_INT_MAP_DATA_RDY_MSK, status)) {
		if (drv_data->drdy_handler) {
			drv_data->drdy_handler(dev, drv_data->drdy_trigger);
		}

		ret = adxl345_set_int_pad_state(dev, cfg->drdy_pad, true);
		__ASSERT(ret == 0, "Interrupt configuration failed");
	}

	/* handle FIFO: overrun */
	// TODO

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

	if (IS_ENABLED(CONFIG_ADXL345_STREAM)) {
		adxl345_stream_irq_handler(dev); // TODO verify dev is working
	}

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

	if (IS_ENABLED(CONFIG_ADXL345_STREAM)) {
		adxl345_stream_irq_handler(dev);  // TODO verify, dev is working
	}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
//static void adxl345_thread(void *p1, void *p2, void *p3) // TODO rm
static void adxl345_thread(struct adxl345_dev_data *drv_data)
{
//	ARG_UNUSED(p2); // TODO rm
//	ARG_UNUSED(p3);
//	struct adxl345_dev_data *drv_data = p1;
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
 * adxl345_trigger_set - Interrupt line setup.
 * @dev: The device instance.
 * @trig: The sensor_trigger instance.
 * @handler: A sensor_trigger_handler instance or NULL.
 * Map sensor events to the interrupt lines.
 * return: 0 for success, or error.
 */
int adxl345_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	uint8_t int_mask, int_en, status;
	int ret;

	if (!cfg->gpio_int1.port && !cfg->gpio_int2.port) {
		return -ENOTSUP;
	}

	/* generally turn off interrupts */
	ret = adxl345_set_int_pad_state(dev, cfg->drdy_pad, false);
	if (ret) {
		return ret;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = trig;
		/* The ADXL345 supports mapping of each sensor event
		 * individually to particular interrupt lines, i.e. either to
		 * INT_1 (0) or INT_2 (1). Until sensor events are not
		 * implemented and for simplicity all sensor events are
		 * generally mapped either to INT_1 or INT_2.
		 *
		 * NB: if INT_1 and INT_2 are available also depends on the
		 * particular hardware setup.
		 */
		int_mask = 0xff;
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}
	
	/* map requested interrupt events according to selected interrupt line */
	if (handler) {
		int_en = (cfg->drdy_pad == 1) ? int_mask : ~int_mask;
	} else {
		int_en = 0U;
	}

	ret = adxl345_reg_update_bits(dev, ADXL345_REG_INT_MAP, int_mask, int_en);
	if (ret < 0) {
		return ret;
	}

	/* Clear status and fifo status */
	ret = adxl345_get_status(dev, &status);
	if (ret < 0) {
		return ret;
	}

	ret = adxl345_get_fifo_status(dev, &status);
	if (ret < 0) {
		return ret;
	}

	return adxl345_set_int_pad_state(dev, cfg->drdy_pad,
					 (handler != NULL));
}

int adxl345_init_interrupt(const struct device *dev)
{
	const struct adxl345_dev_config *cfg = dev->config;
	struct adxl345_dev_data *drv_data = dev->data;
	int ret;

	/* TRIGGER is set, but not INT line was defined in DT */
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

	return 0;
}
