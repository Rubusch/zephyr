/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_
#define ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/gpio.h>

#ifdef CONFIG_ADXL345_STREAM
#include <zephyr/rtio/rtio.h>
#endif /* CONFIG_ADXL345_STREAM */

#define DT_DRV_COMPAT adi_adxl345

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif
#include <zephyr/sys/util.h>

/* ADXL345 communication commands */
#define ADXL345_WRITE_CMD		0x00
#define ADXL345_READ_CMD		0x80
#define ADXL345_MULTIBYTE_FLAG		0x40

#define ADXL345_REG_READ(x)	(FIELD_GET(GENMASK(5, 0), x) | ADXL345_READ_CMD)

#define ADXL345_COMPLEMENT_MASK(x)	GENMASK(15, (x))
#define ADXL345_COMPLEMENT		GENMASK(15, 9)

/* Registers */
#define ADXL345_REG_DEVICE_ID		0x00
#define ADXL345_REG_RATE		0x2c
#define ADXL345_REG_POWER_CTL		0x2d
#define ADXL345_REG_INT_ENABLE		0x2e
#define ADXL345_REG_INT_MAP		0x2f
#define ADXL345_REG_INT_SOURCE		0x30
#define ADXL345_REG_DATA_FORMAT		0x31
#define ADXL345_REG_DATA_XYZ_REGS	0x32
#define ADXL345_REG_FIFO_CTL		0x38
#define ADXL345_REG_FIFO_STATUS		0x39

/* Regstier fields / content values */
#define ADXL345_PART_ID			0xe5

#define ADXL345_DATA_FORMAT_RIGHT_JUSTIFY BIT(2) /* enable left justified */
#define ADXL345_DATA_FORMAT_FULL_RES	BIT(3)
#define ADXL345_DATA_FORMAT_INT_INVERT	BIT(5) /* enable int active low */
#define ADXL345_DATA_FORMAT_3WIRE_SPI	BIT(6) /* enable 3-wire SPI */
#define ADXL345_DATA_FORMAT_SELF_TEST	BIT(7) /* enable self test */

#define ADXL345_DATA_FORMAT_RANGE_2G	0x00
#define ADXL345_DATA_FORMAT_RANGE_4G	0x01
#define ADXL345_DATA_FORMAT_RANGE_8G	0x02
#define ADXL345_DATA_FORMAT_RANGE_16G	0x03

enum adxl345_range {
	ADXL345_RANGE_2G,
	ADXL345_RANGE_4G,
	ADXL345_RANGE_8G,
	ADXL345_RANGE_16G,
};

static const uint8_t adxl345_range_init[] = {
	[ADXL345_RANGE_2G] = ADXL345_DATA_FORMAT_RANGE_2G,
	[ADXL345_RANGE_4G] = ADXL345_DATA_FORMAT_RANGE_4G,
	[ADXL345_RANGE_8G] = ADXL345_DATA_FORMAT_RANGE_8G,
	[ADXL345_RANGE_16G] = ADXL345_DATA_FORMAT_RANGE_16G,
};

#define ADXL345_RATE_25HZ		0x8

/* ADXL345_INT_MAP */
#define ADXL345_INT_OVERRUN		BIT(0)
#define ADXL345_INT_WATERMARK		BIT(1)
#define ADXL345_INT_FREE_FALL		BIT(2)
#define ADXL345_INT_INACT		BIT(3)
#define ADXL345_INT_ACT			BIT(4)
#define ADXL345_INT_DOUBLE_TAP		BIT(5)
#define ADXL345_INT_SINGLE_TAP		BIT(6)
#define ADXL345_INT_DATA_RDY		BIT(7)

/* POWER_CTL */
#define ADXL345_POWER_CTL_WAKEUP_4HZ	BIT(0)
#define ADXL345_POWER_CTL_WAKEUP_2HZ	BIT(1)
#define ADXL345_POWER_CTL_SLEEP		BIT(2)

#define ADXL345_POWER_CTL_MEASURE_MODE	BIT(3)
#define ADXL345_POWER_CTL_STANDBY_MODE	0x0
#define ADXL345_POWER_CTL_MODE_MSK	BIT(3)

/* ADXL345_FIFO_CTL */
#define ADXL345_FIFO_CTL_MODE_MSK	GENMASK(7, 6)
#define ADXL345_FIFO_CTL_MODE_BYPASSED	0x0
#define ADXL345_FIFO_CTL_MODE_OLD_SAVED	0x40
#define ADXL345_FIFO_CTL_MODE_STREAMED	0x80
#define ADXL345_FIFO_CTL_MODE_TRIGGERED	0xc0

enum adxl345_fifo_mode {
	ADXL345_FIFO_BYPASSED,
	ADXL345_FIFO_OLD_SAVED,
	ADXL345_FIFO_STREAMED,
	ADXL345_FIFO_TRIGGERED,
};

static const uint8_t adxl345_fifo_ctl_mode_init[] = {
	[ADXL345_FIFO_BYPASSED] = ADXL345_FIFO_CTL_MODE_BYPASSED,
	[ADXL345_FIFO_OLD_SAVED] = ADXL345_FIFO_CTL_MODE_OLD_SAVED,
	[ADXL345_FIFO_STREAMED] = ADXL345_FIFO_CTL_MODE_STREAMED,
	[ADXL345_FIFO_TRIGGERED] = ADXL345_FIFO_CTL_MODE_TRIGGERED,
};

#define ADXL345_FIFO_SAMPLE_SIZE	6

#define ADXL345_FIFO_CTL_SAMPLES_MSK	GENMASK(4, 0)
#define ADLX345_FIFO_STATUS_ENTRIES_MSK	GENMASK(5, 0)
#define ADXL345_MAX_FIFO_SIZE		32

#define ADXL345_FIFO_CTL_TRIGGER_INT1	0x0
#define ADXL345_FIFO_CTL_TRIGGER_INT2	BIT(5)
#define ADXL345_FIFO_CTL_TRIGGER_UNSET	0x0

/* FIFO trigger, note this is only used in FIFO triggered mode */
enum adxl345_fifo_trigger {
	ADXL345_INT1,
	ADXL345_INT2,
	ADXL345_INT_UNSET,
};

#define ADXL345_ODR_MSK			GENMASK(3, 0)
#define ADXL345_ODR_MODE(x)		((x) & 0xF)

enum adxl345_odr {
	ADXL345_ODR_12HZ = 0x7,
	ADXL345_ODR_25HZ,
	ADXL345_ODR_50HZ,
	ADXL345_ODR_100HZ,
	ADXL345_ODR_200HZ,
	ADXL345_ODR_400HZ,
};

#define ADXL345_BUS_I2C			0
#define ADXL345_BUS_SPI			1

enum adxl345_op_mode {
	ADXL345_OP_STANDBY,
	ADXL345_OP_MEASURE,
};

struct adxl345_fifo_config {
	enum adxl345_fifo_mode fifo_mode;
	enum adxl345_fifo_trigger fifo_trigger;
	uint8_t fifo_samples; /* number of entries to read for STREAM */
};

struct adxl345_fifo_data {
	uint8_t is_fifo: 1;
	uint8_t is_full_res: 1;
	enum adxl345_range selected_range: 2;
	uint8_t sample_set_size: 4;
	uint8_t int_status;
	uint16_t accel_odr: 4;
	uint16_t fifo_byte_count: 12;
	uint64_t timestamp;
} __attribute__((__packed__));

struct adxl345_xyz_accel_data {
#ifdef CONFIG_ADXL345_STREAM
	uint8_t is_fifo: 1;
	uint8_t res: 7;
#endif /* CONFIG_ADXL345_STREAM */
	enum adxl345_range selected_range;
	bool is_full_res;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__));

union adxl345_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
};

struct adxl345_dev_data {
	unsigned int sample_number;
	int16_t bufx[ADXL345_MAX_FIFO_SIZE];
	int16_t bufy[ADXL345_MAX_FIFO_SIZE];
	int16_t bufz[ADXL345_MAX_FIFO_SIZE];
	uint8_t sample_idx; /* index counting up sample_number entries */
	struct adxl345_fifo_config fifo_config;
	bool is_full_res;
	enum adxl345_range selected_range;
	enum adxl345_odr odr;
#ifdef CONFIG_ADXL345_TRIGGER
	struct gpio_callback int1_cb;
	struct gpio_callback int2_cb;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	const struct device *dev;
#if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL345_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ADXL345_TRIGGER */
#ifdef CONFIG_ADXL345_STREAM
	struct rtio_iodev_sqe *sqe;
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
	uint8_t status1;
	uint8_t fifo_ent[1];
	uint64_t timestamp;
	struct rtio *r_cb;
	uint8_t fifo_samples;
	uint16_t fifo_total_bytes;
#endif /* CONFIG_ADXL345_STREAM */
};

typedef bool (*adxl345_bus_is_ready_fn)(const union adxl345_bus *bus);
typedef int (*adxl345_reg_access_fn)(const struct device *dev, uint8_t cmd,
				     uint8_t reg_addr, uint8_t *data, size_t length);

struct adxl345_dev_config {
	const union adxl345_bus bus;
	adxl345_bus_is_ready_fn bus_is_ready;
	adxl345_reg_access_fn reg_access;
	enum adxl345_odr odr;
	uint8_t bus_type;
#ifdef CONFIG_ADXL345_TRIGGER
	struct gpio_dt_spec gpio_int1;
	struct gpio_dt_spec gpio_int2;
	int8_t drdy_pad;
#endif
};

int adxl345_set_gpios_en(const struct device *dev, bool enable);

void adxl345_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);
void adxl345_stream_irq_handler(const struct device *dev);

int adxl345_set_measure_en(const struct device *dev, bool en);

#ifdef CONFIG_ADXL345_TRIGGER
int adxl345_get_status(const struct device *dev, uint8_t *status);

int adxl345_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int adxl345_init_interrupt(const struct device *dev);

#endif /* CONFIG_ADXL345_TRIGGER */

int adxl345_reg_access(const struct device *dev, uint8_t cmd, uint8_t addr,
		       uint8_t *data, size_t len);

int adxl345_reg_write(const struct device *dev, uint8_t addr, uint8_t *data,
		      uint8_t len);

int adxl345_reg_read(const struct device *dev, uint8_t addr, uint8_t *data,
		     uint8_t len);

int adxl345_reg_write_byte(const struct device *dev, uint8_t addr, uint8_t val);

int adxl345_reg_read_byte(const struct device *dev, uint8_t addr, uint8_t *buf);

int adxl345_reg_update_bits(const struct device *dev, uint8_t reg,
			    uint8_t mask, uint8_t val);

#ifdef CONFIG_SENSOR_ASYNC_API
int adxl345_read_sample(const struct device *dev, struct adxl345_xyz_accel_data *sample);
void adxl345_submit(const struct device *dev,
		    struct rtio_iodev_sqe *iodev_sqe);
int adxl345_get_decoder(const struct device *dev,
			const struct sensor_decoder_api **decoder);
void adxl345_accel_convert(struct sensor_value *val, int16_t sample);
#endif /* CONFIG_SENSOR_ASYNC_API */

#ifdef CONFIG_ADXL345_STREAM
size_t adxl345_get_packet_size(const struct adxl345_dev_config *cfg);
#endif /* CONFIG_ADXL345_STREAM */

#endif /* ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_ */
