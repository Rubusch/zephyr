/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_
#define ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_

//#define CONFIG_ADXL345_TRIGGER // TODO rm

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
#define ADXL345_WRITE_CMD			0x00
#define ADXL345_READ_CMD			0x80
#define ADXL345_MULTIBYTE_FLAG			0x40

#define ADXL345_REG_READ(x)			((x & 0xFF) | ADXL345_READ_CMD)

#define ADXL345_COMPLEMENT_MASK(x)		GENMASK(15, (x))
#define ADXL345_COMPLEMENT			GENMASK(15, 9) /* 0xfc00 */

/* Registers */
#define ADXL345_REG_DEVICE_ID			0x00
#define ADXL345_REG_RATE			0x2c
#define ADXL345_REG_POWER_CTL			0x2d
#define ADXL345_REG_INT_ENABLE			0x2e
#define ADXL345_REG_INT_MAP			0x2f
#define ADXL345_REG_INT_SOURCE			0x30
#define ADXL345_REG_DATA_FORMAT			0x31
#define ADXL345_REG_DATAX0			0x32
#define ADXL345_REG_DATA_XYZ_REGS		0x32 // TODO use?
/* DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1 - each 8-bit, output data */
#define ADXL345_REG_FIFO_CTL			0x38
#define ADXL345_REG_FIFO_STATUS			0x39


#define ADXL345_PART_ID				0xe5

#define ADXL345_DATA_FORMAT_RIGHT_JUSTIFY	BIT(2) /* 1: data is left justified on MSB */
#define ADXL345_DATA_FORMAT_FULL_RES		BIT(3)
#define ADXL345_DATA_FORMAT_INT_INVERT		BIT(5) /* 1: set interrupts to active low */
#define ADXL345_DATA_FORMAT_3WIRE_SPI		BIT(6) /* 1: set to 3-wire SPI */
#define ADXL345_DATA_FORMAT_SELF_TEST		BIT(7) /* 1: enable self test */

#define ADXL345_DATA_FORMAT_RANGE_2G		0x00
#define ADXL345_DATA_FORMAT_RANGE_4G		0x01
#define ADXL345_DATA_FORMAT_RANGE_8G		0x02
#define ADXL345_DATA_FORMAT_RANGE_16G		0x03

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

#define ADXL345_RATE_25HZ			0x8

//#define ADXL345_ENABLE_MEASURE_BIT (1 << 3) // TODO used?
//#define ADXL345_FIFO_STREAM_MODE   (1 << 7) // TODO used?

/* ADXL345_STATUS */
//#define ADXL345_STATUS_DOUBLE_TAP(x) (((x) >> 5) & 0x1) // TODO rm, not used
//#define ADXL345_STATUS_SINGLE_TAP(x) (((x) >> 6) & 0x1) // TODO rm, not used
//#define ADXL345_STATUS_DATA_RDY(x)   (((x) >> 7) & 0x1) // TODO rm

/* ADXL345_INT_MAP */
#define ADXL345_INT_MAP_OVERRUN_MSK		BIT(0)
//#define ADXL345_INT_MAP_OVERRUN_MODE(x)    (((x) & 0x1) << 0) // TODO rm, set overrun mode, really??!!!
#define ADXL345_INT_MAP_WATERMARK_MSK		BIT(1)
//#define ADXL345_INT_MAP_WATERMARK_MODE(x)  (((x) & 0x1) << 1) // TODO rm, not used
#define ADXL345_INT_MAP_FREE_FALL_MSK		BIT(2)
//#define ADXL345_INT_MAP_FREE_FALL_MODE(x)  (((x) & 0x1) << 2) // TODO rm, not used
#define ADXL345_INT_MAP_INACT_MSK		BIT(3)
//#define ADXL345_INT_MAP_INACT_MODE(x)      (((x) & 0x1) << 3) // TODO rm, not used
#define ADXL345_INT_MAP_ACT_MSK			BIT(4)
//#define ADXL345_INT_MAP_ACT_MODE(x)        (((x) & 0x1) << 4) // TODO rm, not used
#define ADXL345_INT_MAP_DOUBLE_TAP_MSK		BIT(5)
//#define ADXL345_INT_MAP_DOUBLE_TAP_MODE(x) (((x) & 0x1) << 5) // TODO rm, not used
#define ADXL345_INT_MAP_SINGLE_TAP_MSK		BIT(6)
//#define ADXL345_INT_MAP_SINGLE_TAP_MODE(x) (((x) & 0x1) << 6) // TODO rm, not used
#define ADXL345_INT_MAP_DATA_RDY_MSK		BIT(7)
//#define ADXL345_INT_MAP_DATA_RDY_MODE(x)   (((x) & 0x1) << 7) // TODO rm, not used

/* POWER_CTL */
#define ADXL345_POWER_CTL_WAKEUP_4HZ		BIT(0)
//#define ADXL345_POWER_CTL_WAKEUP_4HZ_MODE(x)	(((x) & 0x1) << 0) // TODO rm
#define ADXL345_POWER_CTL_WAKEUP_2HZ		BIT(1)
//#define ADXL345_POWER_CTL_WAKEUP_2HZ_MODE(x)	(((x) & 0x1) << 1) // TODO rm
#define ADXL345_POWER_CTL_SLEEP			BIT(2)
//#define ADXL345_POWER_CTL_SLEEP_MODE(x)		(((x) & 0x1) << 2)
//#define ADXL345_POWER_CTL_MEASURE_MSK		GENMASK(3, 3)
//#define ADXL345_POWER_CTL_MEASURE_MODE(x)    (((x) & 0x1) << 3)
//#define ADXL345_POWER_CTL_STANDBY_MODE(x)    (((x) & 0x0) << 3)

#define ADXL345_POWER_CTL_MEASURE_MODE		BIT(3)
#define ADXL345_POWER_CTL_STANDBY_MODE		0x0
#define ADXL345_POWER_CTL_MODE_MSK		BIT(3)

/* ADXL345_FIFO_CTL */
#define ADXL345_FIFO_CTL_MODE_MSK		GENMASK(7, 6)
#define ADXL345_FIFO_CTL_MODE_BYPASSED		0x0
#define ADXL345_FIFO_CTL_MODE_OLD_SAVED		0x40
#define ADXL345_FIFO_CTL_MODE_STREAMED		0x80
#define ADXL345_FIFO_CTL_MODE_TRIGGERED		0xc0

enum adxl345_fifo_mode {
	ADXL345_FIFO_BYPASSED,
	ADXL345_FIFO_OLD_SAVED,
	ADXL345_FIFO_STREAMED,
	ADXL345_FIFO_TRIGGERED,
};

#define ADXL345_FIFO_SAMPLE_SIZE		6
//#define ADXL345_FIFO_SAMPLE_MASK 0x3F // TODO rm
//#define ADXL345_FIFO_SAMPLE_NUM  0x1F // TODO rm

#define ADXL345_FIFO_CTL_SAMPLES_MSK		GENMASK(4, 0) // 0x1f
#define ADLX345_FIFO_STATUS_ENTRIES_MSK		GENMASK(5, 0) // 0x3f
#define ADXL345_MAX_FIFO_SIZE			32

//#define ADXL345_FIFO_CTL_MODE_MODE(x)    (((x) & 0x3) << 6)
//#define ADXL345_FIFO_CTL_TRIGGER_MSK		BIT(5)
//#define ADXL345_FIFO_CTL_TRIGGER_MODE(x) (((x) & 0x1) << 5) // TODO rm
#define ADXL345_FIFO_CTL_SAMPLES_MSK		GENMASK(4, 0)
//#define ADXL345_FIFO_CTL_SAMPLES_MODE(x) ((x) & 0x1F)

#define ADXL345_FIFO_CTL_TRIGGER_INT1		0x0
#define ADXL345_FIFO_CTL_TRIGGER_INT2		BIT(5)
#define ADXL345_FIFO_CTL_TRIGGER_UNSET		0x0

/* FIFO trigger, note this is only used in FIFO triggered mode */
enum adxl345_fifo_trigger {
	ADXL345_INT1,
	ADXL345_INT2,
	ADXL345_INT_UNSET,
};

#define ADXL345_ODR_MSK				GENMASK(3, 0)
#define ADXL345_ODR_MODE(x)			((x) & 0xF)

enum adxl345_odr {
	ADXL345_ODR_12HZ = 0x7,
	ADXL345_ODR_25HZ,
	ADXL345_ODR_50HZ,
	ADXL345_ODR_100HZ,
	ADXL345_ODR_200HZ,
	ADXL345_ODR_400HZ,
};

#define ADXL345_BUS_I2C				0
#define ADXL345_BUS_SPI				1

struct adxl345_fifo_config {
	enum adxl345_fifo_mode fifo_mode; /* used for distinguishing to BYPASS */
	enum adxl345_fifo_trigger fifo_trigger; // TODO why shall we store those values at all?
	uint8_t fifo_samples; /* used for STREAM/RTIO */
};

struct adxl345_fifo_data {
	uint8_t is_fifo: 1;
	uint8_t is_full_res: 1; /* used for conversion in decoder, set by STREAM */
	enum adxl345_range selected_range: 2; /* determines shift and conversion in decoder, set by STREAM */
	uint8_t sample_set_size: 4;
	uint8_t int_status;
	uint16_t accel_odr: 4; // TODO 4bit for odr, then uint16_t bitpacked?!
	uint16_t fifo_byte_count: 12; // TODO 12 bit for fifo byte count? -> max 32 entries, each 2 bytes,
				      // 3 axis, makes 192 bytes < 256 bytes -> should fit in 2^8
	uint64_t timestamp;
} __attribute__((__packed__));

BUILD_ASSERT(sizeof(struct adxl345_fifo_data) % 4 == 0,
		"struct adxl345_fifo_data should  be word aligned"); // TODO verify

enum adxl345_op_mode {
	ADXL345_OP_STANDBY,
	ADXL345_OP_MEASURE,
};

struct adxl345_xyz_accel_data { // TODO is this actually needed?
#ifdef CONFIG_ADXL345_STREAM // TODO move to end of struct
	uint8_t is_fifo: 1;
	uint8_t res: 7;
#endif /* CONFIG_ADXL345_STREAM */
	enum adxl345_range selected_range; /* used in decoder_decode, passed w/ sample */
	bool is_full_res; /* used in decoder, passed w/ sample to decode */
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)); // TODO verify, probably needs packed

union adxl345_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
};

struct adxl345_dev_data {
/* // TODO replace this by sample? or replace sample by this?
	int16_t bufx[ADXL345_MAX_FIFO_SIZE];
	int16_t bufy[ADXL345_MAX_FIFO_SIZE];
	int16_t bufz[ADXL345_MAX_FIFO_SIZE];
/*/
	struct adxl345_xyz_accel_data sample[ADXL345_MAX_FIFO_SIZE];
	int sample_number;
	int sample_idx;
// */

 	struct adxl345_fifo_config fifo_config; // TODO rm -> move to config

//	uint8_t fifo_samples;
	bool is_full_res; /* STREAM: to init sample for decoder */
	enum adxl345_range selected_range; /* STREAM: init sample for decoder */
	enum adxl345_odr odr;
#ifdef CONFIG_ADXL345_TRIGGER
	struct gpio_callback int1_cb;
	struct gpio_callback int2_cb;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	const struct device *dev; /* trigger cannot refer directly */
# if defined(CONFIG_ADXL345_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL345_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
# elif defined(CONFIG_ADXL345_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
# endif
#endif /* CONFIG_ADXL345_TRIGGER */
#ifdef CONFIG_ADXL345_STREAM
	struct rtio_iodev_sqe *sqe;
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
	uint8_t status;
	uint8_t fifo_ent[1];
	uint64_t timestamp;
// TODO verify the following
	struct rtio *r_cb;
//	uint8_t fifo_watermark_irq;  // TODO rm?
	uint8_t fifo_samples; // TODO rm, duplicate? in case rename "fifo_samples_current"
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
//	bool op_mode; // TODO rm
//	struct adxl345_fifo_config fifo_config; // TODO rm
	uint8_t bus_type; // TODO rm, verify - what is this good for?
//#ifdef CONFIG_ADXL345_TRIGGER
	struct gpio_dt_spec gpio_int1;
	struct gpio_dt_spec gpio_int2;
	int8_t drdy_pad;
//#endif
};

int adxl345_set_gpios_en(const struct device *dev, bool enable);

void adxl345_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);
void adxl345_stream_irq_handler(const struct device *dev);

int adxl345_set_measure_en(const struct device *dev, bool en);

#ifdef CONFIG_ADXL345_TRIGGER
int adxl345_get_fifo_status(const struct device *dev, uint8_t *fifo_entries); // TODO										         // used?
									      // for
									      // what?
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

//int adxl345_set_op_mode(const struct device *dev, enum adxl345_op_mode op_mode); // TODO rm
#ifdef CONFIG_SENSOR_ASYNC_API
int adxl345_get_accel_data(const struct device *dev, struct adxl345_xyz_accel_data *sample);
void adxl345_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);
int adxl345_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);
void adxl345_accel_convert(struct sensor_value *val, int16_t sample); // TODO rm
#endif /* CONFIG_SENSOR_ASYNC_API */

//#ifdef CONFIG_ADXL345_STREAM // TODO need to have this function to configure fifo bypass
int adxl345_configure_fifo(const struct device *dev, enum adxl345_fifo_mode mode,
		enum adxl345_fifo_trigger trigger, uint8_t fifo_samples);

#ifdef CONFIG_ADXL345_STREAM
size_t adxl345_get_packet_size(const struct adxl345_dev_config *cfg);
#endif /* CONFIG_ADXL345_STREAM */ // TODO rm
#endif /* ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_ */
