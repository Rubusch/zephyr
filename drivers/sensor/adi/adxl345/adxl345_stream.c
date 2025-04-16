/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include "adxl345.h"

LOG_MODULE_DECLARE(ADXL345, CONFIG_SENSOR_LOG_LEVEL);

/* helpers */

//* // TODO needed?
// TODO check struct rtio_iodev_sqe *current_sqe = data->sqe;
static void adxl345_prep_rtio_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t buf_len)
{
	struct adxl345_dev_data *data = dev->data;
	const struct adxl345_dev_config *cfg = dev->config; 
	struct rtio_sqe *write_status_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_status_reg = rtio_sqe_acquire(data->rtio_ctx);

	struct rtio_sqe *check_status_reg = rtio_sqe_acquire(data->rtio_ctx);

	uint8_t read_reg = ADXL345_REG_READ(reg);
	read_reg |= buf_len > 1 ? ADXL345_MULTIBYTE_FLAG : 0;

//	struct rtio_iodev_sqe *current_sqe = data->sqe; // XXX
	rtio_sqe_prep_tiny_write(write_status_addr, data->iodev, RTIO_PRIO_NORM, &read_reg, 1, NULL);
//	rtio_sqe_prep_tiny_write(write_status_addr, data->iodev, RTIO_PRIO_NORM, &read_reg, 1, current_sqe);

	write_status_addr->flags = RTIO_SQE_TRANSACTION;
//	write_status_addr->flags = RTIO_SQE_CHAINED;
	rtio_sqe_prep_read(read_status_reg, data->iodev, RTIO_PRIO_NORM, buf, buf_len, NULL);
//	rtio_sqe_prep_read(read_status_reg, data->iodev, RTIO_PRIO_NORM, buf, buf_len, current_sqe);

	read_status_reg->flags = RTIO_SQE_CHAINED;
	if (cfg->bus_type == ADXL345_BUS_I2C) {
		read_status_reg->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
	}

	rtio_submit(data->rtio_ctx, 0); // TODO rm
}

static void adxl345_prep_rtio_read_reg(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	adxl345_prep_rtio_read(dev, reg, buf, 1);
}
// */

static void adxl345_prep_rtio_write(const struct device *dev,
				    uint8_t reg,
				    uint8_t val)
{
	struct adxl345_dev_data *data = dev->data;
	struct rtio_sqe *write_reg_sqe = rtio_sqe_acquire(data->rtio_ctx);

//LOG_WRN("reg_addr_w[] = { %02x, %02x } // reg, val", reg, val); // TODO rm

	const uint8_t reg_addr_w[2] = {reg, val};
	
	rtio_sqe_prep_tiny_write(write_reg_sqe, data->iodev, RTIO_PRIO_NORM,
				 reg_addr_w, sizeof(reg_addr_w), NULL);

//	rtio_sqe_prep_write(write_reg_sqe, data->iodev, RTIO_PRIO_NORM,
//				 reg_addr_w, sizeof(reg_addr_w), NULL);

}

static void adxl345_prep_rtio_update_cached_bits(const struct device *dev,
						 uint8_t *cached_reg,
						 uint8_t reg,
						 uint8_t mask,
						 uint8_t val)
{
	struct adxl345_dev_data *data = dev->data;
	struct rtio_sqe *write_reg_sqe = rtio_sqe_acquire(data->rtio_ctx);
	uint8_t tmp;

//LOG_INF("reg:\t%02x (before)", *cached_reg); // TODO rm

	tmp = *cached_reg & ~mask;
	tmp |= val & mask;

	const uint8_t reg_addr_w[2] = {reg, tmp};
	rtio_sqe_prep_tiny_write(write_reg_sqe, data->iodev, RTIO_PRIO_NORM,
				 reg_addr_w, sizeof(reg_addr_w), NULL);

	*cached_reg = tmp;

//LOG_INF("reg:\t%02x (after)", *cached_reg); // TODO rm
}


//*
//void rtio_debug_regs_cb(struct rtio *r, const struct rtio_sqe *sqr, void *arg)
//{
//	const struct device *dev = (const struct device *)arg;
//	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;
//
//	LOG_INF("reg:buf %02x:%02x [cb]", data->cached_debug_reg, data->cached_debug_regval); // TODO rm
//}

void rtio_debug_regs(const struct device *dev)
{
	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;

	LOG_INF("%s(): -----------------------------", __func__);

//	data->cached_debug_reg = ADXL345_REG_POWER_CTL;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg POWER_CTL [%02X]:\t\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);
//
//	data->cached_debug_reg = ADXL345_REG_INT_ENABLE;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg INT_ENABLE [%02X]:\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);
//
//	data->cached_debug_reg = ADXL345_REG_INT_MAP;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg INT_MAP [%02X]:\t\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);

	data->cached_debug_reg = ADXL345_REG_DATA_FORMAT;
	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
	k_sleep(K_MSEC(200));
	LOG_INF("%s(): reg DATA_FORMAT [%02X]:\t%02x",
		__func__, data->cached_debug_reg, data->cached_debug_regval);
	
//	data->cached_debug_reg = ADXL345_REG_FIFO_CTL;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg FIFO_CTL [%02X]:\t\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);
//
//	LOG_INF("%s(): -----------------------------", __func__);
//
//	data->cached_debug_reg = ADXL345_REG_INT_SOURCE;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg INT_SOURCE [%02X]:\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);
//
//	data->cached_debug_reg = ADXL345_REG_FIFO_STATUS;
//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	k_sleep(K_MSEC(200));
//	LOG_INF("%s(): reg FIFO_STATUS [%02X]:\t%02x",
//		__func__, data->cached_debug_reg, data->cached_debug_regval);




//	reg = data->cached_debug_reg;
//	adxl345_reg_read_byte(dev, data->cached_debug_reg, &regval);
//	LOG_INF("%s(): reg POWER_CTL [%02X]:\t\t%02x", __func__, reg, regval);

	


//	adxl345_prep_rtio_read_reg(dev, data->cached_debug_reg, &data->cached_debug_regval);
//	struct rtio_sqe *op_2d = rtio_sqe_acquire(data->rtio_ctx);
//	rtio_sqe_prep_callback(op_2d, rtio_debug_regs_cb, (void *) dev, NULL);
//	rtio_submit(data->rtio_ctx, 0);
//
//k_sleep(K_MSEC(100)); // TODO rm XXX
//	LOG_INF("reg:buf %02x:%02x [after]", data->cached_debug_reg, data->cached_debug_regval); // TODO rm
//
//
//
//
//	LOG_INF("XXX"); // TODO rm
//	regval = 0;
//	reg = 0;
//k_sleep(K_MSEC(100)); // TODO rm XXX
//	reg = data->cached_debug_reg;
//	adxl345_reg_read_byte(dev, reg, &regval);
//	LOG_INF("%s(): reg POWER_CTL [%02X]:\t\t%02x", __func__, reg, regval);
//
//
//
//
//	uint8_t regval_2e, reg_2e = ADXL345_REG_INT_ENABLE;
//	adxl345_prep_rtio_read_reg(dev, reg_2e, &regval_2e);
//	rtio_submit(data->rtio_ctx, 0);
//
//	uint8_t regval_2f, reg_2f = ADXL345_REG_INT_MAP;
//	adxl345_prep_rtio_read_reg(dev, reg_2f, &regval_2f);
//	rtio_submit(data->rtio_ctx, 0);
//
//	uint8_t regval_31, reg_31 = ADXL345_REG_DATA_FORMAT;
//	adxl345_prep_rtio_read_reg(dev, reg_31, &regval_31);
//	rtio_submit(data->rtio_ctx, 0);
//
//	uint8_t regval_38, reg_38 = ADXL345_REG_FIFO_CTL;
//	adxl345_prep_rtio_read_reg(dev, reg_38, &regval_38);
//	rtio_submit(data->rtio_ctx, 0);
//
//	uint8_t regval_30, reg_30 = ADXL345_REG_INT_SOURCE;
//	adxl345_prep_rtio_read_reg(dev, reg_30, &regval_30);
//	rtio_submit(data->rtio_ctx, 0);
//
//	uint8_t regval_39, reg_39 = ADXL345_REG_FIFO_STATUS;
//	adxl345_prep_rtio_read_reg(dev, reg_39, &regval_39);
//	rtio_submit(data->rtio_ctx, 0);

	/* submit sqe */
//	struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
//	write_fifo_addr->flags |= RTIO_SQE_CHAINED;

//	rtio_submit(data->rtio_ctx, 0);

//	LOG_INF("%s(): reg POWER_CTL [%02X]:\t\t%02x", __func__, reg_2d, regval_2d);
//	LOG_INF("%s(): reg INT_ENABLE [%02X]:\t%02x", __func__, reg_2e, regval_2e);
//	LOG_INF("%s(): reg INT_MAP [%02X]:\t\t%02x", __func__, reg_2f, regval_2f);
//	LOG_INF("%s(): reg DATA_FORMAT [%02X]:\t%02x", __func__, reg_31, regval_31);
//	LOG_INF("%s(): reg FIFO_CTL [%02X]:\t\t%02x", __func__, reg_38, regval_38);
//	LOG_INF("%s(): -----------------------------", __func__);
//	LOG_INF("%s(): reg INT_SOURCE [%02X]:\t%02x", __func__, reg_30, regval_30);
//	LOG_INF("%s(): reg FIFO_STATUS [%02X]:\t%02x", __func__, reg_39, regval_39);
	LOG_INF("%s(): =============================", __func__);
}
// */




static void adxl345_prep_rtio_measure_en(const struct device *dev, bool en)
{
	struct adxl345_dev_data *data = dev->data;

LOG_INF("RTIO (prep) MEASURE: %s", en ? "on" : "off"); // TODO rm
	adxl345_prep_rtio_update_cached_bits(dev,
					     &data->cached_power_ctl,
					     ADXL345_REG_POWER_CTL,
					     ADXL345_POWER_CTL_MEASURE_MODE,
					     en ? 0xff : 0x00);
}

/* streaming implementation */

static void adxl345_irq_en_cb(struct rtio *r, const struct rtio_sqe *sqr, void *arg)
{
	const struct device *dev = (const struct device *)arg;

LOG_INF("RTIO: called"); // TODO rm

	/* enable trigger defined interrupt line(s) */
	adxl345_set_gpios_en(dev, true);

// TODO needed here?
//	adxl345_prep_rtio_measure_en(dev, true);
}

static void adxl345_fifo_flush_rtio(const struct device *dev)
{
	struct adxl345_dev_data *data = dev->data;
//	struct rtio_sqe *clear_fifo = rtio_sqe_acquire(data->rtio_ctx);
//	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);

LOG_INF("RTIO: called"); // TODO rm

	/* disable measurement */
	adxl345_prep_rtio_measure_en(dev, false);
	rtio_submit(data->rtio_ctx, 0);


	
	/* clear interrupt status and fifo */
	adxl345_prep_rtio_write(dev, ADXL345_REG_INT_ENABLE, 0x00);
	rtio_submit(data->rtio_ctx, 0);

	/* clear FIFO i.e. entries, data ready, overrun and watermark */
	adxl345_prep_rtio_write(dev, ADXL345_REG_FIFO_CTL,
				adxl345_fifo_ctl_mode_init[ADXL345_FIFO_BYPASSED]);
	rtio_submit(data->rtio_ctx, 0);



	/* reset registers */
	adxl345_prep_rtio_write(dev, ADXL345_REG_FIFO_CTL,
				data->fifo_config.fifo_mode);
	rtio_submit(data->rtio_ctx, 0);

	adxl345_prep_rtio_write(dev, ADXL345_REG_INT_ENABLE,
				data->cached_int_enable);
	rtio_submit(data->rtio_ctx, 0);


	
	/* re-enable measurement */
//	adxl345_prep_rtio_measure_en(dev, true);

	/* submit cqes */
//	struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
//	write_fifo_addr->flags |= RTIO_SQE_CHAINED;

	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);
	rtio_sqe_prep_callback(complete_op, adxl345_irq_en_cb, (void *)dev, NULL);
	rtio_submit(data->rtio_ctx, 0);

rtio_debug_regs(dev); // TODO rm XXX
}

void adxl345_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
LOG_INF("RTIO: called"); // TODO rm

	const struct sensor_read_config *cfg =
			(const struct sensor_read_config *) iodev_sqe->sqe.iodev->data;
	struct adxl345_dev_data *data = (struct adxl345_dev_data *)dev->data;
	uint8_t int_value = 0;
	int rc;

	adxl345_set_gpios_en(dev, false);
//	__ASSERT(rc == 0, "Disabling interrupts failed");
//	adxl345_prep_rtio_measure_en(dev, false); // TODO needed? rm

	for (size_t i = 0; i < cfg->count; i++) {
		/* FIFO: data ready */
		if (cfg->triggers[i].trigger == SENSOR_TRIG_DATA_READY) {
			int_value |= ADXL345_INT_DATA_RDY;
		}

		/* FIFO: watermark */
		if (cfg->triggers[i].trigger == SENSOR_TRIG_FIFO_WATERMARK) {
			int_value |= ADXL345_INT_WATERMARK;
		}

		/* FIFO: overrun */
		if (cfg->triggers[i].trigger == SENSOR_TRIG_FIFO_FULL) {
			int_value |= ADXL345_INT_OVERRUN;
		}
	}

	if (FIELD_GET(ADXL345_INT_DATA_RDY, int_value)) {
		adxl345_prep_rtio_update_cached_bits(dev,
						     &data->cached_int_enable,
						     ADXL345_REG_INT_ENABLE,
						     ADXL345_INT_DATA_RDY,
						     int_value);
	}

	if (FIELD_GET(ADXL345_INT_WATERMARK, int_value)) {
		adxl345_prep_rtio_update_cached_bits(dev,
						     &data->cached_int_enable,
						     ADXL345_REG_INT_ENABLE,
						     ADXL345_INT_WATERMARK,
						     int_value);
	}

	if (FIELD_GET(ADXL345_INT_OVERRUN, int_value)) {
		adxl345_prep_rtio_update_cached_bits(dev,
						     &data->cached_int_enable,
				 		     ADXL345_REG_INT_ENABLE,
						     ADXL345_INT_OVERRUN,
						     int_value);
	}

	/* flash FIFO and submit sqes, enable gpio lines and enable measure */
	adxl345_fifo_flush_rtio(dev);
	adxl345_set_gpios_en(dev, true);

	data->sqe = iodev_sqe;
}

static void adxl345_fifo_read_cb(struct rtio *rtio_ctx, const struct rtio_sqe *sqe, void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;
	struct rtio_iodev_sqe *iodev_sqe = sqe->userdata;

LOG_INF("RTIO: called"); // TODO rm

	/*
	 * Return if fifo not empty. The callback here is supposed to evaluate
	 * what was read earlier in the (same) sqe chain from FIFO_STATUS.
	 */
// TODO rm, needed? Actually is placed at entries-1 position inside the loop, if fifo then was still not
// empty, there would be no consequence, so either drop this entirely -> ok, or ??? - verify
	if (data->fifo_samples) {
		return;
	}

	data->fifo_total_bytes = 0;
	rtio_iodev_sqe_ok(iodev_sqe, 0);
	adxl345_set_gpios_en(dev, true);
	adxl345_prep_rtio_measure_en(dev, true);
}

static void adxl345_process_fifo_samples_cb(struct rtio *r, const struct rtio_sqe *sqr, void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;
	const struct adxl345_dev_config *cfg = (const struct adxl345_dev_config *) dev->config;
	struct rtio_iodev_sqe *current_sqe = data->sqe;
//	uint16_t fifo_samples = (data->fifo_ent[0]) & ADXL345_FIFO_COUNT_MASK;
	uint8_t fifo_samples = FIELD_GET(ADLX345_FIFO_STATUS_ENTRIES_MSK, data->fifo_ent[0]); // TODO check this evaluates FIFO_STATUS entries reg
//	size_t sample_set_size = ADXL345_FIFO_SAMPLE_SIZE; // TODO rm
	uint8_t fifo_bytes = fifo_samples * ADXL345_FIFO_SAMPLE_SIZE;
//	int ret; // TODO rm

LOG_INF("RTIO: called"); // TODO rm

	data->sqe = NULL;

	/* Not inherently an underrun/overrun as we may have a buffer to fill next time */
	if (current_sqe == NULL) {
		LOG_ERR("No pending SQE");
		goto err;
	}

	const size_t min_read_size = sizeof(struct adxl345_fifo_data) + ADXL345_FIFO_SAMPLE_SIZE;
	const size_t ideal_read_size = sizeof(struct adxl345_fifo_data) + fifo_bytes;

	uint8_t *buf;
	uint32_t buf_len;

	if (rtio_sqe_rx_buf(current_sqe, min_read_size, ideal_read_size, &buf, &buf_len) != 0) {
		LOG_ERR("Failed to get buffer");
		rtio_iodev_sqe_err(current_sqe, -ENOMEM);
		goto err;
	}
	LOG_DBG("Requesting buffer [%u, %u] got %u", (unsigned int)min_read_size,
		(unsigned int)ideal_read_size, buf_len);

	/* Read FIFO and call back to rtio with rtio_sqe completion */
	struct adxl345_fifo_data *hdr = (struct adxl345_fifo_data *) buf;

	hdr->is_fifo = 1; // TODO needed?
	hdr->timestamp = data->timestamp; 
	hdr->int_status = data->status; // TODO is this cached_int_status? duplicate?
	hdr->is_full_res = data->is_full_res;
	hdr->selected_range = data->selected_range;
	hdr->accel_odr = data->odr;
	hdr->sample_set_size = ADXL345_FIFO_SAMPLE_SIZE; // TODO rm, this is constant ADXL345_FIFO_SAMPLE_SIZE

	uint32_t buf_avail = buf_len;

	buf_avail -= sizeof(*hdr);

	uint32_t read_len = MIN(fifo_bytes, buf_avail);

	if (buf_avail < fifo_bytes) {
		uint32_t pkts = read_len / ADXL345_FIFO_SAMPLE_SIZE;
		read_len = pkts * ADXL345_FIFO_SAMPLE_SIZE;
	}

	((struct adxl345_fifo_data *)buf)->fifo_byte_count = read_len;

	uint8_t *read_buf = buf + sizeof(*hdr);

	/* Flush completions */
	struct rtio_cqe *cqe;
	int res = 0;

	do {
		cqe = rtio_cqe_consume(data->rtio_ctx);
		if (cqe) {
			if ((cqe->result < 0 && res == 0)) {
				LOG_ERR("Bus error: %d", cqe->result);
				res = cqe->result;
			}
			rtio_cqe_release(data->rtio_ctx, cqe);
		}
	} while (cqe != NULL);

	/* Bail/cancel attempt to read sensor on any error */
	if (res) {
		rtio_iodev_sqe_err(current_sqe, res);
		return;
	}

	data->fifo_config.fifo_samples = fifo_samples;
	for (size_t i = 0; i < fifo_samples; i++) {
		struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
		struct rtio_sqe *read_fifo_data = rtio_sqe_acquire(data->rtio_ctx);

		data->fifo_config.fifo_samples--;
		const uint8_t reg_addr = ADXL345_REG_READ(ADXL345_REG_DATAX0)
				| ADXL345_MULTIBYTE_FLAG;

		rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, &reg_addr,
								1, NULL); // TODO use sizeof(*reg_addr)
		write_fifo_addr->flags = RTIO_SQE_TRANSACTION;
		rtio_sqe_prep_read(read_fifo_data, data->iodev, RTIO_PRIO_NORM,
				   read_buf + data->fifo_total_bytes,
				   ADXL345_FIFO_SAMPLE_SIZE, current_sqe);
		data->fifo_total_bytes += ADXL345_FIFO_SAMPLE_SIZE;
		if (cfg->bus_type == ADXL345_BUS_I2C) {
			read_fifo_data->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
		}

		if (i == fifo_samples - 1) {
			struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);

			read_fifo_data->flags = RTIO_SQE_CHAINED;
			rtio_sqe_prep_callback(complete_op, adxl345_fifo_read_cb, (void *)dev,
				current_sqe);
		}
		rtio_submit(data->rtio_ctx, 0);
		ARG_UNUSED(rtio_cqe_consume(data->rtio_ctx));
	}

	return;
err:

	adxl345_set_gpios_en(dev, true);
//	adxl345_prep_rtio_measure_en(dev, true); // TODO needed?
}

static void adxl345_process_status_cb(struct rtio *r, const struct rtio_sqe *sqr, void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;
	const struct adxl345_dev_config *cfg = (const struct adxl345_dev_config *) dev->config;
	struct rtio_iodev_sqe *current_sqe = data->sqe;
	struct sensor_read_config *read_config;
	uint8_t status = data->status;
	int rc;

LOG_INF("RTIO: called"); // TODO rm

	__ASSERT(data->sqe != NULL, "%s data->sqe == NULL", __func__);
	read_config = (struct sensor_read_config *)data->sqe->sqe.iodev->data;
	__ASSERT(read_config != NULL, "%s read_config == NULL", __func__);
	if (read_config->is_streaming == false) {
		return; /* not for us */
	}

	rc = adxl345_set_gpios_en(dev, false);
	__ASSERT(rc == 0, "Disabling interrupt lines failed");
//	adxl345_prep_rtio_measure_en(dev, false); // TODO needed? rm, this is regular ISR processing

//	struct sensor_stream_trigger *fifo_drdy_cfg = NULL;
	struct sensor_stream_trigger *fifo_wmark_cfg = NULL;
	struct sensor_stream_trigger *fifo_full_cfg = NULL;

	for (int i = 0; i < read_config->count; ++i) {
// TODO SENSOR_TRIG_DATA_READY -> data ready event? needed?
//		if (read_config->triggers[i].trigger == SENSOR_TRIG_DATA_READY) {
//			fifo_drdy_cfg = &read_config->triggers[i];
//			continue;
//		}

		if (read_config->triggers[i].trigger == SENSOR_TRIG_FIFO_WATERMARK) {
			fifo_wmark_cfg = &read_config->triggers[i];
			continue;
		}
	
// TODO SENSOR_TRIG_FIFO_FULL -> overrun event?
		if (read_config->triggers[i].trigger == SENSOR_TRIG_FIFO_FULL) {
			fifo_full_cfg = &read_config->triggers[i];
			continue;
		}
	}

//	bool fifo_drdy_irq = false;
	bool fifo_wmark_irq = false;
	bool fifo_full_irq = false;

//	if ((fifo_drdy_cfg != NULL) && FIELD_GET(ADXL345_INT_DATA_RDY, status)) {
//		fifo_drdy_irq = true;
//	}

	if ((fifo_wmark_cfg != NULL) && FIELD_GET(ADXL345_INT_WATERMARK, status)) {
		fifo_wmark_irq = true;
	}

	if ((fifo_full_cfg != NULL) && FIELD_GET(ADXL345_INT_OVERRUN, status)) {
		fifo_full_irq = true;
	}

	if (/* !fifo_drdy_irq && */ !fifo_full_irq && !fifo_wmark_irq) {
// TODO ->interrupt changed to gpio_int1 or gpio_int2 (???)
		goto done;
	}

	/* Flush completions */
	struct rtio_cqe *cqe;
	int res = 0;

	do {
		cqe = rtio_cqe_consume(data->rtio_ctx);
		if (cqe != NULL) {
			if ((cqe->result < 0) && (res == 0)) {
				LOG_ERR("Bus error: %d", cqe->result);
				res = cqe->result;
			}
			rtio_cqe_release(data->rtio_ctx, cqe);
		}
	} while (cqe != NULL);

	/* Bail/cancel attempt to read sensor on any error */
	if (res != 0) {
		rtio_iodev_sqe_err(current_sqe, res);
		return;
	}

// TODO WTF??!!!! if we want drdy to be handled, too?
	enum sensor_stream_data_opt data_opt;
	if (fifo_wmark_cfg != NULL && fifo_full_cfg == NULL) {
		data_opt = fifo_wmark_cfg->opt;
	} else if (fifo_wmark_cfg == NULL && fifo_full_cfg != NULL) {
		data_opt = fifo_full_cfg->opt;
	} else {
		data_opt = MIN(fifo_wmark_cfg->opt, fifo_full_cfg->opt);
	}
// TODO fifo_drdy_cfg missing

	if (data_opt == SENSOR_STREAM_DATA_NOP || data_opt == SENSOR_STREAM_DATA_DROP) {
		uint8_t *buf;
		uint32_t buf_len;

		/* Clear streaming_sqe since we're done with the call */
		data->sqe = NULL;
		if (rtio_sqe_rx_buf(current_sqe, sizeof(struct adxl345_fifo_data),
				    sizeof(struct adxl345_fifo_data), &buf, &buf_len) != 0) {
			rtio_iodev_sqe_err(current_sqe, -ENOMEM);
			goto done;
		}

		struct adxl345_fifo_data *rx_data = (struct adxl345_fifo_data *)buf;

		memset(buf, 0, buf_len);
		rx_data->is_fifo = 1;
		rx_data->timestamp = data->timestamp;
		rx_data->int_status = status;
		rx_data->fifo_byte_count = 0;
		rtio_iodev_sqe_ok(current_sqe, 0);

		if (data_opt == SENSOR_STREAM_DATA_DROP) {
			/* Flush the FIFO by disabling it. Save current mode for after the reset. */
			adxl345_fifo_flush_rtio(dev);
		}

		goto done;
	}

	struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_fifo_data = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);
	const uint8_t reg_addr = ADXL345_REG_READ(ADXL345_REG_FIFO_STATUS);

	rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, &reg_addr, 1, NULL);
	write_fifo_addr->flags = RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_fifo_data, data->iodev, RTIO_PRIO_NORM, data->fifo_ent, 1,
						current_sqe);
	read_fifo_data->flags = RTIO_SQE_CHAINED;
	if (cfg->bus_type == ADXL345_BUS_I2C) {
		read_fifo_data->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
	}

	rtio_sqe_prep_callback(complete_op, adxl345_process_fifo_samples_cb, (void *)dev,
							current_sqe);

	rtio_submit(data->rtio_ctx, 0);

	return;
done:
	rc = adxl345_set_gpios_en(dev, true);
	__ASSERT(rc == 0, "Enabling interrupt lines failed");
//	adxl345_prep_rtio_measure_en(dev, true); // TODO needed?
}

void adxl345_stream_irq_handler(const struct device *dev)
{
	struct adxl345_dev_data *data = (struct adxl345_dev_data *) dev->data;
	const struct adxl345_dev_config *cfg = (const struct adxl345_dev_config *) dev->config;
	uint64_t cycles;
	int rc;

LOG_INF("called - ??? ISR ???"); // TODO rm
	
	if (data->sqe == NULL) {
		return;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc) {
		LOG_ERR("Failed to get sensor clock cycles");
		rtio_iodev_sqe_err(data->sqe, rc);
		return;
	}

	data->timestamp = sensor_clock_cycles_to_ns(cycles);

	struct rtio_sqe *write_status_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_status_reg = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *check_status_reg = rtio_sqe_acquire(data->rtio_ctx);
	uint8_t reg = ADXL345_REG_READ(ADXL345_REG_INT_SOURCE);

	rtio_sqe_prep_tiny_write(write_status_addr, data->iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
	write_status_addr->flags = RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_status_reg, data->iodev, RTIO_PRIO_NORM, &data->status, 1, NULL);
	read_status_reg->flags = RTIO_SQE_CHAINED;
	if (cfg->bus_type == ADXL345_BUS_I2C) {
		read_status_reg->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
	}

	rtio_sqe_prep_callback(check_status_reg, adxl345_process_status_cb, (void *)dev, NULL);
	rtio_submit(data->rtio_ctx, 0);
}
