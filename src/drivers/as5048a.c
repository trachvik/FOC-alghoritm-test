/**
 * @file as5048a.c
 * @brief AS5048A 14-bit Magnetic Rotary Position Sensor Driver Implementation
 */

#include "as5048a.h"
#include "bldc_motor.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(as5048a, LOG_LEVEL_INF);

/* M_PI constant if not defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* AS5048A SPI Commands */
#define AS5048A_CMD_ANGLE    0xFFFF  /* Read angle register */

/* Constants */
#define ANGLE_RESOLUTION 16384.0f  /* 14-bit resolution = 2^14 = 16384 */

int as5048a_init(struct as5048a_device *dev, const struct spi_dt_spec *spi)
{
	if (!dev || !spi) {
		return -EINVAL;
	}

	if (!device_is_ready(spi->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	dev->spi = spi;
	
	LOG_INF("AS5048A encoder initialized");
	
	return 0;
}

int as5048a_read_raw(struct as5048a_device *dev, uint16_t *angle)
{
	if (!dev || !angle) {
		return -EINVAL;
	}

	/* Single SPI transaction to read angle */
	uint16_t tx_data = AS5048A_CMD_ANGLE;
	uint16_t rx_data = 0;

	const struct spi_buf tx_buf = {
		.buf = &tx_data,
		.len = sizeof(tx_data)
	};
	const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1
	};

	const struct spi_buf rx_buf = {
		.buf = &rx_data,
		.len = sizeof(rx_data)
	};
	const struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1
	};

	int ret = spi_transceive_dt(dev->spi, &tx_bufs, &rx_bufs);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		return ret;
	}

	/* Extract 14-bit angle data */
	*angle = rx_data & 0x3FFF;

	return 0;
}

int as5048a_read_angle_deg(struct as5048a_device *dev, float *angle_deg)
{
	uint16_t raw_angle;
	int ret = as5048a_read_raw(dev, &raw_angle);
	if (ret < 0) {
		return ret;
	}

	/* Convert to degrees (0-360) */
	*angle_deg = ((float)raw_angle / ANGLE_RESOLUTION) * 360.0f;

	return 0;
}

int as5048a_read_angle_rad(struct as5048a_device *dev, float *angle_rad)
{
	uint16_t raw_angle;
	int ret = as5048a_read_raw(dev, &raw_angle);
	if (ret < 0) {
		return ret;
	}

	/* Convert to radians (0-2π) */
	*angle_rad = ((float)raw_angle / ANGLE_RESOLUTION) * 2.0f * M_PI;

	return 0;
}

/*
 * Sensor interface wrapper functions for FOC library
 */

/* Global pointer to AS5048A device - set from main */
extern struct as5048a_device *g_as5048a;

/* Cached angle value - updated once per control cycle by sensor_update() */
static float g_cached_angle_rad = 0.0f;

void sensor_update(sensor_t *sensor)
{
	/* Read and cache the current angle - call once per control cycle */
	if (g_as5048a == NULL) return;
	as5048a_read_angle_rad(g_as5048a, &g_cached_angle_rad);
}

float sensor_get_angle(sensor_t *sensor)
{
	/* Return cached value - avoids multiple SPI reads per cycle */
	return g_cached_angle_rad;
}

float sensor_get_velocity(sensor_t *sensor)
{
	/* Calculate velocity from angle difference */
	/* TODO: Implement velocity calculation */
	return 0.0f;
}

bool sensor_needs_search(sensor_t *sensor)
{
	/* No index search needed for absolute encoder */
	return false;
}
