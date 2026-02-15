/**
 * @file as5048a.h
 * @brief AS5048A 14-bit Magnetic Rotary Position Sensor Driver
 * 
 * Driver for AS5048A encoder using SPI interface.
 */

#ifndef AS5048A_H_
#define AS5048A_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>

/**
 * @brief AS5048A device structure
 */
struct as5048a_device {
	const struct spi_dt_spec *spi;
};

/**
 * @brief Initialize AS5048A encoder
 * 
 * @param dev Pointer to AS5048A device structure
 * @param spi Pointer to SPI device tree spec
 * @return 0 on success, negative errno code on failure
 */
int as5048a_init(struct as5048a_device *dev, const struct spi_dt_spec *spi);

/**
 * @brief Read raw angle value (0-16383)
 * 
 * @param dev Pointer to AS5048A device structure
 * @param angle Pointer to store raw angle value
 * @return 0 on success, negative errno code on failure
 */
int as5048a_read_raw(struct as5048a_device *dev, uint16_t *angle);

/**
 * @brief Read angle in degrees (0-360)
 * 
 * @param dev Pointer to AS5048A device structure
 * @param angle_deg Pointer to store angle in degrees
 * @return 0 on success, negative errno code on failure
 */
int as5048a_read_angle_deg(struct as5048a_device *dev, float *angle_deg);

/**
 * @brief Read angle in radians (0-2π)
 * 
 * @param dev Pointer to AS5048A device structure
 * @param angle_rad Pointer to store angle in radians
 * @return 0 on success, negative errno code on failure
 */
int as5048a_read_angle_rad(struct as5048a_device *dev, float *angle_rad);

#endif /* AS5048A_H_ */
