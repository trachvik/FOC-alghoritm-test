#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/spi.h>
#include "drivers/as5048a.h"

/* AS5048A encoder from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);

/* Device instance */
static struct as5048a_device encoder;

int main(void)
{
int ret;

/* Enable USB device */
ret = usb_enable(NULL);
if (ret != 0) {
printk("Failed to enable USB: %d\n", ret);
return -1;
}

/* Wait for USB to be ready */
k_msleep(1000);

printk("\n");
printk("================================================\n");
printk("  BLDC Motor FOC Controller\n");
	printk("  Starting from scratch (AS5048A OK)\n");
	printk("================================================\n");
	printk("\n");

	/* Initialize AS5048A encoder */
	printk("Initializing AS5048A encoder...\n");
	ret = as5048a_init(&encoder, &as5048a_spi);
	if (ret < 0) {
		printk("ERROR: Failed to initialize AS5048A: %d\n", ret);
		return -1;
	}
	printk("  [OK] AS5048A ready\n\n");

	while (1) {
		uint16_t raw_angle;
		float angle_deg;
		
		if (as5048a_read_raw(&encoder, &raw_angle) == 0) {
			angle_deg = ((float)raw_angle / 16384.0f) * 360.0f;
			printk("Encoder: %5u (raw) = %6.2f°\n", raw_angle, (double)angle_deg);
		}
		k_msleep(100);
	}
}
