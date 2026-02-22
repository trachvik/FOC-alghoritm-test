#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include "haptic.h"
#include "drivers/as5048a.h"

/* Device instances */
static struct as5048a_device encoder;
static bldc_driver_6pwm_t driver;
static bldc_motor_t motor;

/* Global pointer for sensor wrapper */
struct as5048a_device *g_as5048a = &encoder;

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

	haptic_init(&motor, (bldc_driver_t*)&driver, (sensor_t*)&encoder);

	static int loop_count = 0;
	while (1)
	{
		//if (loop_count++ == 0) {
		//	printk("DEBUG: Entered main loop!\n");
		//}
		haptic_loop(&motor, (sensor_t*)&encoder);
		k_msleep(1);  /* Yield to other threads */
	}
}
