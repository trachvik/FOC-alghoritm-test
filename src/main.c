#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include "haptic.h"
#include "drivers/as5048a.h"

/* Fault handler — blika SOS na LED kdyz MCU crashne.
 * POZOR: k_msleep zde nesmi byt — pouzivame k_busy_wait. */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);
	printk("\n!!! FATAL ERROR reason=%u !!!\n", reason);
	const struct gpio_dt_spec fault_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	if (gpio_is_ready_dt(&fault_led)) {
		gpio_pin_configure_dt(&fault_led, GPIO_OUTPUT_ACTIVE);
		while (1) {
			/* SOS: 3 kratke */
			for (int i = 0; i < 3; i++) {
				gpio_pin_set_dt(&fault_led, 1); k_busy_wait(150000);
				gpio_pin_set_dt(&fault_led, 0); k_busy_wait(150000);
			}
			/* 3 dlouhe */
			for (int i = 0; i < 3; i++) {
				gpio_pin_set_dt(&fault_led, 1); k_busy_wait(500000);
				gpio_pin_set_dt(&fault_led, 0); k_busy_wait(150000);
			}
			/* 3 kratke */
			for (int i = 0; i < 3; i++) {
				gpio_pin_set_dt(&fault_led, 1); k_busy_wait(150000);
				gpio_pin_set_dt(&fault_led, 0); k_busy_wait(150000);
			}
			k_busy_wait(1000000);
		}
	}
	CODE_UNREACHABLE;
}

/* Device instances */
static struct as5048a_device encoder;
static bldc_driver_6pwm_t driver;
static bldc_motor_t motor;

/* Global pointer for sensor wrapper */
struct as5048a_device *g_as5048a = &encoder;

int main(void)
{
	int ret;

	/* Diagnostika: blikneme LED hned na startu — overime ze main() bezi */
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		for (int i = 0; i < 6; i++) {
			gpio_pin_toggle_dt(&led);
			k_msleep(100);
		}
	}

	/* Enable USB device */
	ret = usb_enable(NULL);
	if (ret != 0) {
		/* Blink fast = usb_enable failed */
		for (int i = 0; i < 20; i++) {
			gpio_pin_toggle_dt(&led);
			k_msleep(50);
		}
		return -1;
	}

	/* Cekej na DTR — 'screen /dev/ttyACM0 115200' ho nastavi automaticky.
	 * Bez toho by vsechny printky pred otevrenim portu byly zahozeny.     */
	const struct device *cdc = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
	if (device_is_ready(cdc)) {
		uint32_t dtr = 0;
		while (!dtr) {
			uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr);
			gpio_pin_toggle_dt(&led);  /* blikej dokud cekame */
			k_msleep(200);
		}
	}

	/* CDC potrebuje ~500 ms po DTR nez je TX buffer aktivni */
	gpio_pin_set_dt(&led, 1);
	k_msleep(500);

	printk("\n\n=== main() USB ready, starting haptic_init ===\n");
	k_msleep(100);

	/* Overeni ze serial vystup funguje — 5x za sebou */
	for (int i = 1; i <= 5; i++) {
		printk("Serial check %d/5\n", i);
		k_msleep(300);
	}

	/* Flush delay — USB CDC potrebuje cas odeslat buffered data */
	k_msleep(500);
	printk("--- calling haptic_init ---\n");
	k_msleep(200);

	haptic_init(&motor, (bldc_driver_t*)&driver, (sensor_t*)&encoder);

	/* haptic_loop is now driven by a 10 kHz timer thread in haptic.c */
	while (1) {
		k_sleep(K_FOREVER);
	}
}
