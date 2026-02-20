#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/spi.h>
#include "drivers/as5048a.h"
#include "drivers/bldc_motor.h"
#include "drivers/bldc_driver_6pwm.h"

/* AS5048A encoder from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);

/* PWM pin definitions for 6PWM BLDC driver */
/* TODO: Update these pin numbers based on your actual hardware */
#define PWM_AH_PIN  0   /* Phase A high-side */
#define PWM_AL_PIN  1   /* Phase A low-side */
#define PWM_BH_PIN  2   /* Phase B high-side */
#define PWM_BL_PIN  3   /* Phase B low-side */
#define PWM_CH_PIN  4   /* Phase C high-side */
#define PWM_CL_PIN  5   /* Phase C low-side */
#define ENABLE_PIN  NOT_SET  /* Optional enable pin */

/* Motor parameters */
#define MOTOR_POLE_PAIRS 11     /* Number of pole pairs */
#define MOTOR_PHASE_RESISTANCE 0.5f  /* Ohms */
#define MOTOR_KV_RATING 320.0f  /* rpm/V */
#define MOTOR_INDUCTANCE 0.0001f /* H */

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

printk("\n");
printk("================================================\n");
printk("  BLDC Motor FOC Controller\n");
printk("  Initializing...\n");
printk("================================================\n");
printk("\n");

/* Initialize AS5048A encoder */
printk("1. Initializing AS5048A encoder...\n");
ret = as5048a_init(&encoder, &as5048a_spi);
if (ret < 0) {
	printk("   ERROR: Failed to initialize AS5048A: %d\n", ret);
	return -1;
}
printk("   [OK] AS5048A ready\n\n");

/* Initialize BLDC 6PWM Driver */
printk("2. Initializing 6PWM driver...\n");
bldc_driver_6pwm_init_struct(&driver, 
                             PWM_AH_PIN, PWM_AL_PIN,
                             PWM_BH_PIN, PWM_BL_PIN,
                             PWM_CH_PIN, PWM_CL_PIN,
                             ENABLE_PIN);

/* Configure driver parameters */
driver.pwm_frequency = 25000;  /* 25 kHz PWM */
driver.voltage_power_supply = 5.0f;  /* 5V supply */
driver.voltage_limit = 5.0f;
driver.dead_zone = 0.02f;  /* 2% dead time */

ret = bldc_driver_6pwm_init_hw(&driver);
if (ret != DRIVER_INIT_OK) {
	printk("   ERROR: Failed to initialize driver\n");
	return -1;
}
printk("   [OK] Driver initialized\n\n");

/* Initialize BLDC Motor */
printk("3. Initializing BLDC motor...\n");
bldc_motor_init_struct(&motor, 
                       MOTOR_POLE_PAIRS,
                       MOTOR_PHASE_RESISTANCE,
                       MOTOR_KV_RATING,
                       MOTOR_INDUCTANCE);

/* Link driver to motor */
bldc_motor_link_driver(&motor, (bldc_driver_t*)&driver);

/* Link sensor to motor - use dummy pointer, actual access via g_as5048a */
bldc_motor_link_sensor(&motor, (sensor_t*)&encoder);

/* Configure motor parameters */
motor.voltage_limit = 5.0f;  /* Limit voltage for safety */
motor.velocity_limit = 20.0f;  /* rad/s */
motor.voltage_sensor_align = 3.0f;

ret = bldc_motor_init(&motor);
if (!ret) {
	printk("   ERROR: Failed to initialize motor\n");
	return -1;
}
printk("   [OK] Motor initialized\n\n");

/* Run FOC calibration */
printk("4. Running FOC calibration...\n");
printk("   This will align sensor and motor phases\n");
printk("   Motor will move slightly during calibration\n");
k_msleep(1000);

ret = bldc_motor_init_foc(&motor);
if (!ret) {
	printk("   ERROR: FOC calibration failed\n");
	printk("   Motor status: %d\n", motor.motor_status);
	return -1;
}
printk("   [OK] FOC calibration complete!\n\n");

printk("================================================\n");
printk("  System Ready - Motor Status: %d\n", motor.motor_status);
printk("  Entering main control loop...\n");
printk("================================================\n\n");

/* Set motor to torque control mode (voltage) */
motor.controller = TORQUE;
motor.target = 0.0f;  /* Start with zero torque */

printk("Starting motor control in 2 seconds...\n");
k_msleep(2000);

/* Main control loop */
while (1) {
	uint16_t raw_angle;
	float angle_deg, angle_rad;
	
	/* Read encoder */
	if (as5048a_read_raw(&encoder, &raw_angle) == 0) {
		angle_deg = ((float)raw_angle / 16384.0f) * 360.0f;
		angle_rad = ((float)raw_angle / 16384.0f) * 2.0f * 3.14159265f;
		
		/* SimpleFOC style: move() first to calculate voltages, then loopFOC() to apply them */
		bldc_motor_move(&motor, 2.0f);  /* Pure voltage control: 2V on q-axis */
		bldc_motor_loop_foc(&motor);  /* Apply calculated voltages */
		
		/* Print status */
		printk("Encoder: %5u | Angle: %6.2f° | Vel: %.2f | Vq: %.2fV | Vd: %.2fV\n",
		       raw_angle, (double)angle_deg,
		       (double)motor.shaft_velocity,
		       (double)motor.voltage.q, (double)motor.voltage.d);
	}
	
	/* Small delay for monitoring, FOC should run as fast as possible */
	k_msleep(1);
}
}
