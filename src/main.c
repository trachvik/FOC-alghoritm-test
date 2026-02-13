#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <arm_math.h>

/* Get AS5048A SPI device from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB, 0);

/* PWM devices for TMC6300 motor driver */
/* TIM1 for Low sides, TIM3 for High sides - independent control */
#define PWM_LOW_NODE DT_NODELABEL(pwm_low)
#define PWM_HIGH_NODE DT_NODELABEL(pwm_high)
static const struct device *pwm_low = DEVICE_DT_GET(PWM_LOW_NODE);
static const struct device *pwm_high = DEVICE_DT_GET(PWM_HIGH_NODE);

/* PWM period (20kHz = 50us period) */
#define PWM_PERIOD_US 50

/* PWM channels - Low sides on TIM1, High sides on TIM3 */
#define PWM_CH_UL 1  /* TIM1_CH1 (PA8) */
#define PWM_CH_VL 2  /* TIM1_CH2 (PA9) */
#define PWM_CH_WL 3  /* TIM1_CH3 (PA10) */
#define PWM_CH_UH 2  /* TIM3_CH2 (PA7) */
#define PWM_CH_VH 3  /* TIM3_CH3 (PB0) */
#define PWM_CH_WH 4  /* TIM3_CH4 (PB1) */

/* AS5048A command to read angle (0xFFFF reads the angle register) */
#define AS5048A_READ_ANGLE 0xFFFF

/* 14-bit mask for angle data */
#define ANGLE_MASK 0x3FFF

/* Motor direction flag: 1 = normal, -1 = inverted */
static int motor_direction = 1;

/* Read raw encoder value (14-bit) */
uint16_t read_encoder_raw(void)
{
	uint16_t tx_data = AS5048A_READ_ANGLE;
	uint16_t rx_data = 0;

	const struct spi_buf tx_buf = {
		.buf = &tx_data,
		.len = sizeof(tx_data)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	const struct spi_buf rx_buf = {
		.buf = &rx_data,
		.len = sizeof(rx_data)
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	/* Perform SPI transaction */
	int ret = spi_transceive_dt(&as5048a_spi, &tx, &rx);
	if (ret < 0) {
		printk("SPI transaction failed: %d\n", ret);
		return 0;
	}

	/* Extract 14-bit angle data */
	return rx_data & ANGLE_MASK;
}

/* Set motor phase PWM with independent high/low side control */
/* For proper BLDC control: need return path through low side */
void set_motor_phase_hl(uint8_t uh, uint8_t ul, uint8_t vh, uint8_t vl, uint8_t wh, uint8_t wl)
{
	uint32_t period_ns = PWM_PERIOD_US * 1000;
	
	pwm_set(pwm_high, PWM_CH_UH, period_ns, (period_ns * uh) / 100, 0);
	pwm_set(pwm_low, PWM_CH_UL, period_ns, (period_ns * ul) / 100, 0);
	
	pwm_set(pwm_high, PWM_CH_VH, period_ns, (period_ns * vh) / 100, 0);
	pwm_set(pwm_low, PWM_CH_VL, period_ns, (period_ns * vl) / 100, 0);
	
	pwm_set(pwm_high, PWM_CH_WH, period_ns, (period_ns * wh) / 100, 0);
	pwm_set(pwm_low, PWM_CH_WL, period_ns, (period_ns * wl) / 100, 0);
}

/* Legacy function - sets only high sides (LOW sides all OFF) */
void set_motor_phase(uint8_t u_percent, uint8_t v_percent, uint8_t w_percent)
{
	set_motor_phase_hl(u_percent, 0, v_percent, 0, w_percent, 0);
	printk("PWM set: UH=%u%%, VH=%u%%, WH=%u%% (all LOW sides=0%%)\n", 
	       u_percent, v_percent, w_percent);
}

/* Calculate encoder difference with overflow handling */
int16_t encoder_diff(uint16_t pos1, uint16_t pos2)
{
	int32_t diff = (int32_t)pos2 - (int32_t)pos1;
	
	/* Handle overflow/underflow for 14-bit encoder (0-16383) */
	if (diff > 8192) {
		diff -= 16384;
	} else if (diff < -8192) {
		diff += 16384;
	}
	
	return (int16_t)diff;
}

/* Direction calibration function */
bool calibrate_motor_direction(void)
{
	uint16_t pos1, pos2;
	int16_t diff;
	
	printk("Starting motor direction calibration...\n");
	printk("Using proper 2-phase energization for BLDC\n");
	
	/* Set all phases to 0 initially */
	set_motor_phase_hl(0, 0, 0, 0, 0, 0);
	k_msleep(1000);
	
	/* Position 1: Current flows U->V (UH=50%, VL=100%, others off) */
	printk("Setting position 1: U high -> V low (current U to V)\n");
	set_motor_phase_hl(50, 0, 0, 100, 0, 0);  // UH on, VL on, others off
	k_msleep(1000);
	pos1 = read_encoder_raw();
	printk("Encoder position 1: %u\n", pos1);
	
	/* Position 2: Current flows U->W (UH=50%, WL=100%, others off) */
	printk("Setting position 2: U high -> W low (current U to W)\n");
	set_motor_phase_hl(50, 0, 0, 0, 0, 100);  // UH on, WL on, others off
	k_msleep(1000);
	pos2 = read_encoder_raw();
	printk("Encoder position 2: %u\n", pos2);
	
	/* Turn off motor */
	set_motor_phase_hl(0, 0, 0, 0, 0, 0);
	
	/* Calculate difference with overflow handling */
	diff = encoder_diff(pos1, pos2);
	printk("Position difference: %d\n", diff);
	
	/* If positive difference, direction is correct */
	if (diff > 0) {
		motor_direction = 1;
		printk("Direction: NORMAL (correct)\n");
		return true;
	} else if (diff < 0) {
		motor_direction = -1;
		printk("Direction: INVERTED (will be corrected in code)\n");
		return true;
	} else {
		printk("ERROR: No movement detected!\n");
		return false;
	}
}

float32_t read_as5048a_angle(void)
{
	uint16_t tx_data = AS5048A_READ_ANGLE;
	uint16_t rx_data = 0;
	uint16_t angle_raw;
	float32_t angle_degrees;

	const struct spi_buf tx_buf = {
		.buf = &tx_data,
		.len = sizeof(tx_data)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	const struct spi_buf rx_buf = {
		.buf = &rx_data,
		.len = sizeof(rx_data)
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	/* Perform SPI transaction */
	int ret = spi_transceive_dt(&as5048a_spi, &tx, &rx);
	if (ret < 0) {
		printk("SPI transaction failed: %d\n", ret);
		return -1.0f;
	}

	/* Extract 14-bit angle data */
	angle_raw = rx_data & ANGLE_MASK;

	/* Convert to degrees: (angle_raw * 360.0) / 8192.0 */
	angle_degrees = ((float32_t)angle_raw * 360.0f) / 8192.0f;

	return angle_degrees;
}

int main(void)
{
	float32_t angle;
	int ret;

	/* Enable USB device */
	ret = usb_enable(NULL);
	if (ret != 0) {
		printk("Failed to enable USB: %d\n", ret);
		return -1;
	}

	/* Wait for USB to be ready */
	k_msleep(1000);

	printk("USB CDC ACM initialized\n");
	printk("*** BLDC Motor Controller with TMC6300 ***\n");

	/* Check if SPI device is ready */
	if (!spi_is_ready_dt(&as5048a_spi)) {
		printk("AS5048A SPI device not ready\n");
		return -1;
	}

	printk("AS5048A sensor initialized\n");

	/* Check if PWM devices are ready */
	if (!device_is_ready(pwm_high) || !device_is_ready(pwm_low)) {
		printk("PWM devices not ready\n");
		return -1;
	}

	printk("PWM devices initialized (TIM1=Low sides, TIM3=High sides)\n");

	/* Simple PWM test before calibration */
	printk("\n=== PWM OUTPUT TEST ===\n");
	printk("Testing Phase U at 50%% for 2 seconds...\n");
	set_motor_phase(50, 0, 0);
	k_msleep(2000);
	set_motor_phase(0, 0, 0);
	printk("Test complete. Check if motor moved or if you can measure voltage on pins.\n");
	k_msleep(1000);

	/* Run direction calibration */
	printk("\n=== MOTOR DIRECTION CALIBRATION ===\n");
	if (!calibrate_motor_direction()) {
		printk("Calibration failed!\n");
		return -1;
	}
	printk("Calibration complete! Motor direction: %s\n\n", 
	       motor_direction == 1 ? "NORMAL" : "INVERTED");

	/* Main loop: read sensor and print angle */
	printk("Starting main loop...\n");
	while (1) {
		/* Read angle from sensor */
		angle = read_as5048a_angle();

		/* Print angle in degrees */
		//printk("Angle: %.2f degrees (Direction: %s)\n", 
		       //(double)angle, motor_direction == 1 ? "NORMAL" : "INVERTED");

		/* Wait 500ms before next reading */
		k_msleep(500);
	}

	return 0;
}