#ifndef HAPTIC_H
#define HAPTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "drivers/bldc_motor.h"
#include "drivers/bldc_driver_6pwm.h"

int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder);
void haptic_loop(bldc_motor_t *motor);
int haptic_update_num_steps_from_button(void);



#endif /* HAPTIC_H */
