/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 * HW interface
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020U; Peng Liu 280600
 * Date:   2012/03/16
 */

#ifndef __HW_H_
#define __HW_H_

#include <inttypes.h>

/*
 * Initialize hardware
 */

void hw_init(void);

/*
 * current time
 */

/* get current time in ticks */
uint32_t hw_time_ticks(void);

/*
 * user interface
 */

int  hw_button_state(void);   /* returns 1 if button is pressed down */ 
void hw_led_set(int state);   /* set LED on or off */

void hw_lcd_byte(uint8_t c);  /* send data byte to LCD display */
void hw_lcd_ctrl(uint8_t c);  /* send control byte to LCD display */

/*
 * IR sensors
 */

/* get current sensors state as bitmask */
uint8_t hw_ir_state(void);

/*
 * servo
 */

/* set servo PWM pulse width (us) */
void hw_servo_set(uint32_t pulse_width_us);

/*
 * motor
 */

/* set speed and direction */
/* speed:  direction and initial power (PWM duyty), -255...255        */
/* pulses: target speed, tacometer ticks in TM_PERIOID_MS             */
/* system tries to maintain target_speed */
void     hw_motor_speed(int16_t speed, uint16_t pulses);

/* get current speed (tacometer ticks / TM_PERIOID_MS) */
uint16_t hw_current_speed(void);

#ifdef DEBUG
uint8_t  hw_motor_pwr(void); /* current pwm duty, 0..255 */
#endif


#endif /* __HW_H_ */
