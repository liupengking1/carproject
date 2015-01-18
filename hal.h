/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 * HW abstraction layer
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020U; Peng Liu 280600
 * Date:   2012/03/16
 */

#ifndef __HAL_H_
#define __HAL_H_

#include <inttypes.h>

/*
 * Initialize hardware
 */

void car_init(void);
void car_reset(void);

/*
 * current time and sleep (accuracy ~ 128 us)
 */

uint32_t car_time_ms(void);
void     sleep_us(uint32_t us);

#define sleep_ms(ms) sleep_us(ms * 1000ul)

/*
 * User interface
 */

void car_wait_button_pressed(void);
int  car_button_down(void);

void car_set_led(int on);
void car_toggle_led(void);

void car_lcd_clear(void);
void car_lcd_goto(uint8_t line, uint8_t col);
void car_lcd_string(const char *s);
void car_lcd_printf(const char *fmt, ...) __attribute__((format (printf, 1, 2)));

/* Show message on LCD display.
 * s1      text for first line (NULL = none)
 * s2      text for second line (NULL = none).
 * time_ms display time. If < 0 wait for button press.
 * If s2 == NULL and time_ms < 0, second line is "press button ..."
 */
void car_lcd_message(const char *s1, const char *s2, int time_ms);

/*
 * Speed
 */

void    car_stop(void);
void    car_set_speed(int16_t speed); /* +/-, cm/s */
int16_t car_current_speed(void);      /* cm/s */

/*
 * Stripe position
 */

#define SENSOR_OFF_TRACK   -127
int8_t car_sensor_pos(void);     /* stripe position: -7...-1: left, 0: center, 1...7: right */
int8_t car_sensor_pos_fd(void);  /* -,,-, filtered version */
int    car_sensor_width(void);   /* stripe width: how many sensors are active */

//int8_t car_sensor_dpos(void);

/*
 * Car direction
 */

void car_set_direction(int8_t angle); /* angle in degrees, 0 = center */


#endif /* __HAL_H_ */
