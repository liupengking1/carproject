/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 * HW interface definitions
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020U; Peng Liu 280600
 * Date:   2012/03/16
 */

#ifndef __HWDEFS_H_
#define __HWDEFS_H_

#include <avr/io.h>  // AVR IO definitions

/**********************************************
 * timings
 **********************************************/

#define SYSTEM_CLOCK     16000000

/* timer interrupt rate divisor */
#define TIMER_DIVISOR    (8*256)   // 1:8 clock, 8-bit counter

/* => system time tick is 128 us */
#define TICK_US        (1000000*TIMER_DIVISOR/SYSTEM_CLOCK)

/* tacometer sampling perioid 33 ms */
#define TM_PERIOID_MS  (33)

/**********************************************
 *
 **********************************************/

#define MOTOR_POWER_MIN     (35)  /* min. power (pwm duty) motor works with */
#define MOTOR_POWER_LIMIT  (120)  /* limit max power for motor */

/**********************************************
 * ports and pins
 **********************************************/

/* User interface */

/* Button in PB3 */
#define BUTTON_PORT_OUT  PORTB
#define BUTTON_PORT_IN   PINB
#define BUTTON_PORT_DDR  DDRB
#define BUTTON_PORT_MASK (1<<3)

/* Led in PB1 */
#define LED_PORT_OUT    PORTB
#define LED_PORT_DDR    DDRB
#define LED_PORT_MASK   (1<<1)

/* Servo in PD4 */
#define SERVO_PORT_OUT  PORTD
#define SERVO_PORT_DDR  DDRD
#define SERVO_PORT_MASK (1<<4)

/* LCD display in PC2...PC7 */
#define LCD_PORT_OUT   PORTC  
#define LCD_PORT_DDR   DDRC
#define LCD_PORT_MASK  (0xfc)
#define LCD_RS_MASK    (1<<2)
#define LCD_EN_MASK    (1<<3)
#define LCD_DATA_SHIFT 4
#define LCD_DATA_MASK  0xf0

/* Motor */

/* Motor diagnostic in PB5,PB6 */
#define MOTOR_DIAG_IN   PINB
#define MOTOR_DIAG_OUT  PORTB
#define MOTOR_DIAG_DDR  DDRB
#define MOTOR_DIAG_MASK ((1<<5)|(1<<6))

/* Motor PWM in PD7 */
#define MOTOR_PWM_OUT   PORTD
#define MOTOR_PWM_DDR   DDRD
#define MOTOR_PWM_MASK  (1<<7)

/* Motor control in PB4,PB7 */
#define MOTOR_CTRL_IN   PINB
#define MOTOR_CTRL_OUT  PORTB
#define MOTOR_CTRL_DDR  DDRB
#define MOTOR_CTRL_MASK ((1<<4)|(1<<7))

#define MOTOR_CTRL_BRAKE_VCC    ((1<<4)|(1<<7))
#define MOTOR_CTRL_CW           (1<<4)
#define MOTOR_CTRL_CCW          (1<<7)
#define MOTOR_CTRL_BRAKE_GND    0

/* tachometer */

/* tachometer in PB0,PD2,PD3,PD6 */
#define MOTOR_TM_DDR1   DDRB
#define MOTOR_TM_DDR2   DDRD
#define MOTOR_TM_OUT1   PORTB
#define MOTOR_TM_OUT2   PORTD
#define MOTOR_TM_MASK1  (1<<0)
#define MOTOR_TM_MASK2  ((1<<2)|(1<<3)|(1<<6))

/* IR sensors */

/* IR sensors in PA0...PA7 */
#define SENSOR_PORT_OUT  PORTA
#define SENSOR_PORT_IN   PINA
#define SENSOR_PORT_DDR  DDRA
#define SENSOR_PORT_MASK ((uint8_t)0xff)

#endif /* __HWDEFS_H_ */
