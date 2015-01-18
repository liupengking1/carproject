/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 * Hardware drivers
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020U; Peng Liu 280600
 * Date:   2012/03/16
 */

#include <avr/interrupt.h>

#include "hw.h"
#include "hwdefs.h"
#include "util.h"

/**********************************************
 * macros
 **********************************************/

#define SET_BIT_HI(port, mask) port |= (mask)
#define SET_BIT_LO(port, mask) port &= (~(mask))

#define nop()   __asm__ __volatile__("nop"::)

static void _adjust_speed(void);

/**********************************************
 * Current time
 **********************************************/

/*
 * Motor PWM timer overflow interrupt is used to maintain current time and adjust speed
 */

volatile uint32_t ticks;  /* current system time */

ISR(TIMER2_OVF_vect)
{
  /* update current time */
  ticks++;

  /* run motor control */
#if TM_PERIOID_MS == 65 && TICK_US == 128
  /* once in 65 ms (once in 512 ticks) */
  if ((ticks & 0x1fful) == 0) {
    _adjust_speed();
  }
#elif TM_PERIOID_MS == 33 && TICK_US == 128
  /* once in 33 ms (once in 256 ticks) */
  if ((ticks & 0xfful) == 0) {
    _adjust_speed();
  }
#else
#  error timings do not match
#endif
}

uint32_t hw_time_ticks(void)
{
  uint32_t result;

  cli();
  result = ticks;
  sei();

  return result;
}

static void _hw_sleep_tick(void)
{
  /* wait at least one tick (=> 128...256 us) */
  uint32_t t;

  t = hw_time_ticks();
  while (hw_time_ticks() == t);

  t = hw_time_ticks();
  while (hw_time_ticks() == t);
}

/**********************************************
 * LCD display
 **********************************************/

#define LCD_BIT_HI(mask) SET_BIT_HI(LCD_PORT_OUT, (mask))
#define LCD_BIT_LO(mask) SET_BIT_LO(LCD_PORT_OUT, (mask))

/* clock LCD bus */
static void _lcd_clock(void)
{
  _hw_sleep_tick();

  cli();
  LCD_BIT_HI(LCD_EN_MASK);
  sei();

  _hw_sleep_tick();

  cli();
  LCD_BIT_LO(LCD_EN_MASK);
  sei();

  _hw_sleep_tick();
}

/* send 4 bit nibble to LCD */
static void _lcd_send_nibble(uint8_t data, int command)
{
  cli();
  LCD_BIT_LO(LCD_PORT_MASK);
  LCD_PORT_OUT |= ((data << LCD_DATA_SHIFT) & LCD_DATA_MASK) | (command ? 0 : LCD_RS_MASK);
  sei();

  _lcd_clock();
}

static void _lcd_send(uint8_t data, int command)
{
  _lcd_send_nibble(data>>4, command);
  _lcd_send_nibble(data,    command);
}

void hw_lcd_byte(uint8_t c)
{
  /* send data byte */
  _lcd_send(c, 0);
}

void hw_lcd_ctrl(uint8_t c)
{
  /* send control byte */
  _lcd_send(c, 1);
}

/**********************************************
 * button
 **********************************************/

int hw_button_state(void)
{
  return !(BUTTON_PORT_IN & BUTTON_PORT_MASK);
}

/**********************************************
 * LED
 **********************************************/

void hw_led_set(int state)
{
  cli();

  if (state) {
    LED_PORT_OUT |= LED_PORT_MASK;
  } else {
    LED_PORT_OUT &= ~LED_PORT_MASK;
  }

  sei();
}

/**********************************************
 * Servo
 **********************************************/

/*
 * servo: 20ms pulse perioid
 *   0.75 ... 2.25 ms pulse width: -85...+85 degrees
 * The servo is connected to the B output of the Timer/Counter1
 * subsystem (Port D pin 4 , OC1B)
 */

const    int servo_pwm_done_at = 3*50; /* PWM active for 3 sec (20ms*50*3) */
volatile int servo_pwm_count   = 0;    /* PWM active timer counter */

/* called once in 20 ms while servo PWM is active */
ISR(TIMER1_COMPB_vect)
{
  /* turn servo off after 3 seconds */

  if (++servo_pwm_count > servo_pwm_done_at) {
    /* disable output */
    TCCR1A &= ~(1<<COM1B1);

    servo_pwm_count = 0;
  }
}

void hw_servo_set(uint32_t pulse_width_us)
{
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  /* COM1A1: Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM 
   * WGM:    fast pwm, mode 14
   * CS:     1/8 clk io
   */
  TCCR1A |= (1<<COM1B1) | (1<<WGM11);
  TCCR1B |= /*(1<<CS10) |*/ (1<<CS11) | (1<<WGM13) | (1<<WGM12);

  /* 20 ms (50 Hz) pulse perioid */
  ICR1  = SYSTEM_CLOCK/(8*50);

  /* pulse width */
  OCR1B = pulse_width_us * (SYSTEM_CLOCK/1000) / (8 * 1000);

  /* reset PWM active counter */
  servo_pwm_count = 0;

  /* enable compare match interrupt */
  sei();
  TIMSK |= (1 << OCIE1B);
}

/**********************************************
 * Tacometer
 **********************************************/

/* current speed and requested speed */
volatile uint16_t current_speed = 0;   /* tacometer ticks / TM_SAMPLING_PERIOID */
volatile uint16_t target_speed  = 0;   /* -,,- */

/* count tacometer counter overflows */
static uint16_t   tm_overflows  = 0;   /* used only in interrupt handlers */

ISR(TIMER0_OVF_vect)
{
  tm_overflows++;
}

static uint32_t _tacometer_ticks(int reset)
{
  uint32_t cnt;

  cnt = TCNT0 + (tm_overflows << 8);

  if (reset) {
    TCNT0 = 0;
    tm_overflows = 0;
  }

  return cnt;
}

#ifdef PRESET_POWER
uint16_t initial_power = 0;
#endif

static void _adjust_speed(void)
{
  /* Called from interrupt handler once in 33 ms
   *
   * Read tacometer and maintain current_speed.
   * Adjust motor power to keep requested speed.
   */

  static uint8_t skip_adjust_cnt = 0;
#if 0
  static int16_t change = 0;
#endif

  /* read and reset tachometer counter */
  current_speed = _tacometer_ticks(1);

  /* Requested speed changed ? */
  static uint8_t prev_target_speed = 0;
  if (target_speed != prev_target_speed) {
#if 0
    change += target_speed - prev_target_speed;
#endif
    prev_target_speed = target_speed;
#ifdef PRESET_POWER
    if (target_speed == 0)
      OCR2 = 1;
    return;
#endif
  }

  /* stop ? */
  if (target_speed == 0) {
    OCR2 = 1;
    return;
  }

  /* wait for stabilization ? */
  if (skip_adjust_cnt > 0) {
    skip_adjust_cnt--;
    return;
  }

  /* adjust power (maintain speed) */
  int16_t diff = target_speed - current_speed;
#if 0
  /* limit change step to requested change */
  if (change > 10) {
    diff = MIN(diff, change);
  } else if (change < -10) {
    diff = MAX(diff, change);
  }
  change = diff;
#endif
  /* ignore minor changes */
  if (diff > -5 && diff < 3) {
    return;
  }

  int16_t pwr = OCR2;

  if (diff < 0) {
    /* brake faster */
    if (diff < -50)
      pwr -= 15;
    else if (diff < -20)
      pwr -= 7;
    else if (diff < 0)
      pwr -= 3;
  } else {
    /* speed up slower */
    if (diff > 30)
      pwr += 6;
    else if (diff > 20)
      pwr += 2;
    else if (diff > 0)
      pwr += 1;
  }

  /* make sure power is in sane range:
   * - smallest possible speed: motor stops if power is too low
   * - limit max. power
   */
  pwr = SATURATE(pwr, MOTOR_POWER_MIN, MOTOR_POWER_LIMIT);

  /* large changes need some time */
  if (diff < -20 || diff > 20)
    skip_adjust_cnt = 3;

  /* reprog pwm */
  OCR2 = pwr;
}

static void _tacometer_init(void)
{
  cli();

  TCCR0 = 0;

  /* WGM: mode 3, fast pwm
   * COM: Clear OC2 on compare match, set on bottom
   * CS: external clock on T0, rising edge
   */
  TCCR0 = /*(1<<WGM01) | (1<<WGM00) | (1<<COM01) |*/ (1<<CS02) | (1<<CS01) /*| (1<<CS00)*/;
  OCR0  = 0;

  sei();

  TIMSK |= (1 << TOIE0); // Enable overflow interrupt
}

uint16_t hw_current_speed(void)
{
  uint16_t s;
  cli();
  s = current_speed;
  sei();
  return s;
}

/**********************************************
 * Motor
 **********************************************/

static void _motor_pwm_init(void)
{
  cli();

  /* WGM: mode 3, fast pwm
   * COM: Clear OC2 on compare match, set on bottom
   * CS: 1:8 CLKt2s
   *
   * => 7812 Hz PWM signal
   */
  TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) /*| (1<<CS22) */| (1<<CS21) /*| (1<<CS20)*/;
  OCR2  = 1; /* pulse width, 0...0xff */

  sei();

  /* overflow interrupt is used to generate system time and adjust speed.
   * timer should be always running, even if there is no output.
   */

  TIMSK |= (1 << TOIE2); // Enable overflow interrupt. used for system time.
}

static void _motor_set_dir(uint8_t dir)
{
  cli();
  MOTOR_CTRL_OUT = (MOTOR_CTRL_OUT & (~MOTOR_CTRL_MASK)) | dir;
  sei();
}

static void _motor_set_speed(uint8_t speed, uint16_t pulses)
{
  if (pulses != target_speed) {

    /* limit max power to 1/3 */
    if (speed > 255/3)
      speed = 255/3;

    cli();

    /* pre set power */
    if (pulses < 2 || speed == 0)
      OCR2 = 1;
#ifdef PRESET_POWER
    else if (speed > 20 && OCR2 < 20)
      OCR2 = speed;
    initial_power = speed;
#endif

    /* requested speed is maintained by timer interrupt handler, based on tacometer feedback */
    target_speed = pulses;
    sei();
  }
}

void hw_motor_speed(int16_t speed, uint16_t pulses)
{
  uint8_t dir, abs_speed;

  if (speed < 0) {
    dir = MOTOR_CTRL_CCW;
    abs_speed = -speed;

  } else if (speed == 0) {
    dir = MOTOR_CTRL_BRAKE_GND;
    abs_speed = 1;

  } else /* if (speed > 0) */ {
    dir = MOTOR_CTRL_CW;
    abs_speed = speed;
  }

  uint8_t current_dir = MOTOR_CTRL_OUT & MOTOR_CTRL_MASK;
  if (dir != current_dir) {
    /* need to stop before changing direction ... ? */
    abs_speed = 1;
  }
 
  _motor_set_dir(dir);
  _motor_set_speed(abs_speed, pulses);
}

uint8_t hw_motor_pwr(void)
{
  return OCR2;
}

/**********************************************
 * Sensors
 **********************************************/

uint8_t hw_ir_state(void)
{
  return SENSOR_PORT_IN & SENSOR_PORT_MASK;
}

/**********************************************
 * Initialize hardware
 **********************************************/

void hw_init(void)
{
  /* button */
  BUTTON_PORT_DDR &= ~BUTTON_PORT_MASK; /* set pin to input */
  BUTTON_PORT_OUT |= BUTTON_PORT_MASK;  /* pull-up resistor on */

  /* led */
  LED_PORT_DDR |= LED_PORT_MASK;        /* set pin to output */
  LED_PORT_OUT &= ~LED_PORT_MASK;       /* LED off */

  /* LCD display */
  LCD_PORT_DDR |= LCD_PORT_MASK;        /* set pin to output */
  LCD_PORT_OUT &= ~LCD_PORT_MASK;       /* output low */

  /* servo */
  SERVO_PORT_DDR |= SERVO_PORT_MASK;    /* set pin to output */
  SERVO_PORT_OUT &= ~SERVO_PORT_MASK;   /* output low */

  /* motor diagnostic */
  MOTOR_DIAG_DDR &= ~MOTOR_DIAG_MASK;   /* set pin to input */
  MOTOR_DIAG_OUT &= ~MOTOR_DIAG_MASK;   /* pull-up resistor off */

  /* motor PWM */
  MOTOR_PWM_DDR |= MOTOR_PWM_MASK;      /* set pin to output */
  MOTOR_PWM_OUT &= ~MOTOR_PWM_MASK;     /* output low */

  /* motor control */
  MOTOR_CTRL_DDR |= MOTOR_CTRL_MASK;    /* set pin to output */
  MOTOR_CTRL_OUT &= ~MOTOR_CTRL_MASK;   /* motor off */

  /* Motor tachometer */
  MOTOR_TM_DDR1 &= ~MOTOR_TM_MASK1;     /* set pins to input */
  MOTOR_TM_DDR2 &= ~MOTOR_TM_MASK2;     /* set pins to input */
  MOTOR_TM_OUT1 &= ~MOTOR_TM_MASK1;     /* pull-up resistors off */
  MOTOR_TM_OUT2 &= ~MOTOR_TM_MASK2;     /* pull-up resistors off */

  /* IR sensors */
  SENSOR_PORT_DDR = (uint8_t)~SENSOR_PORT_MASK;  /* set pins to input */
  SENSOR_PORT_OUT = (uint8_t)~SENSOR_PORT_MASK;  /* pull-up resistors off */

  /* current time */
  ticks = 0;

  /* start HW timers */
  _motor_pwm_init();
  _tacometer_init();
}

