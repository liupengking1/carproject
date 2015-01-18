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

#include <stdio.h>

#include "hw.h"
#include "hal.h"

#include "hwdefs.h" /* TICK_US */
#include "util.h"

/**********************************************
 * HW constants and limits
 **********************************************/

/* servo */

#define DIR_ANGLE_MIN      (-85)
#define DIR_ANGLE_MAX      ( 85)
#define DIR_ANGLE_CENTER     (0)  /* center correction */

#define DIR_PWM_MIN_US     (750)  /* 0.75 ms pulse, -85 deg */
#define DIR_PWM_MAX_US    (2250)  /* 2.25 ms pulse, +85 deg */ 

#define DIR_INVERT_ANGLE       0  /* 0 - normal, 1 - inverse angle */

/* tacometer, tires */

#define TM_PULSES_360      (544)  /* one rotation, 544 pulses */
#define TM_MM_360          (188)  /* distance travelled during one rotation, 188 mm */

/* speed */

#define CAR_SPEED_MIN     (-100)  /* -1.0 m/s */
#define CAR_SPEED_MAX      (150)  /*  1.5 m/s */
#define CAR_SPEED_SLOW      (30)  /* 0.40 m/s, slowest speed */

/**********************************************
 * current time
 **********************************************/

static uint64_t _time_us(void)
{
#if TICK_US == 128
  return ((uint64_t)hw_time_ticks()) << 7;
#else
  return hw_time_ticks() * UINT64_C(TICK_US);
#endif
}

uint32_t car_time_ms(void)
{
  /* return _hw_time_us() / 1000; */
  /* 2.4% error acceptable ... */
#if TICK_US == 128
  return hw_time_ticks() >> 3;
#else
  return _time_us() >> 10;
#endif
}


/**********************************************
 * Sleep
 **********************************************/

void sleep_us(uint32_t us)
{
#if TICK_US == 128
  uint32_t time_finished = hw_time_ticks() + (us >> 7) + 1;
  while (hw_time_ticks() < time_finished);
#else
  uint64_t time_finished = _time_us() + us;
  while (_time_us() < time_finished);
#endif
}


/**********************************************
 * button
 **********************************************/

void car_wait_button_pressed(void)
{
  while (!hw_button_state());
  while (hw_button_state());
}

int car_button_down(void)
{
  return hw_button_state();
}


/**********************************************
 * LED
 **********************************************/

static int led_state = 0;

void car_set_led(int on)
{
  led_state = on;
  hw_led_set(led_state);
}

void car_toggle_led(void)
{
  hw_led_set(!led_state);
}


/**********************************************
 * LCD
 **********************************************/

static void _lcd_init(void)
{
  sleep_us(50000);
  hw_lcd_ctrl(0x03);
  sleep_us(5000);
  hw_lcd_ctrl(0x03);
  sleep_us(5000);
  hw_lcd_ctrl(0x03);
  sleep_us(5000);
  hw_lcd_ctrl(0x02);

  sleep_us(5000);
  hw_lcd_ctrl(0x28);  // 4-bit bus

  sleep_us(500);
  hw_lcd_ctrl(0x08); // display off

  sleep_us(5000);
  hw_lcd_ctrl(0x0c); // display on, cursor on, blink

  sleep_us(5000);
  hw_lcd_ctrl(0x01); // clear

  sleep_us(5000);
  hw_lcd_ctrl(0x06); // cursor movement dir, no display shift

  sleep_us(5000);
  hw_lcd_ctrl(0x01); // clear display

  sleep_us(10000);
}

void car_lcd_clear(void)
{
  hw_lcd_ctrl(0x01); // clear

  sleep_us(5000);
}

void car_lcd_goto(uint8_t line, uint8_t col)
{
  if (!line)
    hw_lcd_ctrl(0x80 | col); // first byte of first line
  else
    hw_lcd_ctrl(0xc0 | col); // first byte of first line
}

void car_lcd_string(const char *s)
{
  while (*s) {
    hw_lcd_byte(*s);
    sleep_us(100);
    s++;
  }
}

void car_lcd_printf(const char *fmt, ...)
{
  va_list argp;
  char buf[64];

  va_start(argp, fmt);
  vsnprintf(buf, sizeof(buf), fmt, argp);
  buf[sizeof(buf)-1] = 0;

  car_lcd_string(buf);

  va_end(argp);
}

void car_lcd_message(const char *s1, const char *s2, int time_ms)
{
  car_lcd_clear();

  if (s1) {
    car_lcd_goto(0,0);
    car_lcd_string(s1);
  }
  if (s2) {
    car_lcd_goto(1,0);
    car_lcd_string(s2);
  }

  if (time_ms < 0) {
    if (!s2) {
      car_lcd_goto(1, 0);
      car_lcd_string("press button...");
    }
    car_wait_button_pressed();
  } else {
    sleep_ms(time_ms);
  }

  car_lcd_clear();
}

/**********************************************
 * IR sensors
 **********************************************/

int car_sensor_width(void)
{
  uint8_t s = hw_ir_state();
  int i, r = 0;
  for (i = 0; i < 8; i++)
    if (s & (1<<i))
      r++;
  return r;
}

static int8_t _car_sensor_pos(int filter)
{
  uint8_t s = hw_ir_state();
  int i;
  int rmin = -1, rmax = -1;

  static uint8_t last_s = 0;
  if (filter) {
    s |= last_s;
    last_s = hw_ir_state();
  }

  if (s == 0) {
    return SENSOR_OFF_TRACK;
  }

  /* lowest "1" bit position */
  for (i = 0; i < 8; i++) {
    if (s & (1<<i)) {
      rmin = i;
      break;
    }
  }
  /* highest "1" bit position */
  for (i = 7; i >= 0; i--) {
    if (s & (1<<i)) {
      rmax = i;
      break;
    }
  }

  /* average (center) position.
   * scale to -7 ... 7 
   */
  int pos = 2 * (rmin + (rmax - rmin + 1) / 2) + ((rmax - rmin)?-1:0) - 7;

  return pos;
}

int8_t car_sensor_pos(void)
{
  return _car_sensor_pos(0);
}

int8_t car_sensor_pos_fd(void)
{
  return _car_sensor_pos(1);
}

#if 0
int8_t car_sensor_dpos(void)
{
  static uint64_t prev_time = 0;
  static uint8_t  prev_pos = 0;

  uint64_t now = _time_us();
  uint8_t  pos = car_sensor_pos();

  if (now != prev_time && pos != prev_pos) {
    int d = (prev_pos - pos) / (now - prev_time);
    prev_time = now;
    prev_pos = pos;

    return d;
  }
  return 0;
}
#endif

/**********************************************
 * direction (angle in degrees)
 **********************************************/

void car_set_direction(int8_t angle)
{
  /* invert ? */
#if DIR_INVERT_ANGLE
  angle = -angle;
#endif 

  /* -85 .. +85 degrees */
  angle = SATURATE(angle, DIR_ANGLE_MIN, DIR_ANGLE_MAX);

  /* center correction */
  angle += DIR_ANGLE_CENTER;

  /* changed ? */
  static int8_t current_angle = 0;
  if (angle != current_angle) {
    /* convert degrees to pulse width (us) */
    uint32_t pw = angle - DIR_ANGLE_MIN; /* -> 0..170 */
    pw = DIR_PWM_MIN_US + pw * (DIR_PWM_MAX_US - DIR_PWM_MIN_US) / (DIR_ANGLE_MAX - DIR_ANGLE_MIN);
    hw_servo_set(pw);

    current_angle = angle;
  }
}


/**********************************************
 * speed (in cm/s)
 **********************************************/

int16_t car_current_speed(void)
{
  uint32_t pulses = hw_current_speed();
  uint32_t mm = pulses * TM_MM_360; mm /= TM_PULSES_360;
  //uint32_t cm = mm / 10;
  //uint32_t cm_s = cm * (1000/TM_PERIOID_MS);
  //cm_s = mm / 10 * (1000/TM_PERIOID_MS);
  uint32_t cm_s = mm * 100ul; cm_s /= TM_PERIOID_MS;

  return cm_s;
}

void car_set_speed(int16_t speed)
{
  /* check if changed */
  static int16_t prev_speed = 0;
  if (speed == prev_speed)
    return;
  prev_speed = speed;

  /* limit speed */
  speed = SATURATE(speed, CAR_SPEED_MIN, CAR_SPEED_MAX);

  /* slowest possible speed */
  uint16_t abs_speed = ABS(speed);
  abs_speed = MAX(abs_speed, CAR_SPEED_SLOW);

  /* cm/s -> tacometer pulses / 65ms */

  uint32_t s = abs_speed * TM_PERIOID_MS / 100ul;  // mm / perioid
  uint32_t pulses = s * TM_PULSES_360 / TM_MM_360;

#if 0
car_lcd_goto(0,8);
char str[32];
sprintf(str, "-> %lu ", pulses);
car_lcd_string(str);
#endif

  /* initial power */
  int16_t pwr = (abs_speed - CAR_SPEED_SLOW) * 3 / 10  + 40;
  if (speed < 0) pwr = -pwr;
  if (speed == 0) { pwr = abs_speed = 0; }
/*
{10  40}
{20  40}
{30  40}
{40  40}
{50  41}
{60  45}
{70  48}
{80  51}
{90  53}
{100 56}
{110 60}
*/

  hw_motor_speed(pwr, pulses);
}

void car_stop(void)
{
  car_set_speed(0);

  /* wait until stopped */
  do {
    sleep_ms(TM_PERIOID_MS);
  } while (hw_current_speed() > 1);
}


/**********************************************
 * initialization
 **********************************************/

void car_reset(void)
{
  hw_init();
  car_set_speed(0);
  car_set_direction(0);
}

void car_init(void)
{
  car_reset();

  _lcd_init();

  car_lcd_goto(0, 0);
  car_lcd_string("HW initialized");
}


