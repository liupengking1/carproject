/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020U; Peng Liu 280600
 * Date:   2012/03/16
 */

#include <stdio.h>

//#define DEBUG

#include "hal.h"
#include "util.h"

#define DRIVE_TEST_TIME  120  /* drive 120 seconds, then stop the car */

/* stop the car */
static void _stop(void)
{
  car_lcd_clear();

  car_lcd_goto(0,0);
  car_lcd_string("stopping... ");

  car_stop();
  car_lcd_string("stopped.    ");
}

/*
 * TESTS
 */

/* blink led 4 times (1 sec) */
#ifdef DEBUG
static void _blink(void)
{
  int n;
  car_set_led(1);
  for(n = 0; n < 8; n++) {
    sleep_ms(500);
    car_toggle_led();
  }
}
#endif

#ifdef DEBUG
static void _servo_test(void)
{
  while (1) {

    car_toggle_led();
    car_wait_button_pressed();

    car_toggle_led();
    car_wait_button_pressed();

    car_toggle_led();
    car_set_direction(-120);
    car_wait_button_pressed();

    car_toggle_led();
    car_set_direction(-40);
    car_wait_button_pressed();

    car_toggle_led();
    car_set_direction(0);
    car_wait_button_pressed();

    car_toggle_led();
    car_set_direction(40);
    car_wait_button_pressed();

    car_toggle_led();
    car_set_direction(120);
    car_wait_button_pressed();

    car_reset();
  }
}
#endif

#ifdef DEBUG
#include "hw.h"
void test_speed_meter(void)
{
  int speed = 0, loop = 0, inc = 10;

  message("speed meter test", NULL, 0);

  car_set_speed(10); /* 0.10 m/s */
  car_lcd_goto(0,0);
  car_lcd_string("10 ");

  while (1) {
    int current_speed = car_current_speed();
    uint8_t pwr = hw_motor_pwr();
    car_lcd_goto(1,0);
    car_lcd_printf("speed %3d (%3d) ", current_speed, pwr);

#if 1 
    /* speed follows stripe position */
    static int last_pos = -127;
    int pos = car_sensor_pos_fd();
    if (pos != last_pos) {
      if (pos < -7 || pos > 7)
        car_set_speed(0);
      else if (pos < 0)
        car_set_speed(pos * 10 - 10);
      else
        car_set_speed(pos * 10 + 10);
    }
    last_pos = pos;
#endif
#if 0
    /* cycle thru several different speeds */
    loop++;
    if (loop >= 5*10) {
      loop = 0;
      speed += inc;
      car_set_speed(speed);
      car_lcd_goto(0,0);
      car_lcd_printf("%2d ", speed);
      if (speed > 100) {
        speed = -10;
        inc = -10;
      }
    }
#endif

    if (car_button_down()) {
      break;
    }

    sleep_ms(100);
  }

  _stop();

  sleep_ms(2000);
}
#endif

/*
 * CAR LOGIC
 */

/* set car direction based on stripe position */
void set_dir(void)
{
  static const int8_t pos2angle[] = {
    -50, -40, -30, -20, -13,  -7,  -3,
    0,
    3,    7,   13,  20,  30,  40,  50 };

  /* current stripe position */
  int8_t pos = car_sensor_pos();

if (ABS(pos) > 3 && car_sensor_width() > 2)
pos = pos + (pos < 0 ? -1 : 1);

  /* position changed ? */
  static int8_t prev_pos = SENSOR_OFF_TRACK;
  if (pos != prev_pos) {

    if (pos >= -7 && pos <= 7) {

      if (0 && car_sensor_width() > 4) {
        car_set_direction(prev_pos < 0 ? 30 : -30);
      } else {
	pos=(pos+prev_pos)/2;
	prev_pos = pos;
        car_set_direction(pos2angle[pos+7]);
      }
    } else {
      car_set_speed(30);
    }
    prev_pos = pos;
  }
}

/* run track */
#define SPEED_ADJUST_TIMER   30 /* adjust speed once in 30ms */
static void _drive_test(uint32_t sec)
{
  uint32_t now       = car_time_ms();
  uint32_t stop_time = now + sec*1000;

  int slow_start = 10;             /* delayed speedup at start */
  int last_dir = SENSOR_OFF_TRACK; /* used to track stripe position change speed */
  int display_speed_timer = 0;     /* update speed display once in 200 ms */
  int off_track_timer = 0;         /* how long we have been off track */

  static int8_t prev_pos = 0;

  car_lcd_message("Drive test", NULL, -1);

int return_t = 0;
#warning better name ?

  while (now < stop_time) {

    /*
     * track stripe as fast as we can
     */
    uint32_t speed_adjust_time = now + SPEED_ADJUST_TIMER;
    do {
      now = car_time_ms();
      set_dir();
    } while (now < speed_adjust_time);

    /*
     * more complex actions once in 30 ms
     * - detect if we are off track
     * - adjust speed
     */

    /* current stripe state */
    int dir = car_sensor_pos();

    /* output diagnostics */
    car_lcd_goto(0,0);
    car_lcd_printf("%03d: d%2d w%2x ", (int)((stop_time - now)/1000), dir, car_sensor_width());
    if (display_speed_timer++ > 4) {
      car_lcd_goto(1,0);
      int speed = car_current_speed();
      car_lcd_printf("speed %3d   ", speed);
      display_speed_timer = 0;
    }
    /* abort ? */
    if (car_button_down()) {
        car_stop();
        car_lcd_message("*** STOPPED ***", NULL, -1);
        return;
    }

    /* off track ? */
    if (dir == SENSOR_OFF_TRACK) {
      /* delay action 150ms */
      off_track_timer += SPEED_ADJUST_TIMER;
      if (off_track_timer < 150)
	continue;

      /* slow down */
      car_set_speed(30);

      /* try to find stripe from the direction it was last seen */
      if (last_dir != SENSOR_OFF_TRACK) {
        if (last_dir < -1)
          car_set_direction(-50);
        else if (last_dir > 1)
          car_set_direction(50);
      }

      /* if stripe is not found in 6 seconds, stop the car */
      if (off_track_timer > 6000) {
        car_stop();
        car_lcd_message("Lost track.", NULL, -1);
        return;
      }
      continue;
    }
    off_track_timer = 0;

    /* Adjust speed: drive faster when stripe is in the middle. */
    int speed = 125;	
    switch (dir) {
      case -7: speed -= 60; return_t=7; break;
      case -6: speed -= 58; return_t=6; break;
      case -5: speed -= 56; return_t=5; break;
      case -4: speed -= 50; return_t=2; break;
      case -3: speed -= 40; return_t=1; break;
      case -2: if(return_t!=0) {return_t--;speed-=20;} break;
      case -1: if(return_t!=0) {return_t--;speed-=20;} break;
      case  0: if(return_t!=0) {return_t--;speed-=20;} break;
      case  1: if(return_t!=0) {return_t--;speed-=20;}break;
      case  2: if(return_t!=0) {return_t--;speed-=20;}break;
      case  3: speed -= 40; return_t=1; break;
      case  4: speed -= 50; return_t=2; break;
      case  5: speed -= 56; return_t=5; break;
      case  6: speed -= 58; return_t=6; break;
      case  7: speed -= 60; return_t=7; break;
    }

    /* slow start */
    if (slow_start > 0) {
     speed = SATURATE(speed - slow_start*10, 0, 50);
      slow_start--;
    }

    /* fast stripe position movement -> slow down */
#if 0
    if (ABS(dir - last_dir) > 4) {
      speed -= 20;
    }
#endif
    /* large stripe angle -> slow down */
    if (car_sensor_width() > 6) {
#if 0
      speed = 0;
      car_stop();
#else
      speed = 20;
#endif
    } else if (car_sensor_width() > 3) {
       speed = 30;
    } else if (car_sensor_width() > 2) {
      speed = MIN(speed, 60);
    }

    /* set new speed */
    car_set_speed(MAX(60, speed));

    last_dir = dir;
  }

  _stop();
}


int main(void)
{
  car_init();

#ifdef DEBUG
  _blink();
#endif

  sleep_ms(1000);
  car_lcd_message("Ready", NULL, 1000);

  while (1) {

#ifdef DEBUG
    _test_speed_meter();
    sleep_ms(1000);
    _servo_test();
#endif

    car_set_direction(0);
    sleep_ms(1000);

    _drive_test(DRIVE_TEST_TIME);

    sleep_ms(1000);
    car_lcd_message("*** END ***", NULL, -1);
  }

  return 0;
} 

