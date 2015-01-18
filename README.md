# carproject
Embedded car racing project at Aalto University.

Usage

  When car is powered on, it displays initialization messages on the LCD screen.
  When everything is ready, display shows "Drive Test." "Press button..."
  Drive test can be started by pressing button. Car starts driving
  within ~one second after button press. 

  Test ends automatically after 90 seconds. Test can be cancelled at any
  time by pressing button. Test will also stop if car loses track for
  several seconds.
  When test ends, display shows "*** END ***" "Press button ...".
  Test can be restarted by following the instructions on the LCD display.

  During drive test LCD display shows some diagnostics.
  First line:  "005: d-2 w03"    (= remaining test time, stripe position, stripe width)
  Second line: "Speed 40 cm/s"   (= current speed reported by tachometer)

  There are several tests to verify hardware drivers. Tests can be enabled from main.c 
  and are not covered by this document.


Algorithm description

  Driving algorithm is very simple. It constantly tracks stripe position and
  changes car direction when stripe is not in the middle.

  Speed is controlled every 30 ms. Speed is faster when stripe is on the middle,
  and slows down relative to stripe position.
  Speed is also reduced when stripe is wide (we're not going to stripe direction),
  when stripe is lost and when stripe position moves fast.

  Driving algorithm uses following functions:

  control the car:
    car_set_speed()  : set car speed (in cm/s), maintained by tacometer HW driver
    car_set_dir()    : set car direction (in degrees)

  gather information about track:
    car_stripe_pos()  : current stripe position, -7...7, 0 = center
    car_stripe_width(): current stripe width (number of active sensors)

  user interaction:
    car_button_pressed()
    car_lcd_printf()
    car_lcd_message()
    car_current_speed()

    sleep_ms()
    car_time_ms()


Implementaton

Implementation is divided to three layers:

  Hardware drivers
    Hardware drivers handle all interaction with hardware.
    Interface hides ports, interrupts, delays and hardware settings.

  Hardware abstraction layer
    This layer converts all hardware-specific units to "real" units
    and hides details of hardware (like LCD control bytes, ...).
    Layer provides useful, simple to use functions for car controller.

  Car controller
    Car controller drives the car. It uses information provided by
    sensors and adjust speed and direction based on those.
    It also handles user interaction (start/stop tests, display diagnostics).

Source files:
  hw.[ch]   hardware drivers
  hwdefs.h  hardware definitions
  hal.[ch]  hardware abstraction layer
  main.c    car controller and tests
