/* Helsinki University of Technology
 * T-106.530 Embedded Systems
 *
 * Car-project
 * HW abstraction layer
 *
 * Target: Atmel AVR ATmega32
 *
 * Author: Petri Hintukainen, 44020; Peng Liu 280600
 * Date:   2012/03/16
 */

#ifndef __CAR_UTIL_H_
#define __CAR_UTIL_H_

#define ABS(x)                  ((x) < 0 ? (-x) : (x))

#define MIN(a,b)                ((a) < (b) ? (a) : (b))
#define MAX(a,b)                ((a) > (b) ? (a) : (b))
#define SATURATE(val, min, max) ((val) > (max) ? (max) : (val) < (min) ? (min) : (val))

#endif /* __CAR_UTIL_H_ */
