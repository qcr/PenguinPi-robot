#ifndef __global_h__
#define __global_h__
#define  LED_DEBUG

#ifdef  LED_DEBUG
  #define LED_DEBUG_R(duration) \
              leds[RED].state = 1;\
              leds[RED].count = duration;
  #define LED_DEBUG_G(duration) \
              leds[GREEN].state = 1;\
              leds[GREEN].count = duration;
  #define LED_DEBUG_B(duration) \
              leds[BLUE].state = 1;\
              leds[BLUE].count = duration;
#else
  #define   LED_DEBUG_R(duration)
  #define   LED_DEBUG_G(duration)
  #define   LED_DEBUG_B(duration)
#endif

// forward defines
void errmessage(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debugmessage(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));


// globals
extern Motor       motorA;
extern Motor       motorB;
extern LED         leds[NLEDS];
extern AnalogIn    vdiv;
extern AnalogIn    csense;
extern uint8_t    pid_on;
extern float pid_dt;

//
//HAT dependant

#include "timer.h"
extern stats_t loop_time;

// refactor into hat1.h, hat2.h etc.
//
#endif
