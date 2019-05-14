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

#include <stdint.h>

#include "motor.h"
#include "io.h"
#include "timer.h"

// forward defines
void errmessage(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debugmessage(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

// globals
extern Motor       motorL;
extern Motor       motorR;
extern LED         leds[NLEDS];
extern AnalogIn    vdiv;
extern AnalogIn    csense;
extern uint8_t     pid_on;

extern volatile uint8_t  second_now;
extern volatile uint32_t seconds_counter;  // wraps every 18 hours
extern volatile uint16_t milliseconds_counter; 

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

typedef struct {
    uint16_t packets_in;
    uint16_t packets_out;
    uint16_t errors;
    stats_t loop_time;
} Performance;

extern Performance performance;

#endif
