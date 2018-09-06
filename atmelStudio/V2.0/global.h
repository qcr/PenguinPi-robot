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

void errmessage(const char *fmt, ...);
void debugmessage(const char *fmt, ...);

void check_datagram();  // create datagram.h ??

PidController pidA;
PidController pidB;
extern uint8_t    pid_on;

//Always have
extern Motor       motorA;
extern Motor       motorB;
extern LED         leds[NLEDS];
extern AnalogIn    vdiv;
extern AnalogIn    csense;
//HAT dependant
extern Display     displayA;   //remove when parsing logic changed

extern uint8_t   datagram_last[];

#include "timer.h"
extern stats_t loop_time;

// refactor into hat1.h, hat2.h etc.
