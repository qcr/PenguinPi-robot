#define  LED_DEBUG

#ifdef  LED_DEBUG
  #define LED_DEBUG_R(duration) \
              ledR.state = 1;\
                      ledR.count = duration;
  #define LED_DEBUG_G(duration) \
              ledG.state = 1;\
                      ledG.count = duration;
  #define LED_DEBUG_B(duration) \
              ledB.state = 1;\
                      ledB.count = duration;
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
extern LED         ledR;
extern LED         ledG;
extern LED         ledB;
extern AnalogIn    vdiv;
extern AnalogIn    csense;
//HAT dependant
extern Display     displayA;   //remove when parsing logic changed

extern uint8_t   datagram_last[];

// refactor into hat1.h, hat2.h etc.
