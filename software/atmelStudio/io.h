/*
 * PenguinPi.h
 *
 * Created: 19/09/2016 20:47:24
 *  Author: Jack
 */ 

#ifndef __io_h__
#define __io_h__

#include <stdint.h>

#define F_CPU 20000000UL
#define I2C_IMU	0b01101000

//Global variables

//Structs

typedef struct {
	uint8_t 			state;
	int32_t 			setPos;				// measured in degrees
	int32_t 			minRange;
	int32_t 			maxRange;
	uint16_t 			minPWM;
	uint16_t 			maxPWM;
} Servo;

enum _leds  {
    RED = 0, BLUE, GREEN, Y2, Y3, Y4, NLEDS
};

typedef struct {
	volatile uint8_t 	state; 				// 1 on, 0 off
	volatile uint8_t 	count; 				// milliseconds
} LED;

typedef struct {
	float 				value;
    float               smooth;
	float 				scale;
    float               alpha;      // z-domain pole
    uint8_t             channel;
    uint8_t             initialized;
} AnalogIn;

struct Battery {
	float 				mvolts_warning;				//volts
	float 				mvolts_shutdown;				//volts
	uint8_t 			warning;                    // inverted screen showing
} battery;

//function prototypes
void 	init_structs		( void );
void 	init				( void );
void 	init_display		( void );
void 	detect_reset		( void );

uint16_t mapRanges			( uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax );

void 	i2cReadnBytes	(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n);
int8_t 	i2cWritenBytes	(uint8_t *data, uint8_t address, uint8_t reg, uint16_t n);
int8_t 	i2cWriteByte	(uint8_t data, uint8_t address, uint8_t reg);

void LEDOff(enum _leds led);
void LEDOn(enum _leds led);

void io_analog_filter_step(AnalogIn *chan, uint16_t value);
void io_init(void);


//#################################################################################################
//
// PINs
//
//#################################################################################################

//Board Specific Defines
#define MOTOR_A_PWM PB3
#define MOTOR_B_PWM PB4
#define MOTOR_A_PHA PB0
#define MOTOR_B_PHA PB1

//swapping these definitions and the control logic in the main loop will reverse the running direction of the motors
#define MOTOR_A_ENC_1 PA0
#define MOTOR_A_ENC_2 PA1

#define MOTOR_B_ENC_1 PA2
#define MOTOR_B_ENC_2 PA3

#define HAT6    PA4
#define HAT1    PA5
#define HAT2    PB2
#define HAT5    PD2
#define HAT3    PD3
#define HAT4    PD4
#define HAT0    PC6
#define HAT7    PC7

//DELETE	#define SERVO_A PD0
//DELETE	#define SERVO_B PD1

#define LED_Y0	PC2
#define LED_Y1	PC3

#define LED_R PD7	
#define LED_G PD5
#define LED_B PD6

//#################################################################################################
//
// DEFINEs
//
//#################################################################################################

//maps from scale amin->amax to scale omin->omax
#define MAP(a, amin, amax, omin, omax) ((a-amin)*(omax-omin)/(amax-amin)) + omin 

//Control Defines
#define SERVO_PWM_RANGE_MIN 1500
#define SERVO_PWM_RANGE_MAX 3000

//Vcc reference voltage with external cap on AREF
#define ADC_REF (0<<REFS1)|(1<<REFS0) 


#endif 
