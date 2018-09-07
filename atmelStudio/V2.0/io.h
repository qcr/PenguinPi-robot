/*
 * PenguinPi.h
 *
 * Created: 19/09/2016 20:47:24
 *  Author: Jack
 */ 


#ifndef PENGUINPI_H_
#define PENGUINPI_H_

#include <stdint.h>

#define DGRAM_MAX_LENGTH 10 	//bytes

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

#ifdef notdef
typedef struct {
	int8_t 				address;			// TWI address
	int8_t 				draw;
	uint8_t 			value;
	int8_t 				digit0;				// if digit 0 or 1 are not -1, then .value is overridden
	int8_t 				digit1;
    int8_t 				mode;  				// 0 is hex, 1 is unsigned decimal, 2 is signed decimal
} Display;

typedef struct {
	volatile uint8_t 	pinState;
	volatile uint8_t 	state;
	uint8_t 			pinMode;			// mode 0: state is toggled on both edges. mode 1: state is toggled on falling edges (only changes on depression)
	uint8_t 			programMode;		// mode 0: state is cleared after update code is run. mode 1: state is maintained after update (continuously updates)
	volatile int16_t 	debounceCount;
} Button;
#endif

typedef struct {
	float 				value;
    float               smooth;
	float 				scale;
    float               alpha;
    uint8_t             channel;
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

void analogFilter(AnalogIn *chan, uint16_t value);

#ifdef notdef
void update_dd7s(Display *display);
void displayBase10(uint8_t *reg, int16_t value);
void buttonLogic(Button *button, uint8_t btnVal);
#endif


//#################################################################################################
//
// PINs
//
//#################################################################################################

//Board Specific Defines
#define MOTOR_A_PWM PB3		//V1	PB2
#define MOTOR_B_PWM PB4		//V1	PB1
#define MOTOR_A_PHA PB0		//V1	PC0
#define MOTOR_B_PHA PB1		//V1	PC1

//swapping these definitions and the control logic in the main loop will reverse the running direction of the motors
#define MOTOR_A_ENC_1 PA0	//V1	PC2
#define MOTOR_A_ENC_2 PA1	//V1	PC3

#define MOTOR_B_ENC_1 PA2	//V1	PE1
#define MOTOR_B_ENC_2 PA3	//V1	PE0

//DELETE	#define SERVO_A PD0
//DELETE	#define SERVO_B PD1

#define LED_Y0	PC2
#define LED_Y1	PC3

#define LED_R PD7	//OC2A 		//V1 PD6 OC0A
#define LED_G PD5	//OC1A		//V1 PD5 OC0B
#define LED_B PD6	//OC2B		//V1 PD3 OC2B

//DELETE	#define BTN_A PB0
//DELETE	#define BTN_B PD7
//DELETE	#define BTN_C PD4

//NOT USED	//#define ADC_VDIV 	 PA6	//ADC6		//V1 ADC6	PE2
//NOT USED	//#define ADC_CSENSE PA7	//ADC7		//V1 ADC7	PE3

//#################################################################################################
//
// DEFINEs
//
//#################################################################################################

#define RPM2DPS(rpm) rpm*6.0 //convert RPM to deg/s
#define DPS2RPM(dps) dps/6.0 //convert deg/s to RPM
#define MAP(a, amin, amax, omin, omax) ((a-amin)*(omax-omin)/(amax-amin)) + omin //maps from scale amin->amax to scale omin->omax
// #define COUNTPERDEG 2.1333334//1.066667 
#define DEGPERCOUNT 0.9375 //0.46875//

//LED limits
#define RED_MAX 220
#define GREEN_MAX 120
#define BLUE_MAX 140

//Control Defines
#define CONTROL_COUNT 1
#define PID_SCALE 128
#define MOTOR_PWM_RANGE_MIN 30000

#define SERVO_PWM_RANGE_MIN 1500
#define SERVO_PWM_RANGE_MAX 3000

#define ADC_COUNT 6 //number of dummy readings before accepting an ADC read. Recommend between 1-10 (more and strange results occur)


#endif /* PENGUINPI_H_ */
