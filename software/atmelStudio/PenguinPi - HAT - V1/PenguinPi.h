/*
 * PenguinPi.h
 *
 * Created: 19/09/2016 20:47:24
 *  Author: Jack
 */ 


#ifndef PENGUINPI_H_
#define PENGUINPI_H_

//Structs
typedef struct {
	volatile uint8_t enc1PinState;
	volatile uint8_t enc2PinState;
	volatile int16_t position;  // the "encoder" value
	int8_t encoderMode;//mode 0: single encoder, mode 1: quadrature, mode 2: x4 counting (xor quadrature)
	
	int16_t speedDPS;
	int16_t degrees;
	
	int8_t dir;
	int8_t lastDir;
	int16_t setSpeedDPS;
	int16_t setDegrees;
	
	volatile uint8_t pidTimerFlag;
	int16_t gainP;
	int16_t gainI;
	int16_t gainD;
	int32_t errorSum;
	int16_t lastVal;
	int16_t maxError;
	int32_t maxErrorSum;
    int8_t  controlMode; // 0=set speed mode, 1=PID control mode
} Motor;

typedef struct {
	uint8_t state;
	int32_t setPos;//measured in degrees
	int32_t minRange;
	int32_t maxRange;
	uint16_t minPWM;
	uint16_t maxPWM;
} Servo;

typedef struct {
	volatile int8_t state; //1 on, 0 off
	int16_t brightness;
	volatile uint16_t count; //multiple of 32 us
} LED;

typedef struct {
	int8_t address;//TWI address
	int8_t draw;
	uint8_t value;
	int8_t digit0;//if digit 0 or 1 are not -1, then .value is overridden
	int8_t digit1;
    int8_t mode;  // 0 is hex, 1 is unsigned decimal, 2 is signed decimal
} Display;

typedef struct {
	volatile uint8_t pinState;
	volatile uint8_t state;
	uint8_t pinMode;//mode 0: state is toggled on both edges. mode 1: state is toggled on falling edges (only changes on depression)
	uint8_t programMode;//mode 0: state is cleared after update code is run. mode 1: state is maintained after update (continuously updates)
	volatile int16_t debounceCount;
} Button;

typedef struct {
	int16_t raw;
	float value;
	float scale;

	volatile uint8_t count;
	volatile uint8_t ready;
} AnalogIn;


union {
	float f;
	uint8_t c[4];
} floatChar;

union dgramMem {
	uint8_t uch;
	uint8_t ch;
	uint16_t uin;
	int16_t in;
	float fl;
} dgrammem;

//function prototypes
void init_structs(void);
void init(void);
void init_display(void);
void detect_reset(void);

float readFloat(uint8_t *datagram);
uint8_t *float2char(float f);
uint16_t mapRanges(uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax);

uint8_t crc8(uint8_t *word, uint8_t length);
uint8_t checkBuffer(void);
void uart1putcs(uint8_t *datagram);
void formdatagram(uint8_t *dgram, uint8_t address, uint8_t opCode, union dgramMem payl, uint8_t type);
void parseDatagram(uint8_t *datagram);

void i2cReadnBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n);
int8_t i2cWritenBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n);
int8_t i2cWriteByte(uint8_t data, uint8_t address, uint8_t reg);

void parseMotorOp(uint8_t *datagram, Motor *motor);
void parseServoOp(uint8_t *datagram, Servo *servo);
void parseLEDOp(uint8_t *datagram, LED *led);
void parseDisplayOp(uint8_t *datagram, Display *display);
void parseButtonOp(uint8_t *datagram, Button *btn);
void parseADCOp(uint8_t *datagram, AnalogIn *adc);
void parseAllOp(uint8_t *datagram);

int16_t motorPIDControl(int16_t setPoint, Motor *motor);

void LEDOff(uint8_t led);
void LEDOn(uint8_t led);
void redLEDPercent(uint8_t percent);
void greenLEDPercent(uint8_t percent);
void blueLEDPercent(uint8_t percent);

void update_dd7s(Display *display);
void displayBase10(uint8_t *reg, int16_t value);

void buttonLogic(Button *button, uint8_t btnVal);

#define RPM2DPS(rpm) rpm*6.0 //convert RPM to deg/s
#define DPS2RPM(dps) dps/6.0 //convert deg/s to RPM
#define MAP(a, amin, amax, omin, omax) ((a-amin)*(omax-omin)/(amax-amin)) + omin //maps from scale amin->amax to scale omin->omax
#define COUNTPERDEG 2.1333334//1.066667 
#define DEGPERCOUNT 0.46875//0.9375

//Board Specific Defines
#define UART_RX PB4
#define UART_TX PB3

#define TWI_SCL PC5
#define TWI_SDA PC4

#define MOTOR_A_PWM PB2
#define MOTOR_B_PWM PB1
#define MOTOR_A_PHA PC0
#define MOTOR_B_PHA PC1

//#define MOTOR_A_ENC_1 PC2 //swapping these definitions and the control logic in the main loop will reverse the running direction of the motors
//#define MOTOR_A_ENC_2 PC3
#define MOTOR_A_ENC_1 PC2
#define MOTOR_A_ENC_2 PC3

//#define MOTOR_B_ENC_1 PE0
//#define MOTOR_B_ENC_2 PE1
#define MOTOR_B_ENC_1 PE1
#define MOTOR_B_ENC_2 PE0

#define SERVO_A PD0
#define SERVO_B PD1

#define LED_R PD6
#define LED_G PD5
#define LED_B PD3

#define BTN_A PB0
#define BTN_B PD7
#define BTN_C PD4

#define ADC_VDIV PE2
#define ADC_CSENSE PE3


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

//Communications Defines
#define STARTBYTE 0x11 //Device Control 1
#define STOPBYTE  0x12 //Device Control 2

//#define LITTLE_ENDIAN
#define BIG_ENDIAN //NETWORK BYTE ORDER

#define DGRAM_MAX_LENGTH 10 //bytes
#define UART_INTERBYTE_WAIT 80 //us
#define CRC_8_POLY 0xAE //generator polynomial

//device addresses
#define AD_MOTORS 0x01
#define AD_MOTOR_A 0x04
#define AD_MOTOR_B 0x05

#define AD_SERVOS 0x02
#define AD_SERVO_A 0x08
#define AD_SERVO_B 0x09

#define AD_LEDS 0x03
#define AD_LED_R 0x0C
#define AD_LED_G 0x0D
#define AD_LED_B 0x0E

#define AD_DISPLAYS 0x04
#define AD_DISPLAY_A 0x10

#define AD_BTNS 0x05
#define AD_BTN_A 0x14
#define AD_BTN_B 0x15
#define AD_BTN_C 0x16

#define AD_ADCS 0x06
#define AD_ADC_V 0x18
#define AD_ADC_C 0x19

#define AD_ALL 0xFF

//device opcodes
//MOTOR
#define MOTOR_SET_SPEED_DPS 0x01
#define MOTOR_SET_DEGREES 0x02
#define MOTOR_SET_DIRECTION 0x03
#define MOTOR_SET_GAIN_P 0x04
#define MOTOR_SET_GAIN_I 0x05
#define MOTOR_SET_GAIN_D 0x06
#define MOTOR_SET_ENC_MODE 0x07
#define MOTOR_SET_ENC 0x08
#define MOTOR_SET_CONTROL_MODE 0x09

#define MOTOR_GET_SPEED_DPS 0x81
#define MOTOR_GET_DEGREES 0x82
#define MOTOR_GET_DIRECTION 0x83
#define MOTOR_GET_GAIN_P 0x84
#define MOTOR_GET_GAIN_I 0x85
#define MOTOR_GET_GAIN_D 0x86
#define MOTOR_GET_ENC_MODE 0x87
#define MOTOR_GET_ENC 0x88
#define MOTOR_GET_CONTROL_MODE 0x89

//SERVO
#define SERVO_SET_POSITION 0x01
#define SERVO_SET_STATE 0x02
#define SERVO_SET_MIN_RANGE 0x03
#define SERVO_SET_MAX_RANGE 0x04
#define SERVO_SET_MIN_PWM 0x05
#define SERVO_SET_MAX_PWM 0x06

#define SERVO_GET_POSITION 0x81
#define SERVO_GET_STATE 0x82
#define SERVO_GET_MIN_RANGE 0x83
#define SERVO_GET_MAX_RANGE 0x84
#define SERVO_GET_MIN_PWM 0x85
#define SERVO_GET_MAX_PWM 0x86

//LED
#define LED_SET_STATE 0x01
#define LED_SET_BRIGHTNESS 0x02
#define LED_SET_COUNT 0x03

#define LED_GET_STATE 0x81
#define LED_GET_BRIGHTNESS 0x82
#define LED_GET_COUNT 0x83

//DISPLAY
#define DISPLAY_SET_VALUE 0x01
#define DISPLAY_SET_DIGIT_1 0x02
#define DISPLAY_SET_DIGIT_0 0x03
#define DISPLAY_SET_MODE 0x04

#define DISPLAY_GET_VALUE 0x81
#define DISPLAY_GET_DIGIT_1 0x82
#define DISPLAY_GET_DIGIT_0 0x83
#define DISPLAY_GET_MODE 0x84

//BUTTON
#define BUTTON_SET_PROGRAM_MODE 0x01
#define BUTTON_SET_PIN_MODE 0x02

#define BUTTON_GET_PROGRAM_MODE 0x81
#define BUTTON_GET_PIN_MODE 0x82

//ADC
#define ADC_SET_SCALE 0x01
#define ADC_SET_RAW 0x02
#define ADC_SET_READING 0x03

#define ADC_GET_SCALE 0x81
#define ADC_GET_RAW 0x82
#define ADC_GET_READING 0x83

//GLOBAL
#define ALL_STOP 0xFF
#define CLEAR_DATA 0xEE
#define SET_RESET 0xDD
#define GET_RESET 0x11


#endif /* PENGUINPI_H_ */
