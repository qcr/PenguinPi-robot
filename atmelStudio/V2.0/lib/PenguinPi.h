/*
 * PenguinPi.h
 *
 * Created: 19/09/2016 20:47:24
 *  Author: Jack
 */ 


#ifndef PENGUINPI_H_
#define PENGUINPI_H_

#define DGRAM_MAX_LENGTH 10 	//bytes

#define I2C_IMU	0b01101000


//Global variables
uint8_t datagramG[DGRAM_MAX_LENGTH+1];
char 	fstring[32];





//Structs
typedef struct {
	volatile uint8_t 	enc1PinState;
	volatile uint8_t 	enc2PinState;
	volatile int16_t 	position;  			// the "encoder" value
	int8_t 			 	encoderMode;		// mode 0: single encoder, mode 1: quadrature, mode 2: x4 counting (xor quadrature)
 
  int16_t      enc_raw1;
  int16_t      enc_raw2; 
	
	int16_t 			speedDPS;
	int16_t 			degrees;
	
	int8_t				which_motor;		// 0 for MotorA, 1 for MotorB, so different quadrature maps can be used when encoder changes.
	int8_t  			dir;
	int8_t  			lastDir;
	int16_t 			setSpeedDPS;
	int16_t 			setDegrees;
	
	volatile uint8_t 	pidTimerFlag;
	int16_t 			gainP;
	int16_t 			gainI;
	int16_t 			gainD;
	int32_t 			errorSum;
	int16_t 			lastVal;
	int16_t 			maxError;
	int32_t 			maxErrorSum;
    int8_t  			controlMode; 		// 0=set speed mode, 1=PID control mode
} Motor;

typedef struct {
	uint8_t 			state;
	int32_t 			setPos;				// measured in degrees
	int32_t 			minRange;
	int32_t 			maxRange;
	uint16_t 			minPWM;
	uint16_t 			maxPWM;
} Servo;

typedef struct {
	volatile int8_t 	state; 				// 1 on, 0 off
	int16_t 			brightness;
	volatile uint16_t 	count; 				// multiple of 32 us
} LED;

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

typedef struct {
	int16_t 			raw;
	float 				value;
	float 				scale;

	volatile uint8_t 	count;
	volatile uint8_t 	ready;
} AnalogIn;

struct Battery {
	float 				cutoff;				//volts
	uint16_t 			count;
	uint16_t 			limit;				//number of cycles until it triggers a shutdown
} battery;

typedef struct {
	int8_t 				config;				// -1 if no HAT present
	int8_t 				dip;				// Status of the DIP switches 
	uint8_t				dir;				// Set bit to 1 if direction of bit needs to be an output
	uint8_t				int_07;				// Set bit to a 1 if interrupt enabled on HAT07
	uint8_t				has_oled;			// Set bit to a 1 if the I2C OLED is on the hat
} Hat_s;

typedef struct {
	uint8_t 			show_option;		// Selects which screen to show
	
	//IP Addresses
	int16_t				eth_addr_1;	
	int16_t				eth_addr_2;
	int16_t				eth_addr_3;
	int16_t				eth_addr_4;
	
	int16_t				wlan_addr_1;	
	int16_t				wlan_addr_2;
	int16_t				wlan_addr_3;
	int16_t				wlan_addr_4;
	
	//Error messages
	uint8_t				err_line_1[21];
	uint8_t 			err_line_2[21];
	uint8_t				err_line_3[21];
	
} Hat_oled;


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



//PID PROTO

typedef struct PidController {
    // GAINS
    int16_t kP;
    int16_t kI;
    int16_t kD;

    // TERMS
    int16_t p;
    int16_t i;
    int16_t d;

    // ERROR
    int16_t error;
    int16_t prevError;

    int16_t integralError;

    int16_t motorErrorScale;

    // Conversion
    int16_t prevDegrees;

    int16_t dt;

    // Out
    int16_t output;
    int16_t motorCommand;
} PidController;




//function prototypes
void 	init_structs		( void );
void 	init				( void );
void 	init_display		( void );
void 	init_hat			( Hat_s *hat );
void 	init_oled			( void );

void 	oled_clear_frame	( );
void    oled_frame_divider  ( );
void 	oled_write_frame	( );
void 	oled_character		( uint8_t x, uint8_t y, char character );
void 	oled_string			( uint8_t x, uint8_t y, char *string );

void 	oled_screen    		( Hat_oled *oled, AnalogIn *vdiv, AnalogIn *csense, Motor *motorA, Motor *motorB, Display *display, uint8_t *datagram, PidController pidA, PidController pidB, uint8_t pid_on, double pid_dt);
void    oled_next_screen	( Hat_oled *oled ); 
void    oled_show_error		( Hat_oled *oled, char *msg ); 

void 	detect_reset		( void );

float 	readFloat			( uint8_t *datagram );
uint8_t *float2char			( float f );
uint16_t mapRanges			( uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax );

uint8_t crc8(uint8_t *word, uint8_t length);
uint8_t checkBuffer(void);
void uartputcs(uint8_t *datagram);
void formdatagram(uint8_t *dgram, uint8_t address, uint8_t opCode, union dgramMem payl, uint8_t type);
void parseDatagram(uint8_t *datagram);

void 	i2cReadnBytes	(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n);
int8_t 	i2cWritenBytes	(uint8_t *data, uint8_t address, uint8_t reg, uint16_t n);
int8_t 	i2cWriteByte	(uint8_t data, uint8_t address, uint8_t reg);

void parseMotorOp		( uint8_t *datagram, Hat_oled *oled, Motor *motor);
void parseServoOp		( uint8_t *datagram, Hat_oled *oled, Servo *servo);
void parseLEDOp			( uint8_t *datagram, Hat_oled *oled, LED *led);
void parseOLEDOp		( uint8_t *datagram, Hat_oled *hat_oled);
void parseDisplayOp		( uint8_t *datagram, Hat_oled *oled, Display *display);
void parseButtonOp		( uint8_t *datagram, Hat_oled *oled, Button *btn);
void parseADCOp			( uint8_t *datagram, Hat_oled *oled, AnalogIn *adc);
void parseAllOp			( uint8_t *datagram);

int16_t motorPIDControl			( int16_t setPoint, Motor *motor );
void 	fn_update_motor_states  ( Motor *motor, uint8_t enc_1_val, uint8_t enc_2_val );
void 	fn_dbg_motor			( Motor *motor );

void LEDOff(uint8_t led);
void LEDOn(uint8_t led);
void redLEDPercent(uint8_t percent);
void greenLEDPercent(uint8_t percent);
void blueLEDPercent(uint8_t percent);

void update_dd7s(Display *display);
void displayBase10(uint8_t *reg, int16_t value);

void buttonLogic(Button *button, uint8_t btnVal);





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

//Communications Defines
#define STARTBYTE 0x11 //Device Control 1
#define STOPBYTE  0x12 //Device Control 2

//#define LITTLE_ENDIAN
#define BIG_ENDIAN //NETWORK BYTE ORDER


#define UART_INTERBYTE_WAIT 80 	//us
#define CRC_8_POLY 0xAE 		//generator polynomial

//device addresses
#define AD_MOTORS  		0x01
#define AD_MOTOR_A 		0x04
#define AD_MOTOR_B 		0x05
		
#define AD_SERVOS  		0x02
#define AD_SERVO_A 		0x08
#define AD_SERVO_B 		0x09

#define AD_LEDS  		0x03
#define AD_LED_R 		0x0C
#define AD_LED_G 		0x0D
#define AD_LED_B 		0x0E

//#define AD_DISPLAYS  	0x04	--same as MOTOR A !!!
#define AD_DISPLAY_A 	0x10

#define AD_BTNS  		0x05
#define AD_BTN_A 		0x14
#define AD_BTN_B 		0x15
#define AD_BTN_C 		0x16

#define AD_ADCS 		0x06
#define AD_ADC_V 		0x18
#define AD_ADC_C 		0x19

#define AD_OLED			0x07

#define AD_ALL 			0xFF

//device opcodes
//MOTOR
#define MOTOR_SET_SPEED_DPS 	0x01
#define MOTOR_SET_DEGREES 		0x02
#define MOTOR_SET_DIRECTION 	0x03
#define MOTOR_SET_GAIN_P 		0x04
#define MOTOR_SET_GAIN_I 		0x05
#define MOTOR_SET_GAIN_D 		0x06
#define MOTOR_SET_ENC_MODE 		0x07
#define MOTOR_SET_ENC 			0x08
#define MOTOR_SET_CONTROL_MODE 	0x09

#define MOTOR_GET_SPEED_DPS 	0x81
#define MOTOR_GET_DEGREES 		0x82
#define MOTOR_GET_DIRECTION 	0x83
#define MOTOR_GET_GAIN_P 		0x84
#define MOTOR_GET_GAIN_I 		0x85
#define MOTOR_GET_GAIN_D 		0x86
#define MOTOR_GET_ENC_MODE 		0x87
#define MOTOR_GET_ENC 			0x88
#define MOTOR_GET_CONTROL_MODE 	0x89

//SERVO
#define SERVO_SET_POSITION 		0x01
#define SERVO_SET_STATE 		0x02
#define SERVO_SET_MIN_RANGE 	0x03
#define SERVO_SET_MAX_RANGE 	0x04
#define SERVO_SET_MIN_PWM 		0x05
#define SERVO_SET_MAX_PWM 		0x06

#define SERVO_GET_POSITION 		0x81
#define SERVO_GET_STATE 		0x82
#define SERVO_GET_MIN_RANGE 	0x83
#define SERVO_GET_MAX_RANGE 	0x84
#define SERVO_GET_MIN_PWM 		0x85
#define SERVO_GET_MAX_PWM 		0x86

//LED
#define LED_SET_STATE 0x01
#define LED_SET_BRIGHTNESS 0x02
#define LED_SET_COUNT 0x03

#define LED_GET_STATE 0x81
#define LED_GET_BRIGHTNESS 0x82
#define LED_GET_COUNT 0x83

//DISPLAY
#define DISPLAY_SET_VALUE 	0x01
#define DISPLAY_SET_DIGIT_1 0x02
#define DISPLAY_SET_DIGIT_0 0x03
#define DISPLAY_SET_MODE 	0x04

#define DISPLAY_GET_VALUE 	0x81
#define DISPLAY_GET_DIGIT_1 0x82
#define DISPLAY_GET_DIGIT_0 0x83
#define DISPLAY_GET_MODE 	0x84

//OLED
#define OLED_SET_IP_ETH_1 	0x01
#define OLED_SET_IP_ETH_2 	0x02
#define OLED_SET_IP_ETH_3 	0x03
#define OLED_SET_IP_ETH_4	0x04
#define OLED_SET_IP_WLAN_1 	0x05
#define OLED_SET_IP_WLAN_2 	0x06
#define OLED_SET_IP_WLAN_3 	0x07
#define OLED_SET_IP_WLAN_4	0x08

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

//OLED Screen Options
#define OLED_IP_ADDR	0
#define OLED_BATTERY	1
#define OLED_MOTORS		2 
#define OLED_ENCODERS	3
#define OLED_POSITION 4
#define OLED_PID      5

//#define OLED_DISPLAY	5
//#define OLED_DATAGRAM	5

//Options are screens that appear but not via user selection
#define OLED_ERROR		6	//Keep this as first is non user list
#define OLED_SHUTDOWN	7	


static const uint8_t ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};


#endif /* PENGUINPI_H_ */
