#ifndef __datagram_h__
#define __datagram_h__

// function signatures
uint8_t datagram_poll();  // create datagram.h ??
void datagram_validate(uint8_t *datagram, uint8_t paylen, char *msg);
void datagram_return(uint8_t *datagram, uint8_t type, ...);


//Communications Defines
#define STARTBYTE 0x11 //Device Control 1
#define STOPBYTE  0x12 //Device Control 2

//#define LITTLE_ENDIAN
#define BIG_ENDIAN //NETWORK BYTE ORDER

#define UART_INTERBYTE_WAIT 80 	//us
#define CRC_8_POLY 0xAE 		//generator polynomial

//device addresses
enum _addresses {
    AD_MOTORS =		0x10,
    AD_MOTOR_R,
    AD_MOTOR_L,
            
    AD_SERVOS =		0x20,
    AD_SERVO_A,
    AD_SERVO_B,

    AD_LEDS =		0x30,
    AD_LED_R,
    AD_LED_G,
    AD_LED_B,
    AD_LED_2,
    AD_LED_3,
    AD_LED_4,

    AD_ADCS =       0x40,
    AD_ADC_V,
    AD_ADC_C,
    AD_OLED =       0x50,

    AD_ALL =		0x60
};

#ifdef notdef
#define AD_DISPLAY_A 	0x10

#define AD_BTNS  		0x05
#define AD_BTN_A 		0x14
#define AD_BTN_B 		0x15
#define AD_BTN_C 		0x16
#endif


//device opcodes
//MOTOR

enum _motor {
    MOTOR_SET_SPEED = 1,
    MOTOR_SET_KP,
    MOTOR_SET_KI,
    MOTOR_SET_KD,
    MOTOR_SET_KVP,
    MOTOR_SET_KVI,
    MOTOR_SET_ENC_MODE,
    MOTOR_SET_ENC_ZERO,
    MOTOR_SET_CONTROL_MODE,

    MOTOR_GET_SPEED = 0x81,
    MOTOR_GET_KP,
    MOTOR_GET_KD,
    MOTOR_GET_KI,
    MOTOR_GET_KVP,
    MOTOR_GET_KVI,
    MOTOR_GET_ENC_MODE,
    MOTOR_GET_ENC,
    MOTOR_GET_CONTROL_MODE
};

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
enum _led {
    LED_SET_STATE = 1,
    LED_SET_COUNT,

    LED_GET_STATE = 0x81
};

#ifdef notdef
//DISPLAY
#define DISPLAY_SET_VALUE 	0x01
#define DISPLAY_SET_DIGIT_1 0x02
#define DISPLAY_SET_DIGIT_0 0x03
#define DISPLAY_SET_MODE 	0x04

#define DISPLAY_GET_VALUE 	0x81
#define DISPLAY_GET_DIGIT_1 0x82
#define DISPLAY_GET_DIGIT_0 0x83
#define DISPLAY_GET_MODE 	0x84
#endif

//OLED
#define OLED_SET_IP_ETH_1 	0x01
#define OLED_SET_IP_ETH_2 	0x02
#define OLED_SET_IP_ETH_3 	0x03
#define OLED_SET_IP_ETH_4	0x04
#define OLED_SET_IP_WLAN_1 	0x05
#define OLED_SET_IP_WLAN_2 	0x06
#define OLED_SET_IP_WLAN_3 	0x07
#define OLED_SET_IP_WLAN_4	0x08

enum _oled {
    OLED_SET_IP_ETH = 0x10,
    OLED_SET_IP_WLAN,
    OLED_GET_BUTTON = 0x80
};

#ifdef notdef
//BUTTON
#define BUTTON_SET_PROGRAM_MODE 0x01
#define BUTTON_SET_PIN_MODE 0x02

#define BUTTON_GET_PROGRAM_MODE 0x81
#define BUTTON_GET_PIN_MODE 0x82
#endif

//ADC
enum _adc {
    ADC_SET_SCALE = 1,

    ADC_GET_SCALE =  0x81,
    ADC_GET_VALUE,
    ADC_GET_SMOOTH
};

//GLOBAL
enum _all {
    ALL_STOP = 0x0F,
    ALL_CLEAR_DATA,
    ALL_RESET,
    ALL_GET_DIP = 0xA0,
    ALL_GET_ENC_SET_SPEED
};

#endif
