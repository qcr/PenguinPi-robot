#ifndef __datagram_h__
#define __datagram_h__

// function signatures
uint8_t datagram_poll();  // create datagram.h ??
void datagram_validate(uint8_t *datagram, uint8_t paylen, char *msg);
void datagram_return(uint8_t *datagram, uint8_t type, ...);


//Communications Defines
#define STARTBYTE 0x11 //Device Control 1
#define BIG_ENDIAN //NETWORK BYTE ORDER
#define CRC_8_POLY 0x97 		//generator polynomial
#define DGRAM_MAX_LENGTH 10 	//bytes
#define BAUD  115200

extern uint8_t   	datagram_last[];  // previous datagram

void datagram_init();

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

//device opcodes
//MOTOR

enum _motor {
    MOTOR_SET_SPEED = 1,
    MOTOR_SET_KVP,
    MOTOR_SET_KVI,
    MOTOR_SET_ENC_ZERO,
    MOTOR_SET_ENC_MODE,
    MOTOR_SET_CONTROL_MODE,

    MOTOR_GET_SPEED = 0x81,
    MOTOR_GET_ENC,
    MOTOR_GET_KVP,
    MOTOR_GET_KVI,
    MOTOR_GET_ENC_MODE,
    MOTOR_GET_CONTROL_MODE
};

//LED
enum _led {
    LED_SET_STATE = 1,
    LED_SET_COUNT,

    LED_GET_STATE = 0x81
};

enum _oled {
    OLED_SET_IP_ETH = 0x10,
    OLED_SET_IP_WLAN
};

//ADC
enum _adc {
    ADC_SET_SCALE = 1,
    ADC_SET_POLE,

    ADC_GET_SCALE =  0x81,
    ADC_GET_POLE,
    ADC_GET_VALUE,
    ADC_GET_SMOOTH
};

//GLOBAL
enum _all {
    ALL_STOP = 0x0F,
    ALL_CLEAR_DATA,
    ALL_RESET,
    ALL_GET_DIP = 0xA0,
    ALL_GET_BUTTON,
    ALL_GET_ENC_SET_SPEED
};

//SERVO
enum _servo {
    SERVO_SET_POSITION = 0x01,
    SERVO_SET_STATE,
    SERVO_SET_MIN_RANGE,
    SERVO_SET_MAX_RANGE,
    SERVO_SET_MIN_PWM,
    SERVO_SET_MAX_PWM,

    SERVO_GET_POSITION = 0x80,
    SERVO_GET_STATE,
    SERVO_GET_MIN_RANGE,
    SERVO_GET_MAX_RANGE,
    SERVO_GET_MIN_PWM,
    SERVO_GET_MAX_PWM
};

#endif
