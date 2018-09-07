#include  <stdio.h>
#include  <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdarg.h>
#include "uart.h"
#include    <util/delay.h>
#include    <util/atomic.h>
#include <setjmp.h>

#include "PenguinPi.h"
#include "hat.h"
#include "global.h"
#include "datagram.h"

// TODO
//  macros for byte packing/unpacking

//#################################################################################################
//
// Message Parsing
//
//#################################################################################################

// forward defines
static uint8_t checkBuffer(void);
static void datagram_parse( uint8_t *datagram );
static uint8_t crc8(uint8_t *word, uint8_t length);
static uint8_t checkBuffer(void);
static void uartputcs(uint8_t *datagram);
static void datagram_return(uint8_t *dgram, uint8_t type, ...);
static void datagram_parse(uint8_t *datagram);
static void datagram_print(uint8_t *datagram, char *label);

static void parseMotorOp		( uint8_t *datagram, Motor *motor);
static void parseLEDOp			( uint8_t *datagram, LED *led);
static void parseADCOp			( uint8_t *datagram, AnalogIn *adc);
static void parseAllOp			( uint8_t *datagram);
//static void parseServoOp		( uint8_t *datagram, Servo *servo);
//void parseDisplayOp		( uint8_t *datagram, Display *display);
//void parseButtonOp		( uint8_t *datagram, Button *btn);
static float char2float(uint8_t *datagram);

	
// local variables
static uint8_t      datagramG[DGRAM_MAX_LENGTH+1];      // current datagram
static uint8_t   	datagram_last[DGRAM_MAX_LENGTH+1];  // previous datagram
static jmp_buf bad_datagram;


void
datagram_poll()
{
	uint8_t com;

    com = checkBuffer();					
    if(com != STARTBYTE)
        return;

    datagram_parse( datagramG );
    LED_DEBUG_B(2);
    
    //datagram_print(datagram, "DG");
    //Save a copy of datagram for OLED display
        memcpy(&datagram_last, &datagramG, DGRAM_MAX_LENGTH);		
}

// create a return datagram in place, reusing the address and opCode
static void datagram_return(uint8_t *datagram, uint8_t type, ...)
{
    va_list ap;

    va_start(ap, type);

	// datagrams : length, address, opCode, payload[1-4], crc
	uint8_t paylen;
	switch(type) {
		case 'C':
		case 'c':
			datagram[3] = va_arg(ap, int);  // uint8_t is promoted to int by ...
			paylen = 1;
            break;
		case 'I':
		case 'i': {
            uint16_t val = va_arg(ap, uint16_t);
			datagram[3] = (val >> 8);
			datagram[4] = val & 0xFF;
			paylen = 2;
            break;
            }
		case 'H':
		case 'h': {
            uint16_t val = va_arg(ap, uint16_t);
			datagram[3] = (val >> 8);
			datagram[4] = val & 0xFF;

            val = va_arg(ap, uint16_t);
			datagram[5] = val >> 8;
			datagram[6] = val & 0xFF;
			paylen = 4;
            break;
            }
		case 'f': {
            union { float f; uint8_t c[4]; } fc;

            fc.f = (float) va_arg(ap, double);  // float is promoted to double by ...
			for(uint8_t i = 0; i < 4; i++)
				datagram[3+i] = fc.c[3-i];//avr stores as little endian
			paylen = 4;
            break;
            }
		default:
			paylen = 0;
		break;
	}
    va_end(ap);

	datagram[0] = 4 + paylen;
	datagram[3+paylen] = crc8(datagram, datagram[0]-1);

    uartputcs(datagram);
}

static void datagram_parse( uint8_t *datagram ){
	//flash RED LED
    LED_DEBUG_B(250);
	
	_delay_us(UART_INTERBYTE_WAIT);
	uint8_t dlen = checkBuffer();
	datagram[0] = dlen; //length of the datagram
	for(uint8_t i = 1; i < dlen; i++){
		_delay_us(UART_INTERBYTE_WAIT);
		
		datagram[i] = checkBuffer();
		if(i >= DGRAM_MAX_LENGTH){
			errmessage("Datagram Buffer Overflow");
            LED_DEBUG_R(250);
			return;
		}
	}
	uint8_t crcDgram = datagram[dlen-1];
	datagram[dlen-1] = 0;
	uint8_t crcCalc = crc8(datagram, dlen-1);
	datagram[0] -= 1;	

	if(crcCalc != crcDgram){
		errmessage("CRC failed");
        datagram_print(datagram, "DG");
        LED_DEBUG_R(100);
		return;
	}

    // set up a longjmp return to here
    //   return value 1 -> bad payload length
    //   return value 2 -> unknown upcode
    if (setjmp(bad_datagram) > 0)
        return;
	 
//	sprintf(fstring, "PDG %x\n", datagram[1] );
//	uart_puts(fstring);		 
	

	switch( datagram[1] ){
		case AD_MOTOR_R:
			parseMotorOp(datagram, &motorR);
            break;
		case AD_MOTOR_L:
			parseMotorOp(datagram, &motorL);
            break;
		case AD_LED_R:
			parseLEDOp(datagram, &leds[RED]);
            break;
		case AD_LED_G:
			parseLEDOp(datagram, &leds[GREEN]);
            break;
		case AD_LED_B:
			parseLEDOp(datagram, &leds[BLUE]);
            break;
#ifdef notdef
		case AD_DISPLAY_A:
			parseDisplayOp(datagram, &displayA);
            break;
#endif
		case AD_ADC_V:
			parseADCOp(datagram, &vdiv);
            break;
		case AD_ADC_C:
			parseADCOp(datagram, &csense);
            break;
		case AD_ALL:
			parseAllOp(datagram);
            break;
		default: {
            uint8_t status;
            status = hat_datagram(datagram);

            if (status == 0) {
                errmessage("Datagram: unknown address %d", datagram[1]);
                //flash RED LED
                LED_DEBUG_R(100);
            }
        }
		break;
	}
}

void 
datagram_validate(uint8_t *datagram, uint8_t paylen, char *msg)
{
    if (datagram[0] != (paylen+3)) {
        errmessage("bad datagram length (%d) for %s", datagram[0], msg);
        longjmp(bad_datagram, 1);
    }
}



/* TODO:
  change the constant in datagram[0] test to a symbolic value
   ISINT
   ISFLOAT
   ISCHAR  etc
 */
static void parseMotorOp	( uint8_t *datagram, Motor *motor ){
    uint8_t flMem[4];

	switch(datagram[2]) {
		//SETTERS
		case MOTOR_SET_SPEED:
            datagram_validate(datagram, 2, "MOTOR_SET_SPEED");
            motor->speed_dmd = (datagram[3]<<8) | datagram[4];
            break;
		case MOTOR_SET_KP:
            datagram_validate(datagram, 4, "MOTOR_SET_KP");
            for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
            motor->Kp = char2float(flMem) * PID_SCALE;
            break;
		case MOTOR_SET_KI:
            datagram_validate(datagram, 4, "MOTOR_SET_KI");
            for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
            motor->Ki = char2float(flMem) * PID_SCALE;
            break;
		case MOTOR_SET_KD:
            datagram_validate(datagram, 4, "MOTOR_SET_KD");
            for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
            motor->Kd = char2float(flMem) * PID_SCALE;
            break;
		case MOTOR_SET_KVP:
            datagram_validate(datagram, 2, "MOTOR_SET_KVP");
            motor->Kvp = (datagram[3]<<8) | datagram[4];
            break;
		case MOTOR_SET_KVI:
            datagram_validate(datagram, 2, "MOTOR_SET_KVI");
            motor->Kvi = (datagram[3]<<8) | datagram[4];
            break;
        case MOTOR_SET_ENC_ZERO:
            datagram_validate(datagram, 0, "MOTOR_SET_ENC_ZERO");
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                motor->position = 0;
            }
            break;
		case MOTOR_SET_ENC_MODE:
            datagram_validate(datagram, 4, "MOTOR_SET_ENC_MODE");
            switch (datagram[3]) {
                case 0:
                case 1:
                case 2:
                    motor->encoderMode = datagram[3];
                    break;
                default:
                    errmessage("bad encoder mode set %d", datagram[3]);
            }
            break;
		case MOTOR_SET_CONTROL_MODE:
            datagram_validate(datagram, 4, "MOTOR_SET_CONTROL_MODE");
            motor->controlMode = datagram[3];
            // should also reset some of the motor struct
            break;
		
		//GETTERS
		case MOTOR_GET_SPEED:
            datagram_validate(datagram, 0, "MOTOR_GET_SPEED");
			datagram_return(datagram, 'i', motor->speed_dmd);
            break;	
		case MOTOR_GET_KP:
            datagram_validate(datagram, 0, "MOTOR_GET_KP");
			datagram_return(datagram, 'f', motor->Kp/PID_SCALE);
			uartputcs(datagramG);
            break;	
		case MOTOR_GET_KI:
            datagram_validate(datagram, 0, "MOTOR_GET_KI");
			datagram_return(datagram, 'f', motor->Ki/PID_SCALE);
        case MOTOR_GET_KD:
            datagram_validate(datagram, 0, "MOTOR_GET_KD");
			datagram_return(datagram, 'f', motor->Kd/PID_SCALE);
            break;
		case MOTOR_GET_KVP:
            datagram_validate(datagram, 0, "MOTOR_GET_KVP");
			datagram_return(datagram, 'i', motor->Kvp);
            break;
		case MOTOR_GET_KVI:
            datagram_validate(datagram, 0, "MOTOR_GET_KVI");
			datagram_return(datagram, 'i', motor->Kvi);
            break;
		case MOTOR_GET_ENC_MODE:
            datagram_validate(datagram, 0, "MOTOR_GET_ENC_MODE");
			datagram_return(datagram, 'c', motor->encoderMode);
            break;
		case MOTOR_GET_CONTROL_MODE:
            datagram_validate(datagram, 0, "MOTOR_GET_CONTROL_MODE");
			datagram_return(datagram, 'c', motor->controlMode);
            break;
		case MOTOR_GET_ENC: {
            int16_t position;
            datagram_validate(datagram, 0, "MOTOR_GET_ENC");
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                position = motor->position;
            }
			datagram_return(datagram, 'i', position);
            break;
		    }
		default:
			errmessage("bad motor opcode %d", datagram[2]);
            longjmp(bad_datagram, 2);
		break;		
	}
}

static void parseLEDOp( uint8_t *datagram, LED *led )
{
	switch(datagram[2]) {
		//SETTERS
		case LED_SET_STATE:
            datagram_validate(datagram, 1, "LED_SET_STATE");
            led->state = datagram[3];
            break;
		case LED_SET_COUNT:
            datagram_validate(datagram, 1, "LED_SET_COUNT");
            led->count = (datagram[3]<<8)|datagram[4];
            led->state = 1;
            break;

		//GETTERS
		case LED_GET_STATE:
            datagram_validate(datagram, 0, "LED_GET_STATE");
			datagram_return(datagram, 'c', led->state);
            break;
            
		default:
			errmessage("bad LED opcode %d", datagram[2]);
            longjmp(bad_datagram, 2);
            break;
	}
}

static void parseADCOp( uint8_t *datagram, AnalogIn *adc )
{
    uint8_t flMem[4];

	switch(datagram[2]) {
		//SETTERS
		case ADC_SET_SCALE:
            datagram_validate(datagram, 4, "ADC_SET_SCALE");
            for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
            adc->scale = char2float(flMem);
            break;
		
		//GETTERS
		case ADC_GET_SCALE:
            datagram_validate(datagram, 0, "ADC_GET_SCALE");
			datagram_return(datagram, 'f', adc->scale);
            break;
		case ADC_GET_VALUE:
            datagram_validate(datagram, 0, "ADC_GET_VALUE");
			datagram_return(datagram, 'i', adc->value);
            break;
		case ADC_GET_SMOOTH:
            datagram_validate(datagram, 0, "ADC_GET_SMOOTH");
			datagram_return(datagram, 'f', adc->smooth);
            break;
		
		default:
			errmessage("bad ADC opcode %d", datagram[2]);
            longjmp(bad_datagram, 2);
            break;
	}
}

static void parseAllOp( uint8_t *datagram )
{
	switch(datagram[2]){
        case ALL_GET_ENC_SET_SPEED:
            datagram_validate(datagram, 2, "ALL_GET_ENC_SET_SPEED");
            motorR.speed_dmd = datagram[3];
            motorL.speed_dmd = datagram[4];
			datagram_return(datagram, 'h', motorL.position, motorR.position);
            break;
        case ALL_GET_DIP:
            datagram_validate(datagram, 0, "ALL_GET_DIP");
			datagram_return(datagram, 'c', (uint8_t) ((hat_status.dip >> 4) & 0x0f));
            break;
		case ALL_STOP:
            datagram_validate(datagram, 0, "ALL_STOP");
			motorR.speed_dmd = 0;
			motorL.speed_dmd = 0;
			
			// FIXME hat_oled.show_option = OLED_SHUTDOWN;
            break;
		case ALL_CLEAR_DATA:
            datagram_validate(datagram, 0, "ALL_CLEAR_DATA");
			motorR.position = 0;
			motorR.speed_dmd = 0;
            motorL.position = 0;
			motorL.speed_dmd = 0;
            break;
		
		default:
			errmessage("bad ALL opcode %d", datagram[2]);
            longjmp(bad_datagram, 2);
            break;
	}
}

static uint8_t crc8(uint8_t *word, uint8_t length){
	uint8_t crc = 0;
	for(uint8_t i=0; i < length; i++){
		crc ^= word[i];
		for(uint8_t j=0; j < 8; j++){
			if(crc & 1) crc = (crc >> 1) ^ CRC_8_POLY;
			else crc = (crc >> 1);
		}
	}
	return crc;
}

static void uartputcs(uint8_t *datagram){
	uart_putc(STARTBYTE);
	for(uint8_t i = 0; i < datagram[0]; i++){
		uart_putc(datagram[i]);
	}
	//uart_putc(STOPBYTE);
}

static uint8_t checkBuffer(void){
	uint16_t com = uart_getc();
	
	if( com & UART_NO_DATA ) {
		// there's no data available		
		return 0;
	} else {
		//check for errors
        if(com & UART_FRAME_ERROR){
            /* Framing Error detected, i.e no stop bit detected */
            errmessage("Bad UART Frame");
			//flash Blue LED
            LED_DEBUG_R(250);
			return 0;
        }
        if(com & UART_OVERRUN_ERROR){
            /* 
                * Overrun, a character already present in the UART UDR register was 
                * not read by the interrupt handler before the next character arrived,
                * one or more received characters have been dropped
                */
            errmessage("UART Buffer Overrun");
			//flash BLUE LED
            LED_DEBUG_R(250);
			return 0;
        }
        if(com & UART_BUFFER_OVERFLOW){
            /* 
                * We are not reading the receive buffer fast enough,
                * one or more received character have been dropped 
                */
            errmessage("UART Buffer Overflow");
			//flash BLUE LED
            LED_DEBUG_R(250);
			return 0;
        }
		return com & 0xFF;//return lowbyte
	}
}


#ifdef notdef
void parseDisplayOp	( uint8_t *datagram, Display *display ){

//	sprintf	 ( fstring, "parseDisplayOp: %x : ", datagram[2] );
//	uart_puts( fstring );			
	
	switch(datagram[2]){
		//SETTERS
		case DISPLAY_SET_VALUE:
			if(datagram[0] == 4){
				display->value = datagram[3];
				display->draw = 1;
			}else
				errmessage("DISPLAY_SET_VALUE: incorrect type %d", datagram[0]);
		break;
		case DISPLAY_SET_DIGIT_0:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit0 = 15;
				else if(digit < -15) display->digit0 = -15;
				else display->digit0 = digit;
				display->draw = 1;
			}else
				errmessage("DISPLAY_SET_DIGIT: incorrect type %d", datagram[0]);
		break;
		case DISPLAY_SET_DIGIT_1:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit1 = 15;
				else if(digit < -15) display->digit1 = -15;
				else display->digit1 = digit;
				display->draw = 1;
			}else
				errmessage("DISPLAY_SETDIGIT: incorrect type %d", datagram[0]);
        case DISPLAY_SET_MODE:
			if(datagram[0] == 4){
				display->mode = datagram[3];
				display->draw = 1;
			}else
				errmessage("DISPLAY_SETMODE: incorrect type %d", datagram[0]);

		break;
		
		//GETTERS
		case DISPLAY_GET_VALUE:
			dgrammem.ch = display->value;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_VALUE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_0:
			dgrammem.ch = display->digit0;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_0, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_1:
			dgrammem.ch = display->digit1;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_1, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_MODE:
			dgrammem.ch = display->mode;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		
		default:
			errmessage("bad display opcode %d", datagram[2]);
		break;
	}
}
#endif

static float char2float(uint8_t *flMem)
{
    union { float f; uint8_t c[4]; } floatChar;

	/*
	* Method courtesy AVR Freaks user 'clawson'
	* http://www.avrfreaks.net/forum/converting-4-bytes-float-help
	*/
	for(uint8_t i=0; i<4; i++){
		#ifdef LITTLE_ENDIAN
		// fill in 0, 1, 2, 3
		floatChar.c[i] = flMem[i];
		#else
		// fill in 3, 2, 1, 0
		floatChar.c[3-i] = flMem[i];
		#endif
	}
	return floatChar.f;
}

static void
datagram_print(uint8_t *datagram, char *label)
{
    char buf[80] = {0};
    int k;

    sprintf(buf, "%s: ", label);
    k = strlen(buf);
    for( uint8_t j = 0; j < datagram[0]; j++)
        k += sprintf(buf+k, "%02x ", datagram[j] );
    debugmessage(buf);
}
