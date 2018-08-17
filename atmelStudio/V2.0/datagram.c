#include  <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include "uart.h"
#include    <util/delay.h>
#include    <util/atomic.h>

#include "PenguinPi.h"
#include "hat.h"
#include "global.h"

//#################################################################################################
//
// Message Parsing
//
//#################################################################################################

uint8_t checkBuffer(void);
	
uint8_t   	datagram_last[DGRAM_MAX_LENGTH+1];
	

void
check_datagram()
{
	uint8_t com;

    com = checkBuffer();					
    if(com != STARTBYTE)
        return;

    parseDatagram( datagramG );
    
#ifdef notdef
        //Print Datagram
        uart_puts_P("DG : ");
        for( uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) {			
            sprintf(fstring, "%x ", datagramG[j] );
            uart_puts(fstring);				
        }
        uart_puts_P("\n");									
#endif      
    //Save a copy of datagram for OLED display
        memcpy(&datagram_last, &datagramG, DGRAM_MAX_LENGTH);		
}

uint8_t checkBuffer(void){
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
            LED_DEBUG_R(1000);
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
            LED_DEBUG_R(1000);
			return 0;
        }
        if(com & UART_BUFFER_OVERFLOW){
            /* 
                * We are not reading the receive buffer fast enough,
                * one or more received character have been dropped 
                */
            errmessage("UART Buffer Overflow");
			//flash BLUE LED
            LED_DEBUG_R(1000);
			return 0;
        }
		return com & 0xFF;//return lowbyte
	}
}

void formdatagram(uint8_t *datagram, uint8_t address, uint8_t opCode, union dgramMem payl, uint8_t type){
	// datagrams : length, address, opCode, payload[1-4], crc
	datagram[1] = address;
	datagram[2] = opCode; 
	uint8_t paylen;
	switch(type){
		case 'C':
		case 'c':;
			datagram[3] = payl.ch;
			paylen = 1;
			
		break;
		case 'I':
		case 'i':;
			datagram[3] = (payl.in >> 8);
			datagram[4] = payl.in & 0xFF;
			paylen = 2;
		break;
		case 'H':
		case 'h':;
			datagram[3] = (payl.uint2[0] >> 8);
			datagram[4] = payl.uint2[0] & 0xFF;
			datagram[5] = (payl.uint2[1] >> 8);
			datagram[6] = payl.uint2[1] & 0xFF;
			paylen = 4;
		break;
		case 'f':;
			uint8_t *fl = float2char(payl.fl);
			for(uint8_t i = 0; i < 4; i++){
				datagram[3+i] = fl[3-i];//avr stores as little endian
			}
			paylen = 4;
		break;
		default:
			paylen = 0;
		break;
	}
	datagram[0] = 4 + paylen;
	datagram[3+paylen] = crc8(datagram, datagram[0]-1);
}

void parseDatagram( uint8_t *datagram ){
	//flash RED LED
    LED_DEBUG_B(1000);
	
	_delay_us(UART_INTERBYTE_WAIT);
	uint8_t dlen = checkBuffer();
	datagram[0] = dlen; //length of the datagram
	for(uint8_t i = 1; i < dlen; i++){
		_delay_us(UART_INTERBYTE_WAIT);
		
		datagram[i] = checkBuffer();
		if(i >= DGRAM_MAX_LENGTH){
			errmessage("Datagram Buffer Overflow");
            LED_DEBUG_R(1000);
			return;
		}
	}
	uint8_t crcDgram = datagram[dlen-1];
	datagram[dlen-1] = 0;
	uint8_t crcCalc = crc8(datagram, dlen-1);
	datagram[0] -= 1;	

	if(crcCalc != crcDgram){
		errmessage("CRC Failed");
        LED_DEBUG_R(1000);
		return;
	}
	 
//	sprintf(fstring, "PDG %x\n", datagram[1] );
//	uart_puts(fstring);		 
	

	switch( datagram[1] ){
		case AD_MOTOR_A:
			parseMotorOp(datagram, &motorA);
		break;
		case AD_MOTOR_B:
			parseMotorOp(datagram, &motorB);
		break;
		
//DELETE		case AD_SERVO_A:
//DELETE			parseServoOp(datagram, &servoA);
//DELETE		break;
//DELETE		case AD_SERVO_B:
//DELETE			parseServoOp(datagram, &servoB);
//DELETE		break;
		
		case AD_LED_R:
			parseLEDOp(datagram, &ledR);
		break;
		case AD_LED_G:
			parseLEDOp(datagram, &ledG);
		break;
		case AD_LED_B:
			parseLEDOp(datagram, &ledB);
		break;
		
		case AD_DISPLAY_A:
			parseDisplayOp(datagram, &displayA);
		break;
		
		case AD_ADC_V:
			parseADCOp(datagram, &vdiv);
		break;
		case AD_ADC_C:
			parseADCOp(datagram, &csense);
		break;

		case AD_ALL:
			parseAllOp(datagram);
		break;
		
		default:
            hat_datagram(datagram);

			errmessage("Datagram: unknown address %d", datagram[1]);
			//flash BLUE LED
            LED_DEBUG_R(1000);
		break;
	}
}

/* TODO:
  change the constant in datagram[0] test to a symbolic value
   ISINT
   ISFLOAT
   ISCHAR  etc
 */
void parseMotorOp	( uint8_t *datagram, Motor *motor ){
	switch(datagram[2]){
		//SETTERS
		case MOTOR_SET_SPEED_DPS:
			if(datagram[0] == 5){
				int16_t speed = (datagram[3]<<8) | datagram[4];
				motor->setSpeedDPS = abs(speed);
				if(speed > 0) motor->dir = 1;
				else if(speed < 0) motor->dir = -1;
				else motor->dir = 0;
			}else
				errmessage("ERROR: MOTOR_SET_SPEED: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_DEGREES:
			if(datagram[0] == 5){
                ATOMIC_BLOCK(ATOMIC_FORCEON) {
                    motor->position = 0;
                }
                motor->degrees = 0;

				int16_t degrees = (datagram[3]<<8) | datagram[4];
				motor->setDegrees = degrees;
				if(degrees > 0) motor->dir = 1;
				else if(degrees < 0) motor->dir = -1;
				else motor->dir = 0;
			}else
				errmessage("MOTOR_SET_DEG: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_ENC:
            // actually does a reset
            cli();
                motor->position = 0;
            sei();
            motor->degrees = 0;
        break;
		case MOTOR_SET_DIRECTION:
			if(datagram[0] == 4){
				int8_t direction = datagram[3];
				if(direction > 0) motor->dir = 1;
				else if(direction < 0) motor->dir = -1;
				else motor->dir = 0;
			} else
				errmessage("MOTOR_SET_DIR: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_GAIN_P:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainP = readFloat(flMem) * PID_SCALE;
				motor->maxError = INT16_MAX / (motor->gainP + 1);				
			}else
				errmessage("MOTOR_SET_P: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_GAIN_I:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainI = readFloat(flMem) * PID_SCALE;
				motor->maxErrorSum = (INT32_MAX / 2) / (motor->gainI + 1);
			}else
				errmessage("MOTOR_SET_I: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_GAIN_D:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainD = readFloat(flMem) * PID_SCALE;
			}else
				errmessage("MOTOR_SET_D: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_ENC_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode > 2) motor->encoderMode = 1; //the default
				else motor->encoderMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else
				errmessage("MOTOR_SET_ENCMODE: incorrect type %d", datagram[0]);
		break;
		case MOTOR_SET_CONTROL_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				motor->controlMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else
				errmessage("MOTOR_SET_CONTROLMODE: incorrect type %d", datagram[0]);
		break;
		
		//GETTERS
		case MOTOR_GET_SPEED_DPS:
			dgrammem.in = motor->speedDPS;
			formdatagram(datagramG, datagram[1], MOTOR_SET_SPEED_DPS, dgrammem, 'i');
			uartputcs(datagramG);
		break;	
		case MOTOR_GET_DEGREES:
			dgrammem.in = motor->degrees;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DEGREES, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_DIRECTION:
			dgrammem.ch = motor->dir;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DIRECTION, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_GAIN_P:
			dgrammem.fl = motor->gainP/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_P, dgrammem, 'f');
			uartputcs(datagramG);
		break;	
		case MOTOR_GET_GAIN_I:
			dgrammem.fl = motor->gainI/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_I, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_GAIN_D:
			dgrammem.fl = motor->gainD/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_D, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_ENC_MODE:
			dgrammem.ch = motor->encoderMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_CONTROL_MODE:
			dgrammem.ch = motor->controlMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_CONTROL_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_ENC:
            cli();
                dgrammem.in = motor->position;
            sei();
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		
		default:
			errmessage("bad motor opcode %d", datagram[2]);
		break;		
	}
}

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

void parseLEDOp		( uint8_t *datagram, LED *led ){
	switch(datagram[2]){
		//SETTERS
		case LED_SET_STATE:
			if(datagram[0] == 4){
				int8_t state = datagram[3];
				if(state >= 1) led->state = 1;
				else led->state = 0;
			}else
				errmessage("LED_SETSTATE: incorrect type %d", datagram[0]);
		break;
		case LED_SET_BRIGHTNESS:
			if(datagram[0] == 4){
				int8_t brightness = datagram[3];
				if(brightness > 100) led->brightness = 100;
				else if(brightness > 0) led->brightness = brightness;
				else led->brightness = 0;
			}else
				errmessage("LED_SETBRIGHT: incorrect type %d", datagram[0]);

		break;
		case LED_SET_COUNT:
			if(datagram[0] == 5){
				led->count = (datagram[3]<<8)|datagram[4];
                led->state = 1;
			}else
				errmessage("LED_SETCOUNT: incorrect type %d", datagram[0]);
		break;

		//GETTERS
		case LED_GET_STATE:
			dgrammem.ch = led->state;
			formdatagram(datagramG, datagram[1], LED_SET_STATE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case LED_GET_BRIGHTNESS:
			dgrammem.ch = led->brightness;
			formdatagram(datagramG, datagram[1], LED_SET_BRIGHTNESS, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case LED_GET_COUNT:
			dgrammem.uin = led->count;
			formdatagram(datagramG, datagram[1], LED_SET_COUNT, dgrammem, 'i');
			uartputcs(datagramG);
		break;

		default:
			errmessage("bad LED opcode %d", datagram[2]);
		break;
	}
}

void parseADCOp		( uint8_t *datagram, AnalogIn *adc ){
	switch(datagram[2]){
		//SETTERS
		case ADC_SET_SCALE:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				adc->scale = readFloat(flMem);
			}else
				errmessage("ADC_SETSCALE: incorrect type %d", datagram[0]);
		break;
		
		//GETTERS
		case ADC_GET_SCALE:
			dgrammem.fl = adc->scale;
			formdatagram(datagramG, datagram[1], ADC_SET_SCALE, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case ADC_GET_RAW:
			dgrammem.in = adc->raw;
			formdatagram(datagramG, datagram[1], ADC_SET_RAW, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		case ADC_GET_READING:
			dgrammem.fl = adc->value;
			formdatagram(datagramG, datagram[1], ADC_SET_READING, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		
		default:
			errmessage("bad ADC opcode %d", datagram[2]);
			//flash BLUE LED
            LED_DEBUG_R(1000);
		break;
	}
}

void parseAllOp		( uint8_t *datagram ){
	switch(datagram[2]){
        case ALL_GET_MOTORS:
			if(datagram[0] == 5) {
				int16_t speedA = datagram[3];
				motorA.setSpeedDPS = abs(speedA);
				if(speedA > 0)
                    motorA.dir = 1;
				else if(speedA < 0)
                    motorA.dir = -1;
				else 
                    motorA.dir = 0;
				int16_t speedB = datagram[4];
				motorB.setSpeedDPS = abs(speedB);
				if(speedB > 0)
                    motorB.dir = 1;
				else if(speedB < 0)
                    motorB.dir = -1;
				else 
                    motorB.dir = 0;

			} else
				errmessage("ERROR: ALL_GET_MOTORS: incorrect type %d", datagram[0]);
			dgrammem.uint2[0] = motorA.position;
			dgrammem.uint2[1] = motorB.position;
			formdatagram(datagramG, datagram[1], ALL_SET_MOTORS, dgrammem, 'h');
			uartputcs(datagramG);
            break;

        case ALL_GET_DIP:
			dgrammem.ch = (hat_status.dip >> 4) & 0x0f;
			formdatagram(datagramG, datagram[1], ALL_SET_DIP, dgrammem, 'c');
			uartputcs(datagramG);
        break;
		case ALL_STOP:
			motorA.position = 0;
			motorA.setDegrees = 0;
			motorA.setSpeedDPS = 0;
			motorB.position = 0;
			motorB.setDegrees = 0;
			motorB.setSpeedDPS = 0;
//DELETE			servoA.state = 0;
//DELETE			servoB.state = 0;
			displayA.draw = 0;
			ledR.state = 0;
			ledG.state = 0;
			ledB.state = 0;
			
			
			// FIXME hat_oled.show_option = OLED_SHUTDOWN;
		break;
		case CLEAR_DATA:
			motorA.position = 0;
			motorA.setDegrees = 0;
			motorA.setSpeedDPS = 0;
		//	motorA.enc_raw1 = 0;
		//	motorA.enc_raw2 = 0;
            
            motorB.position = 0;
			motorB.setDegrees = 0;
			motorB.setSpeedDPS = 0;
		//	motorB.enc_raw1 = 0;
		//	motorB.enc_raw2 = 0;
		
            
            //motorA = (Motor){0};
			//motorB = (Motor){0};
//DELETE			servoA = (Servo){0};
//DELETE			servoB = (Servo){0};
			//ledR = (LED){0};
			//ledG = (LED){0};
			//ledB = (LED){0};
			//displayA = (Display){0};
//DELETE			buttonA = (Button){0};
//DELETE			buttonB = (Button){0};
//DELETE			buttonC = (Button){0};
			//vdiv = (AnalogIn){0};
			//csense = (AnalogIn){0};
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
            LED_DEBUG_R(1000);
		break;
	}
}



