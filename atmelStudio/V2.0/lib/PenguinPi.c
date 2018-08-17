 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "uart.h"

#include "PenguinPi.h"



int8_t 	mAQuadTable[4][4] = {{ 0, +1, -1,  2},
							 {-1,  0,  2, +1},
							 {+1,  2,  0, -1},
							 { 2, -1, +1,  0}};

int8_t  mBQuadTable[4][4] = {{ 0, -1, +1,  2},
							 {+1,  0,  2, -1},
							 {-1,  2,  0, +1},
							 { 2, +1, -1,  0}};


void detect_reset(void){
	//read MCUSR and determine what reset the AVR
	uint8_t reg = MCUSR;
	MCUSR = 0x00;
	switch (reg){
		case 1:
			uart_puts_P("Device Reset from Power On Reset\n");
		break;
		case 2:
			uart_puts_P("Device Reset from External Reset\n");
		break;
		case 4:
			uart_puts_P("Device Reset from Brown Out Detector\n");
		break;
		case 8:
			uart_puts_P("Device Reset from Watchdog Timeout\n");
		break;
		case 16:
			uart_puts_P("Device Reset from JTAG flag\n");
		break;		
		default:
			uart_puts_P("Device turned on\n");
		break;
	}
}

float readFloat(uint8_t *flMem){
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

uint8_t *float2char(float f){
	floatChar.f = f;
	return floatChar.c;
}

uint8_t crc8(uint8_t *word, uint8_t length){
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

uint16_t mapRanges(uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax){
	return ((a-amin)*((omax-omin)/(amax-amin))) + omin; //maps from scale amin->amax to scale omin->omax
}

void buttonLogic(Button *button, uint8_t btnVal){
	if(button->debounceCount == 0){
		button->pinState = btnVal;
		if(btnVal == 0){//button is held down, falling edge
			button->state ^= 1;
		}else if(btnVal == 1 && button->pinMode == 1){//rising edge
			button->state ^= 1;
		}
		button->debounceCount = 500;    // stop further transitions being processed for 500 ticks
	}
}

void uartputcs(uint8_t *datagram){
	uart_putc(STARTBYTE);
	for(uint8_t i = 0; i < datagram[0]; i++){
		uart_putc(datagram[i]);
	}
	//uart_putc(STOPBYTE);
}


//#################################################################################################
//
// I2C
//
//#################################################################################################
void i2cReadnBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg);				//where to read
	i2c_rep_start(address+I2C_READ);
	
	for(uint8_t i = 0; i < n-1; i++){
		data[i] = i2c_readAck();
	}
	data[n-1] = i2c_readNak();
	i2c_stop();
}

int8_t i2cWriteByte(uint8_t data, uint8_t address, uint8_t reg){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg); //where to write...
	i2c_write(data);
	i2c_stop();
	return 1;//completed all writes without failure
}

int8_t i2cWritenBytes(uint8_t *data, uint8_t address, uint8_t reg, uint16_t n){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg); //where to write...
	for(uint16_t i = 0; i < n; i++){
		i2c_write(data[i]);
	}
	i2c_stop();
	return 1;//completed all writes without failure
}


//#################################################################################################
//
// LEDs
//
//#################################################################################################
void LEDOff(uint8_t led){
	if(led == 0xFF){
		DDRD &= ~((1<<LED_R)|(1<<LED_G)|(1<<LED_B));
		//PORTD |= (1<<LED_R)|(1<<LED_G)|(1<<LED_B);
	}else{
		DDRD &= ~(1<<led);
		//PORTD |= (1<<led);//common anode so high: off
	}
}

void LEDOn(uint8_t led){
	if(led == 0xFF){
		DDRD |= (1<<LED_R)|(1<<LED_G)|(1<<LED_B);
		//PORTD &= ~((1<<LED_R)|(1<<LED_G)|(1<<LED_B));
	}else{
		DDRD |= (1<<led);
		//PORTD &= ~(1<<led);//common anode so low: on
	}
}

void redLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_R);
	}else{
		DDRD |= (1<<LED_R);
		OCR2A = MAP(percent, 0, 100, 0, RED_MAX);
	}
}

void greenLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_G);
	}else{
		DDRD |= (1<<LED_G);
		OCR1A = MAP(percent, 0, 100, 0, GREEN_MAX);
	}
}

void blueLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_B);
	}else{
		DDRD |= (1<<LED_B);
		OCR2B = MAP(percent, 0, 100, 0, BLUE_MAX);		
	}
}


//#################################################################################################
//
// MOTORs
//
//#################################################################################################

void debugmessage(const char *fmt, ...);

int16_t velocityPIDLoop(int16_t setPoint, Motor *motor, PidController *pid) {
    // Scale the set point to match the non-pid commands
    // Pid loop is approx 0.02 seconds, setpoint after scaling is the number of ticks per time step (of the loop)
    setPoint = setPoint/5;
    int16_t deltaDegrees = motor->degrees - pid->prevDegrees;
    pid->dt = deltaDegrees;
    pid->prevDegrees = motor->position;

    // Error
    pid->prevError = pid->error;
    pid->error = setPoint - deltaDegrees;
    pid->integralError += pid->error;

    // Controller, only proportional for now
    int16_t controlOut = (pid->kP * pid->error); // + (pid->kI * pid->integralError) + (pid->kD * (pid->error - pid->prevError));
    pid->output = controlOut;

    // Increase motor command by control output
    pid->motorCommand = pid->motorCommand + controlOut;

    /*
    debugmessage("sp %d, dt %d, e %d, u %d, mc0 %d, mc %d", 
        setPoint, deltaDegrees, pid->error, pid->output, mc0, pid->motorCommand);
        */

    // Cap the commands
    if (pid->motorCommand > 100) {
        pid->motorCommand = 100;
    }
    if (pid->motorCommand < -100) {
        pid->motorCommand = -100;
    }

    // If the input is 0, reset the controller
    if (setPoint == 0) {
        pid->motorCommand = 0;
        pid->error = 0;
        pid->prevError = 0;
        pid->output = 0;
    }

    return pid->motorCommand;
}

int16_t motorPIDControl(int16_t setPoint, Motor *motor){
	//based upon Application Note AVR221
	int16_t error = setPoint - motor->degrees;

	int16_t pTerm, dTerm;
	int32_t iTerm;
	
	//P term, also limit overflow
	if(error > motor->maxError)			pTerm = INT16_MAX;
	else if(error < -motor->maxError)	pTerm = -INT16_MAX;
	else								pTerm = motor->gainP * error;
	
	//I term, also limit overflow
	int32_t newSum = motor->errorSum + error;
	if(newSum > motor->maxErrorSum){
		motor->errorSum = motor->maxErrorSum;
		iTerm = INT32_MAX/2;
	}else if(newSum < -motor->maxErrorSum){
		motor->errorSum = -motor->maxErrorSum;
		iTerm = -(INT32_MAX/2);
	}else{
		motor->errorSum = newSum;
		iTerm = motor->gainI * motor->errorSum;
	}

	//D term //put this on a slower interval
	dTerm = motor->gainD * (motor->lastVal - motor->degrees);
	
	motor->lastVal = motor->degrees;
	
	int32_t result = (pTerm + iTerm + dTerm)/PID_SCALE;
	
	if(result > INT16_MAX) result = INT16_MAX;
	else if(result < -INT16_MAX) result = -INT16_MAX;
	
	return (int16_t)result;		
}

void fn_update_motor_states( Motor *motor, uint8_t enc_1_val, uint8_t enc_2_val ){
	
	if( motor->encoderMode == 0 ) {
		//single encoder mode, on pin 1
		//uint8_t encdiff = motor->enc1PinState ^ enc_1_val;
		if( motor->enc1PinState ^ enc_1_val ){
			if( motor->dir == 1){
				motor->position++;
				motor->lastDir = 1;
			}else if( motor->dir == -1 ){
				motor->position--;
				motor->lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if( motor->lastDir == 1 ) 		motor->position++;
				else if( motor->lastDir == -1 )	motor->position--;
			}
			motor->enc1PinState = enc_1_val;
		}//otherwise there was a tick but it wasn't the first channel...	
	}
	else if(motor->encoderMode == 1){
		//standard quadrature
		uint8_t lastEncSum 	= (motor->enc1PinState<<1)|(motor->enc2PinState);
		uint8_t encSum 		= (enc_1_val<<1)|(enc_2_val);
		int8_t  effect;
		
		if ( motor->which_motor == 1 ) {
			//Motor B
			effect 			= mBQuadTable[lastEncSum][encSum];
		}
		else {
			//Motor A
			effect 			= mAQuadTable[lastEncSum][encSum];
		}
		
		motor->position 	+= effect;
	
		motor->enc1PinState = enc_1_val;
		motor->enc2PinState = enc_2_val;
	
	}
	else if(motor->encoderMode == 2){
		//x4 counting (xor'ed both channels)
		uint8_t x4 = enc_1_val ^ enc_2_val;
		if(motor->enc1PinState ^ x4){
			if(motor->dir == 1){
				motor->position++;
				motor->lastDir = 1;
			}else if(motor->dir == -1){
				motor->position--;
				motor->lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motor->lastDir == 1) 		motor->position++;
				else if(motor->lastDir == -1) 	motor->position--;
			}
			motor->enc1PinState = x4;
		}
	}
	else{
		//my mode isn't specified
		motor->encoderMode = 1;//set to default
	}	
}

void fn_dbg_motor ( Motor *motor ){
	char 	fstring[32];
	
	if ( motor->dir == -1 ) 	uart_puts_P("  dir: -1\n");
	else if ( motor->dir == 1 )	uart_puts_P("  dir:  1\n");
	else 						uart_puts_P("  dir:  0\n");					
	
	sprintf(fstring, "  enc: %6d\n", motor->position);
	uart_puts(fstring);
	sprintf(fstring, "  deg: %6d\n", motor->degrees);
	uart_puts(fstring);			
	sprintf(fstring, "  dps: %6d\n", motor->setSpeedDPS);
	uart_puts(fstring);		
	
}
