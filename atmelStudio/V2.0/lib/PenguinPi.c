 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "PCA6416A.h"
#include "SSD1306.h"
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

//OLED FRAME	
uint8_t oled_frame[ SSD1306_BUFFERSIZE ] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8,
0xF8, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0x1F, 0x18, 0x3E, 0x24, 0x2E,
0x08, 0x0F, 0x7F, 0xFC, 0xF0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
0xFE, 0xFE, 0xFE, 0x02, 0x02, 0x86, 0xFE, 0xFE, 0xFC, 0x78, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xE0,
0x20, 0x60, 0xE0, 0xC0, 0x80, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x60, 0xE0, 0xE0, 0xE0, 0xC0,
0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0x20, 0xE0, 0xF8, 0xCC, 0x9C, 0x1C, 0x00, 0x00, 0xE0, 0xE0,
0xE0, 0xE0, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0xEE, 0xEE, 0xEE, 0xE0, 0x00, 0x00,
0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x60, 0xE0, 0xE0, 0xE0, 0xC0, 0x00, 0x00, 0xFE, 0xFE, 0xFE, 0xFE,
0x02, 0x02, 0x86, 0xFE, 0xFE, 0xFC, 0x78, 0x00, 0x00, 0xEE, 0xEE, 0xEE, 0xE0, 0x00, 0x00, 0x00,

0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x38, 0x3F, 0x4F, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xC0, 0x28, 0x7F, 0x7F, 0x7F, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
0x7F, 0x7F, 0x7F, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0x7A,
0x42, 0x42, 0x43, 0x3B, 0x1B, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F,
0x00, 0x70, 0x73, 0xFF, 0xEF, 0xEF, 0xE8, 0xEF, 0xEF, 0xE7, 0xE3, 0xC0, 0x00, 0x00, 0x3F, 0x7F,
0x7F, 0x7F, 0x60, 0x30, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00,
0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F,
0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x00,

0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x13, 0x0F, 0x0C, 0x0C, 0x0C,
0x0E, 0x0E, 0x0E, 0x11, 0x30, 0x10, 0x08, 0x0C, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x07, 0x04, 0x04, 0x04, 0x04, 0x04, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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


//#################################################################################################
//
// HATs
//
//#################################################################################################

void init_hat( Hat_s *hat ) {
	
	uint8_t data_r[2] = {0, 0};
	
	//Config always on PCA6416A_1
	//Read 2 bytes from address 0
	//Check if HAT responds on I2C - Bails out if not an ACK on first attempt - I2C will hang otherwise.
	
	if ( i2c_check_device( PCA6416A_1+I2C_WRITE ) == -1 ){
		//HAT not present or not ready
	}
	else {	
		i2cReadnBytes(data_r, PCA6416A_1, 0x00, 2 );
	
//		for(uint8_t i = 0; i < 2; i++){
//			sprintf(fstring, "CFG: 0x%2x\n", data_r[i] );
//			uart_puts(fstring);					
//		}			

		hat->config	= data_r[0] & 0x0F;
	}
	
	//What to do now with each different HAT option
	switch( hat->config ) {
		case 1 : 
			uart_puts_P("HAT ID 1 : LEDs and OLED\n");			
			//This is the OLED and LED HAT made for EGB439
			//Has a DIP switch
				hat->dip		= data_r[0] & 0xF0;
			//4 buttons on data_r[1][3:0]
				
			//GPIO : All inupts
			//Interrupt on HAT07
				hat->dir		= 0x00;
				hat->int_07		= 1;
				
			//Has OLED
				hat->has_oled	= 1;
				
			//Expander0 on HAT needs all IO turned to outputs for LEDS
				data_r[0]	= 0;	//0 for OUTPUT
				data_r[1]	= 0;	//0 for OUTPUT
				i2cWritenBytes( data_r, PCA6416A_0, 0x06, 2);
				
			break;

		case 2 : 
			uart_puts_P("HAT ID 2 : IMU and OLED\n");			
			//This is the OLED and IMU made for EGB445 ( Segway Robot )
			//Has a DIP switch
				hat->dip		= data_r[0] & 0xF0;
			//4 buttons on data_r[1][3:0]
			
			//GPIO : All inupts
			//Interrupt on HAT07
				hat->dir		= 0x00;
				hat->int_07		= 1;

			//Has OLED
				hat->has_oled	= 1;
				
			break;			
	
		default :
			uart_puts_P("HAT NOT PRESENT or not SUPPORTED\n");			
	}


	if ( hat->int_07 == 1 ) {
		//Enable PCINT2 interrupt
//		uart_puts_P("EN PCINT2\n");	
		
		PCMSK2  |=   (1<<PCINT23);
		
		PCICR 	|=   (1<<PCIE2);		
	}
	
	
	//If any bits in hat->dir are '1' we need to make them an output from the AVR	
	//HAT 07	C7
	//HAT 06	A4
	//HAT 05	D2
	//HAT 04	D4
	//HAT 03	D3
	//HAT 02	B2
	//HAT 01	A5
	//HAT 00	C6

	
	if ( hat->has_oled == 1) {
		//Initialise OLED
		init_oled();
	}










	
//Debug to test switches on HAT
//	while (1){
//		i2cReadnBytes(data_r, PCA6416A_1, 0x00, 2 );	
//
//		sprintf(fstring, "00 : 0x%2x", data_r[0] );
//		uart_puts(fstring);					
//		sprintf(fstring, "..01 : 0x%2x\n", data_r[1] );
//		uart_puts(fstring);	
//
//		_delay_ms(100);		
//		
//	}

//Debug test to set LEDs on HAT	
	data_r[0]	= 0;
	data_r[1]	= 0;
	for(uint8_t j = 0; j < 8; j++) {

		data_r[0]	=0<<j;

		i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
		_delay_ms(500);		
	}

	for(uint8_t j = 0; j < 8; j++) {

		data_r[1]	= 0<<j;

		i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
		_delay_ms(500);		
	}
}


//#################################################################################################
//
// OLED
//
//#################################################################################################

void init_oled () {
//	uart_puts_P("INIT OLED\n");
	
	i2cWriteByte( SSD1306_DISPLAYOFF, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
	
    i2cWriteByte( SSD1306_SETDISPLAYCLOCKDIV, 	SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x80, 						SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_SETMULTIPLEX, 		SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( SSD1306_HEIGHT-1,				SSD1306_DEFAULT_ADDRESS, 0x00 );
    
    i2cWriteByte( SSD1306_SETDISPLAYOFFSET, 	SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x00, 						SSD1306_DEFAULT_ADDRESS, 0x00 );
    
    i2cWriteByte( SSD1306_SETSTARTLINE, 		SSD1306_DEFAULT_ADDRESS, 0x00 );
    
    // We use internal charge pump
    i2cWriteByte( SSD1306_CHARGEPUMP, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x14, 						SSD1306_DEFAULT_ADDRESS, 0x00 );
    
    // Horizontal memory mode
    i2cWriteByte( SSD1306_MEMORYMODE, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x00, 						SSD1306_DEFAULT_ADDRESS, 0x00 );
    
    i2cWriteByte( SSD1306_SEGREMAP | 0x1, 		SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_COMSCANDEC, 			SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_SETCOMPINS, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x02, 						SSD1306_DEFAULT_ADDRESS, 0x00 );

    // Max contrast
    i2cWriteByte( SSD1306_SETCONTRAST, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x8F, 						SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_SETPRECHARGE, 		SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0xF1, 						SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_SETVCOMDETECT, 		SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( 0x40, 						SSD1306_DEFAULT_ADDRESS, 0x00 );

    i2cWriteByte( SSD1306_DISPLAYALLON_RESUME, 	SSD1306_DEFAULT_ADDRESS, 0x00 );

    // Non-inverted display
    i2cWriteByte( SSD1306_NORMALDISPLAY, 		SSD1306_DEFAULT_ADDRESS, 0x00 );

    // Turn display back on
    i2cWriteByte( SSD1306_DISPLAYON, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
	
	//Update screen with initial Splash Screen contents
		oled_write_frame();
		
		_delay_ms(5000);
	
	
//	oled_clear_frame();	
//	oled_string( 0, 0, "PenguinPi" );	
//	oled_string( 1, 1, "111111111" );
//	oled_string( 2, 2, "222222222" );
//	oled_string( 3, 3, "333333333" );
//	oled_write_frame();	
}

void oled_clear_frame () {
	
	for ( uint16_t i=0; i < SSD1306_BUFFERSIZE ; i++) {
		oled_frame[i]	= 0;
	}
}

void oled_frame_divider () {
	//When showing LEFT and RIGHT data have a line down screen to divide	
	oled_frame[204]	= 0x33;
	oled_frame[332]	= 0x33;
	oled_frame[460]	= 0x33;
}


void oled_write_frame () {
	
	uint8_t frame_i2c_buffer[16];
	
	i2cWriteByte( SSD1306_COLUMNADDR, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
	i2cWriteByte( 0, 							SSD1306_DEFAULT_ADDRESS, 0x00 );    // Column start address (0 = reset)
	i2cWriteByte( SSD1306_WIDTH-1,    			SSD1306_DEFAULT_ADDRESS, 0x00 );	// Column end address (127 = reset)

	i2cWriteByte( SSD1306_PAGEADDR, 			SSD1306_DEFAULT_ADDRESS, 0x00 );
	i2cWriteByte( 0, 							SSD1306_DEFAULT_ADDRESS, 0x00 ); 	// Page start address (0 = reset)
  
	i2cWriteByte( 3, 							SSD1306_DEFAULT_ADDRESS, 0x00 ); 	// Page end address 		32 => 3, 64 => 7
	
	//Split 512 bytes frame into smaller 16 bytes I2C transactions.
	for (uint16_t i=0; i<SSD1306_BUFFERSIZE/16; i++) {
		
		memcpy( frame_i2c_buffer, &oled_frame[16*i], 16 );
	
		i2cWritenBytes( frame_i2c_buffer, SSD1306_DEFAULT_ADDRESS, 0x40, 16 );	
	}
}

void oled_character( uint8_t x, uint8_t y, char character ) {

	int 		char_in_ascii 	= character;
	uint16_t 	frame_addr 		= x + (128*y);		//128 added as oled_frame is 512 bytes or 128 columns x 4 rows
	
//	sprintf(fstring, "%d\n", char_in_ascii );
//	uart_puts(fstring);	
	
	//Take 5 entries from the ASCII array and put into the frame
		for (int index = 0; index < 5; index++)
		{
			oled_frame[ frame_addr + index ]	= ASCII[ char_in_ascii - 0x20][index];
		}	
		
		//space between characters
		oled_frame[ frame_addr + 5 ] 			= 0x00;		
}

void oled_string   ( uint8_t x, uint8_t y, char *string ) {
	//128 x 32 pixels on screen
	//Characters are 6 pixels by 8
	//Character display is therefore 21 x 4
	//x and y inputs are intended to be on these terms, so y=3 is bottom line of characters
		
	uint8_t local_x = (6*x);
	
	while ( *string ) {
		oled_character( local_x, y, *string++ );	
		
		local_x = local_x + 6;
	}
}

void oled_next_screen ( Hat_oled *oled ) {
	
	if ( oled->show_option >= OLED_ERROR-1 ) {		//Dont increment into OLED_ERROR screen or anything further
		oled->show_option = 0;		
	}
	else {
		oled->show_option = oled->show_option + 1;
//		uart_puts_P("OLED ++\n");		
	}
	
}

void  oled_show_error	( Hat_oled *oled, char *msg ){
	
	uint8_t char_count = 0;
	
	while ( *msg ) {
		if ( char_count>63) {
			//ERROR TOO LONG => ignore rest			
		}
		else if ( char_count<21 ) {
			oled->err_line_1[char_count]	= *msg++;
		}
		else if ( char_count<42 ) {
			oled->err_line_2[char_count-21]	= *msg++;
		}
		else {
			oled->err_line_3[char_count-42]	= *msg++;
		}
		
		char_count++;
	}
	
	oled->show_option = OLED_ERROR;
}

void oled_screen   ( Hat_oled *oled, AnalogIn *vdiv, AnalogIn *csense, Motor *motorA, Motor *motorB, Display *display, uint8_t *datagram, PidController pidA, PidController pidB, uint8_t pid_on, double pid_dt) {
	
	oled_clear_frame();	
	
	switch ( oled->show_option ) {
        case OLED_PID :
            sprintf(fstring, "   A    B  On? %d", pid_on);
            oled_string( 0, 0, fstring);
            oled_string( 0, 0, "   A    B  " ); 

			sprintf(fstring, "e  %d", pidA.error );
            oled_string( 0, 1, fstring);
			sprintf(fstring, "mc %d", pidA.motorCommand );
            oled_string( 0, 2, fstring);
            sprintf(fstring, "de %d", pidA.dt );
            oled_string( 0, 3, fstring);



			sprintf(fstring, "%d", pidB.error );
            oled_string( 8, 1, fstring);
			sprintf(fstring, "%d", pidB.motorCommand );
            oled_string( 8, 2, fstring);
            sprintf(fstring, "%d", pidB.dt );
            oled_string( 8, 3, fstring);

            sprintf(fstring, "dt:%f", pid_dt );
            oled_string( 11, 3, fstring);


            break;

		/* case OLED_DISPLAY :  */
		/* 	oled_string( 0, 0, "DISPLAY" ); */
		/* 	sprintf(fstring, "%d", display->address ); */
		/* 	oled_string ( 0,1,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display->draw ); */
		/* 	oled_string ( 10,1,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display->value ); */
		/* 	oled_string ( 0,2,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display->mode ); */
		/* 	oled_string ( 10,2,fstring ); 				 */
		/* 	sprintf(fstring, "%d", display->digit0 ); */
		/* 	oled_string ( 0,3,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display->digit1 ); */
		/* 	oled_string ( 10,3,fstring ); 	 */
		/*  */
		/* 	break; */
		
		case OLED_BATTERY :
			oled_string( 0, 0, "BATTERY" ); 
			
			oled_string( 0, 1, "V = " );
			oled_string( 0, 2, "I = " );

			sprintf  	( fstring, "%1.3f V\n", vdiv->value);
			oled_string ( 6,1,fstring ); 	
			
			sprintf  	( fstring, "%3.3f mA\n", csense->value);
			oled_string ( 4,2,fstring );			
			
			break;	

		case OLED_MOTORS :
			oled_string( 0, 0, "MOTORS" );
			
			//DIR
			oled_string( 0, 1, "dir:" );
				if ( motorA->dir == -1 ) 		oled_string( 8, 1, " CW" );
				else if ( motorA->dir == 1 )	oled_string( 8, 1, "CCW" );
				else 							oled_string( 8, 1, "OFF" );	

				if ( motorB->dir == -1 ) 		oled_string( 16, 1, " CW" );
				else if ( motorB->dir == 1 )	oled_string( 16, 1, "CCW" );
				else 							oled_string( 16, 1, "OFF" );			
			
			//DPS
			oled_string( 0, 2, "dps:" );
				sprintf(fstring, "%6d", motorA->setSpeedDPS);
				oled_string ( 5,2,fstring );			
			
				sprintf(fstring, "%6d", motorB->setSpeedDPS);
				oled_string ( 13,2,fstring );			
		
			//Divide screen
				oled_frame_divider();
			break;

		case OLED_ENCODERS :
			oled_string( 0, 0, "ENCODERS" );

      //Encoders RAW
			oled_string( 0, 1, "enc1:" );
				sprintf(fstring, "%6d", motorA->enc_raw1);
				oled_string ( 5,1,fstring );			
			
				sprintf(fstring, "%6d", motorB->enc_raw1);
				oled_string ( 13,1,fstring );

			oled_string( 0, 2, "enc2:" );
				sprintf(fstring, "%6d", motorA->enc_raw2);
				oled_string ( 5,2,fstring );			
			
				sprintf(fstring, "%6d", motorB->enc_raw2);
				oled_string ( 13,2,fstring );
        	
			//Divide screen
				oled_frame_divider();				
			break;
      
		case OLED_POSITION :
			oled_string( 0, 0, "POSITION" );
        
			//position
			oled_string( 0, 1, "pos:" );
				sprintf(fstring, "%6d", motorA->position);
				oled_string ( 5,1,fstring );			
			
				sprintf(fstring, "%6d", motorB->position);
				oled_string ( 13,1,fstring );						

			//degrees
			oled_string( 0, 2, "deg:" );
				sprintf(fstring, "%6d", motorA->degrees);
				oled_string ( 5,2,fstring );			
			
				sprintf(fstring, "%6d", motorB->degrees);
				oled_string ( 13,2,fstring );
				
				
			//Divide screen
				oled_frame_divider();				
			break;
			
		case OLED_IP_ADDR :
			oled_string( 0, 0, "IP Addresses" );
			
			oled_string( 0, 1, "eth      .   .   ." );			
				sprintf(fstring, "%3d", oled->eth[0] );
				oled_string (  6,1,fstring );			
				sprintf(fstring, "%3d", oled->eth[1] );
				oled_string ( 10,1,fstring );
				sprintf(fstring, "%3d", oled->eth[2] );
				oled_string ( 14,1,fstring );
				sprintf(fstring, "%3d", oled->eth[3] );
				oled_string ( 18,1,fstring );			
			
			oled_string( 0, 2, "wlan     .   .   ." );
				sprintf(fstring, "%3d", oled->wlan[0] );
				oled_string (  6,2,fstring );			
				sprintf(fstring, "%3d", oled->wlan[1] );
				oled_string ( 10,2,fstring );
				sprintf(fstring, "%3d", oled->wlan[2] );
				oled_string ( 14,2,fstring );
				sprintf(fstring, "%3d", oled->wlan[3] );
				oled_string ( 18,2,fstring );						
			
		
			break;
		/* 	 */
		/* case OLED_DATAGRAM : */
		/* 	oled_string( 0, 0, "Datagram (hex)" ); */
        /*  */
		/* 	sprintf(fstring, "%x", datagram[0] ); */
		/* 	oled_string (  0,1,fstring ); */
		/* 	sprintf(fstring, "%x", datagram[1] ); */
		/* 	oled_string (  5,1,fstring );			 */
		/* 	sprintf(fstring, "%x", datagram[2] ); */
		/* 	oled_string ( 10,1,fstring ); */
		/* 	sprintf(fstring, "%x", datagram[3] ); */
		/* 	oled_string ( 15,1,fstring ); */
        /*  */
		/* 	sprintf(fstring, "%x", datagram[4] ); */
		/* 	oled_string (  0,2,fstring ); */
		/* 	sprintf(fstring, "%x", datagram[5] ); */
		/* 	oled_string (  5,2,fstring );			 */
		/* 	sprintf(fstring, "%x", datagram[6] ); */
		/* 	oled_string ( 10,2,fstring ); */
		/* 	sprintf(fstring, "%x", datagram[7] ); */
		/* 	oled_string ( 15,2,fstring ); */
		/* 	 */
		/* 	sprintf(fstring, "%x", datagram[8] ); */
		/* 	oled_string (  0,3,fstring ); */
		/* 	sprintf(fstring, "%x", datagram[9] ); */
		/* 	oled_string (  5,3,fstring ); */
		/* 	 */
		/* 	break; */
			
		case OLED_ERROR :
			oled_string( 0, 0, "ERROR" );
				sprintf(fstring, "%s", oled->err_line_1 );
				oled_string (  0,1,fstring );			
				sprintf(fstring, "%s", oled->err_line_2 );
				oled_string (  0,2,fstring );
				sprintf(fstring, "%s", oled->err_line_3 );
				oled_string (  0,3,fstring );
			break;
			
		case OLED_SHUTDOWN : 
			oled_string( 0, 0, "SHUTDOWN" );
			break;
	}			
	
	oled_write_frame();
}
