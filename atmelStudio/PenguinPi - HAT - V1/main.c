/*
 * PenguinPi - HAT - V1.c
 *
 * Created: 19/09/2016 19:19:41
 * Author : Jack
 */ 

/*
 * NOTES
 * ========
 *
 * Motor, set_power() invokes MOTOR_SET_SPEED_DPS which sets motor->dir and motor->setSpeedDPS
 * set_degrees() invokes MOTOR_SET_DEGREES which sets motor->setDegrees which is the PID setpoint
 *
 * LED usage:
 *  red - valid command packet
 *  green - encoder ISR
 *  blue - bad command or framing error
 */

/*
 * cpu is ATmega 328PB
 */
#define BAUD 115200
#define F_CPU 12000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <util/crc16.h>
#include "i2cmaster.h"
#include "uart.h"
#include "PenguinPi.h"
#include "PCA6416A.h"


Motor motorA;
Motor motorB;
Servo servoA;
Servo servoB;
LED ledR;
LED ledG;
LED ledB;
Display displayA;
Button buttonA;
Button buttonB;
Button buttonC;
AnalogIn vdiv;
AnalogIn csense;

//Global variables
char fstring[32];
uint8_t datagramG[DGRAM_MAX_LENGTH+1];
int8_t mAQuadTable[4][4] = {{ 0, +1, -1,  2},
							{-1,  0,  2, +1},
							{+1,  2,  0, -1},
							{ 2, -1, +1,  0}};

int8_t mBQuadTable[4][4] = {{ 0, -1, +1,  2},
							{+1,  0,  2, -1},
							{-1,  2,  0, +1},
							{ 2, +1, -1,  0}};
//digit strings for display: includes hexadecimal only
uint8_t digit0[] = {DIGIT0_0, DIGIT0_1, DIGIT0_2, DIGIT0_3, DIGIT0_4, DIGIT0_5, DIGIT0_6, DIGIT0_7, DIGIT0_8, DIGIT0_9, DIGIT0_A, DIGIT0_B, DIGIT0_C, DIGIT0_D, DIGIT0_E, DIGIT0_F};
uint8_t digit1[] = {DIGIT1_0, DIGIT1_1, DIGIT1_2, DIGIT1_3, DIGIT1_4, DIGIT1_5, DIGIT1_6, DIGIT1_7, DIGIT1_8, DIGIT1_9, DIGIT1_A, DIGIT1_B, DIGIT1_C, DIGIT1_D, DIGIT1_E, DIGIT1_F};
//Battery struct
struct Battery {
	float cutoff;//volts
	uint16_t count;
	uint16_t limit;//number of cycles until it triggers a shutdown
} battery;

ISR(PCINT1_vect){
	//Motor A encoders
	//detect which pin triggered the interrupt
	uint8_t enc1val = (PINC & (1<<MOTOR_A_ENC_1))>>MOTOR_A_ENC_1; //store the current state
	uint8_t enc2val = (PINC & (1<<MOTOR_A_ENC_2))>>MOTOR_A_ENC_2;
	
	if(motorA.encoderMode == 0){
		//single encoder mode, on pin 1
		//uint8_t encdiff = motorA.enc1PinState ^ enc1val;
		if(motorA.enc1PinState ^ enc1val){
			if(motorA.dir == 1){
				motorA.position++;
				motorA.lastDir = 1;
			}else if(motorA.dir == -1){
				motorA.position--;
				motorA.lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motorA.lastDir == 1) motorA.position++;
				else if(motorA.lastDir == -1) motorA.position--;
			}
			motorA.enc1PinState = enc1val;
		}//otherwise there was a tick but it wasn't the first channel...

	}else if(motorA.encoderMode == 1){
		//standard quadrature
		uint8_t lastEncSum = (motorA.enc1PinState<<1)|(motorA.enc2PinState);
		uint8_t encSum = (enc1val<<1)|(enc2val);
		int8_t effect = mAQuadTable[lastEncSum][encSum];

		motorA.position += effect;

		motorA.enc1PinState = enc1val;
		motorA.enc2PinState = enc2val;

	}else if(motorA.encoderMode == 2){
		//x4 counting (xor'ed both channels)
		uint8_t x4 = enc1val ^ enc2val;
		if(motorA.enc1PinState ^ x4){
			if(motorA.dir == 1){
				motorA.position++;
				motorA.lastDir = 1;
			}else if(motorA.dir == -1){
				motorA.position--;
				motorA.lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motorA.lastDir == 1) motorA.position++;
				else if(motorA.lastDir == -1) motorA.position--;
			}
			motorA.enc1PinState = x4;
		}
	}else{
		//my mode isn't specified
		motorA.encoderMode = 1;//set to default
	}

	ledG.state = 1;
	ledG.count = 100;
}

ISR(PCINT3_vect){
	//Motor B encoders
	//detect which pin triggered the interrupt
	uint8_t enc1val = (PINE & (1<<MOTOR_B_ENC_1))>>MOTOR_B_ENC_1; //store the current state
	uint8_t enc2val = (PINE & (1<<MOTOR_B_ENC_2))>>MOTOR_B_ENC_2;
	
	if(motorB.encoderMode == 0){
		//single encoder mode, on pin 1
		//uint8_t encdiff = motorB.enc1PinState ^ enc1val;
		if(motorB.enc1PinState ^ enc1val){
			if(motorB.dir == 1){
				motorB.position++;
				motorB.lastDir = 1;
			}else if(motorB.dir == -1){
				motorB.position--;
				motorB.lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motorB.lastDir == 1) motorB.position++;
				else if(motorB.lastDir == -1) motorB.position--;
			}
			motorB.enc1PinState = enc1val;
		}//otherwise there was a tick but it wasn't the first channel...

	}else if(motorB.encoderMode == 1){
		//standard quadrature
		uint8_t lastEncSum = (motorB.enc1PinState<<1)|(motorB.enc2PinState);
		uint8_t encSum = (enc1val<<1)|(enc2val);
		int8_t effect = mBQuadTable[lastEncSum][encSum];

		motorB.position += effect;

		motorB.enc1PinState = enc1val;
		motorB.enc2PinState = enc2val;

	}else if(motorB.encoderMode == 2){
		//x4 counting (xor'ed both channels)
		uint8_t x4 = enc1val ^ enc2val;
		if(motorB.enc1PinState ^ x4){
			if(motorB.dir == 1){
				motorB.position++;
				motorB.lastDir = 1;
			}else if(motorB.dir == -1){
				motorB.position--;
				motorB.lastDir = -1;
			}else{
				//wheel slip!!
				//probably going to still be rotating in the direction it just was, so use that past value
				if(motorB.lastDir == 1) motorB.position++;
				else if(motorB.lastDir == -1) motorB.position--;
			}
			motorB.enc1PinState = x4;
		}
	}else{
		//my mode isn't specified
		motorB.encoderMode = 1;//set to default
	}

	ledG.state = 1;
	ledG.count = 100;
}

ISR(PCINT0_vect){
	uint8_t btnAval = (PINB & (1<<BTN_A))>>BTN_A;

	buttonLogic(&buttonA, btnAval);
}

ISR(PCINT2_vect){
	//determine which button triggered the interrupt
	uint8_t btnBval = (PIND & (1<<BTN_B))>>BTN_B;
	uint8_t btnCval = (PIND & (1<<BTN_C))>>BTN_C;

	if(buttonB.pinState ^ btnBval){//has it actually changed?
		buttonLogic(&buttonB, btnBval);
	}
	if(buttonC.pinState ^ btnCval){
		buttonLogic(&buttonC, btnCval);
	}
}

ISR(TIMER2_OVF_vect){//period of 21.3333us. use a counter for longer delays
	static uint8_t motorControlCount = 0;
	
	if(motorControlCount < CONTROL_COUNT){//should achieve a period of 64 us
		motorControlCount++;
	}else{
		//set PID flags
		motorA.pidTimerFlag = 1;
		motorB.pidTimerFlag = 1;
		motorControlCount = 0;
	}
	
	if(ledR.count > 0) ledR.count--;
	else ledR.state = 0;
	if(ledG.count > 0) ledG.count--;
	else ledG.state = 0;
	if(ledB.count > 0) ledB.count--;
	else ledB.state = 0;
	
	if(buttonA.debounceCount > 0) buttonA.debounceCount--;
	if(buttonB.debounceCount > 0) buttonB.debounceCount--;
	if(buttonC.debounceCount > 0) buttonC.debounceCount--;
}

ISR(ADC_vect){//period of 69.3333us for a standard ADC read, 2x longer for first
	if(vdiv.count > 1){
		vdiv.count--;//really this is just a counter to get a few readings before actually using the ADC value
		ADCSRA |= (1<<ADSC);
	}else if(vdiv.count == 1){
		vdiv.ready = 1;
		vdiv.count = 0;
	}
	if(csense.count > 1){
		csense.count--;
		ADCSRA |= (1<<ADSC);
	}else if(csense.count == 1){
		csense.ready = 1;
		csense.count = 0;
	}
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

int16_t main(void){
	
	init_structs();
	init();
	init_display();
	detect_reset();
	ledB.state = 1;
	ledB.count = 10000;
	ledR.state = 1;
	ledR.count = 10000;

	uint8_t com;

	vdiv.count = ADC_COUNT;
	ADCSRA |= (1<<ADSC);//start the first ADC conversion
	
    while (1) 
    {
		com = checkBuffer();					
		if(com == STARTBYTE){
			parseDatagram(datagramG);
		}
		//cleanup buffers
		com = 0;
		for(uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) datagramG[j] = 0;
		dgrammem.fl = 0;

//sprintf(fstring, "encA: %6d\n", motorA.position);
//uart1_puts(fstring);
//sprintf(fstring, "encB: %6d\n", motorB.position);
//uart1_puts(fstring);

		motorA.degrees = motorA.position * DEGPERCOUNT;
		motorB.degrees = motorB.position * DEGPERCOUNT;


        /*
         * this is the non-PID code that needs to be run only if motor->controlMode == 0
         */
		//OCR1B = ((motorA.setSpeedDPS-0)*((0xFFFF-0)/(100-0))) + 0;
		OCR1B = mapRanges(abs(motorA.setSpeedDPS), 0, 100, 0, 0xFFFF);

		if(motorA.dir == 1){
			PORTC |= (1<<MOTOR_A_PHA);
			}else if(motorA.dir == -1){
			PORTC &= ~(1<<MOTOR_A_PHA);
			}else{
			OCR1B = 0;
		}

		OCR1A = mapRanges(abs(motorB.setSpeedDPS), 0, 100, 0, 0xFFFF);

		if(motorB.dir == 1){
			PORTC &= ~(1<<MOTOR_B_PHA);
			}else if(motorB.dir == -1){
			PORTC |= (1<<MOTOR_B_PHA);
			}else{
			OCR1A = 0;
		}

        /*
         * this is the PID code that needs to be reinstated if motor->controlMode == 1
         */
		//Motor update		
		/*if(motorA.pidTimerFlag == 1){
			motorA.degrees = motorA.position * DEGPERCOUNT;
			int16_t result = motorPIDControl(motorA.setDegrees, &motorA);
			
			OCR1B = (mapRanges(abs(result), 0, 0xFFFF, MOTOR_PWM_RANGE_MIN, 0xFFFF));
			if(result > 0){
				motorA.dir = 1;
				//PORTC &= ~(1<<MOTOR_A_PHA);//reversed
				PORTC |= (1<<MOTOR_A_PHA);
			}else if(result < 0){
				motorA.dir = -1;
				//PORTC |= (1<<MOTOR_A_PHA);//reversed
				PORTC &= ~(1<<MOTOR_A_PHA);
			}else{
				motorA.dir = 0;
				OCR1B = 0;
			}
			motorA.pidTimerFlag = 0;
		}
		if(motorB.pidTimerFlag == 1){
			motorB.degrees = motorB.position * DEGPERCOUNT;
			int16_t result = motorPIDControl(motorB.setDegrees, &motorB);
			
			OCR1A = (mapRanges(abs(result), 0, 0xFFFF, MOTOR_PWM_RANGE_MIN, 0xFFFF));
			if(result > 0){
				motorB.dir = 1;
				//PORTC |= (1<<MOTOR_B_PHA);//reversed
				PORTC &= ~(1<<MOTOR_B_PHA);
			}else if(result < 0){
				motorB.dir = -1;
				//PORTC &= ~(1<<MOTOR_B_PHA);//reversed
				PORTC |= (1<<MOTOR_B_PHA);
			}else{
				motorB.dir = 0;
				OCR1A = 0;
			}
			motorB.pidTimerFlag = 0;
		}*/
		
		//Servo update
		if(servoA.state){
			if(servoA.setPos < servoA.minRange) servoA.setPos = servoA.minRange;//clip the position to within bounds
			if(servoA.setPos > servoA.maxRange) servoA.setPos = servoA.maxRange;
			OCR3A = MAP(servoA.setPos, servoA.minRange, servoA.maxRange, servoA.minPWM, servoA.maxPWM);//bad stuff happens when data types are to small
			//debug
			//sprintf(fstring, "ServoA: %ld deg\n", servoA.setPos);
			//uart1_puts(fstring);
			//sprintf(fstring, "Mapped: %ld tic\n", MAP(servoA.setPos, servoA.minRange, servoA.maxRange, servoA.minPWM, servoA.maxPWM));
			//uart1_puts(fstring);
			//displayA.value = servoA.setPos/10;
		}else{
			PORTD &= ~(1<<SERVO_A);
		}
		if(servoB.state){
			if(servoB.setPos < servoB.minRange) servoB.setPos = servoB.minRange;
			if(servoB.setPos > servoB.maxRange) servoB.setPos = servoB.maxRange;
			OCR4A = MAP(servoB.setPos, servoB.minRange, servoB.maxRange, servoB.minPWM, servoB.maxPWM);
		}else{
			PORTD &= ~(1<<SERVO_B);
		}
				

		//Display update
		if(displayA.draw){
			update_dd7s(&displayA);
			displayA.draw = 0;
		}
		
						
		//LED update
		if(ledR.state > 0){
			if(ledR.brightness > 0) redLEDPercent(ledR.brightness);
			else LEDOn(LED_R);
		} else LEDOff(LED_R);

		if(ledG.state > 0){
			if(ledG.brightness > 0) greenLEDPercent(ledG.brightness);
			else LEDOn(LED_G);
		} else LEDOff(LED_G);

		if(ledB.state > 0){ 
			if(ledB.brightness > 0) blueLEDPercent(ledB.brightness);
			else LEDOn(LED_B);
		} else LEDOff(LED_B);
		

		//Button update
		if(buttonA.state == 1){
			ledR.state = 1;
			ledR.count = 2000;
// 			displayA.value--;
// 			displayA.draw = 1;
			
			uart1_puts_P("This is a shutdown request");

			motorA.position = 0;
			motorA.setDegrees = 180;
			motorA.dir = -1;
			motorB.position = 0;
			motorB.setDegrees = 180;
			motorB.dir = 1;
			if(buttonA.programMode == 0) buttonA.state = 0;
		}else{
			
		}
		if(buttonB.state == 1){
			ledG.state = 1;
			ledG.count = 2000;
// 			displayA.value = 0;
// 			displayA.draw = 1;

			motorA.position = 0;
			motorA.setSpeedDPS = motorA.setSpeedDPS +10;
			motorA.dir = -1;
			motorB.position = 0;
			motorB.setDegrees = -180;
			motorB.dir = -1;
			if(buttonB.programMode == 0) buttonB.state = 0;
		}else{
			
		}
		if(buttonC.state == 1){
			ledB.state = 1;
			ledB.count = 2000;
// 			displayA.value++;
// 			displayA.draw = 1;

			motorA.position = 0;
			motorA.setSpeedDPS = motorA.setSpeedDPS -10;
			motorA.dir = 1;
			motorB.position = 0;
			motorB.setDegrees = -180;
			motorB.dir = -1;
			if(buttonC.programMode == 0) buttonC.state = 0;
		}else{
			
		}

		//Analog update
		if(vdiv.ready && !(ADCSRA&(1<<ADSC))){
			vdiv.raw = ADC;
			vdiv.value = vdiv.raw * vdiv.scale/1000;
			vdiv.ready = 0;

			//change multiplexer to csense
			ADMUX |= (1<<MUX0);//change mux
			csense.count = ADC_COUNT;
			ADCSRA |= (1<<ADSC);//restart

			//sprintf(fstring, "Voltage: %5.3f V\n", vdiv.value);
			//uart1_puts(fstring);
		}
		if(csense.ready && !(ADCSRA&(1<<ADSC))){
			csense.raw = ADC;
			csense.value = csense.raw * csense.scale;
			csense.ready = 0;

			//change multiplexer to csense
			ADMUX &= ~(1<<MUX0);//change mux
			vdiv.count = ADC_COUNT;
			ADCSRA |= (1<<ADSC);//restart

			//sprintf(fstring, "Current: %5.3f mA\n", csense.value);
			//uart1_puts(fstring);
		}

		if(vdiv.value < battery.cutoff){
			if(battery.count < battery.limit) battery.count++;
			else{
				//lockout most functions
				PRR0 |= (1<<PRTIM2)|(1<<PRTIM0)|(1<<PRTIM1);//|(1<<PRADC);
				//display low battery warning
				uint8_t reg[2] = {0, 0};
				reg[0] = DIGIT0_B;
				reg[1] = DIGIT1_F;
				i2cWritenBytes(reg, displayA.address, OUTPUT_0, 2);
								
				while(1){
					//send shutdown command
					uart1_puts_P("Low battery shutdown request");
					PORTB &= ~(1<<PB5);//pull the pin low
					//loop until the battery really is flat...
				}			
			}
		}else battery.count = 0;
    }
}

void init_structs(void){
	float kP = 70.0;
	float kI = 0.0075;
	float kD = 2.0;

	motorA.gainP = kP*PID_SCALE;
	motorA.gainI = kI*PID_SCALE;
	motorA.gainD = kD*PID_SCALE;
	motorA.maxError = INT16_MAX / (motorA.gainP + 1);
	motorA.maxErrorSum = (INT32_MAX / 2) / (motorA.gainI + 1);
	motorA.encoderMode = 1;

	motorB.gainP = kP*PID_SCALE;
	motorB.gainI = kI*PID_SCALE;
	motorB.gainD = kD*PID_SCALE;
	motorB.maxError = INT16_MAX / (motorB.gainP + 1);
	motorB.maxErrorSum = (INT32_MAX / 2) / (motorB.gainI + 1);
	motorB.encoderMode = 1;

	servoA.setPos = 90;
	servoA.minRange = 0;
	servoA.maxRange= 180;
	servoA.minPWM = SERVO_PWM_RANGE_MIN;
	servoA.maxPWM = SERVO_PWM_RANGE_MAX;

	servoB.setPos= 90;
	servoB.minRange = 0;
	servoB.maxRange= 180;
	servoB.minPWM = SERVO_PWM_RANGE_MIN;
	servoB.maxPWM = SERVO_PWM_RANGE_MAX;

	displayA.address = PCA6416A_0;
	displayA.value= 0;
	displayA.digit0 = 0;
	displayA.digit1 = 0;
	displayA.draw = 0;

	buttonA.pinMode = 0;
	buttonB.pinMode = 0;
	buttonC.pinMode = 0;
	buttonA.programMode = 0;
	buttonB.programMode = 0;
	buttonC.programMode = 0;

	vdiv.count = 0;
	vdiv.scale = 16.018497;//mV per div
	csense.count = 0;
	csense.scale = 3.2226562;//mA per div

	battery.cutoff = 6.5;
	battery.count= 0;
	battery.limit = 5000;//about 1.5 seconds
}

void init(void){
	//Power reduction register
	PRR0 &= ~((1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRADC));

	//Motor Pins
	DDRB |= (1<<MOTOR_A_PWM)|(1<<MOTOR_B_PWM);
	DDRC |= (1<<MOTOR_A_PHA)|(1<<MOTOR_B_PHA);

	//Motor PWM
	TCCR1A |= (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10); // Non-inverting, 16 bit fast PWM
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); // DIV1 prescaler
	ICR1 = 0xFFFF; //set TOP

	//Encoders
	DDRC &= ~((1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2));
	PORTC |= (1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2); //enable internal pull ups
	PCMSK1 = (1<<PCINT10)|(1<<PCINT11);
	PCICR |= (1<<PCIE1);

	DDRE &= ~((1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2));
	PORTE |= (1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2);
	PCMSK3 = (1<<PCINT24)|(1<<PCINT25);
	PCICR |= (1<<PCIE3);
	
	//Servo
	DDRD |= (1<<SERVO_A)|(1<<SERVO_B);
	TCCR3A |= (1<<COM3A1)|(0<<COM3A0)|(1<<WGM31)|(0<<WGM30); // non-inverting, 15 bit resolution fast PWM
	TCCR3B |= (1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(0<<CS30); // DIV8 prescaler, TOP 20,000 to run at 50 Hz for 20ms pulse
	ICR3 = 0x7530;
	
	TCCR4A |= (1<<COM4A1)|(0<<COM4A0)|(1<<WGM41)|(0<<WGM40); // non-inverting, 15 bit resolution fast PWM
	TCCR4B |= (1<<WGM43)|(1<<WGM42)|(0<<CS42)|(1<<CS41)|(0<<CS40);	// DIV8 prescaler	
	ICR4 = 0x7530;

	//LED's
	TCCR0A |= (1<<COM0A1)|(1<<COM0A0)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00); // inverting, 8 bit fast PWM
	TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00); // DIV64 prescaler, try and run LED's below 1kHz
	
	TCCR2A |= (0<<COM2A1)|(0<<COM2A0)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM21)|(1<<WGM20); // inverting, 8 bit fast PWM, Blue is OC2B
	TCCR2B |= (0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20); //DIV1 prescaler (for control timer)
	
	//Buttons (external pullups)
	DDRB &= ~(1<<BTN_A);
	PCMSK0 |= (1<<PCINT0);

	DDRD &= ~(1<<BTN_B);
	PCMSK2 |= (1<<PCINT23);
	
	DDRD &= ~(1<<BTN_C);
	PCMSK2 |= (1<<PCINT20);
	PCICR |= (1<<PCIE0)|(1<<PCIE2);

	//Pi Interrupt
	DDRB |= (1<<PB5);
	PORTB |= (1<<PB5);

	//8 bit controller timer
	TIMSK2 |= (1<<TOIE2);

	//ADC
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0); //AVCC reference voltage with external cap on AREF, multiplex to channel 6
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);//Enable ADC, enable interrupt, prescaler of 64 (187.5kHz sample rate)
	DIDR0 = (1<<6)|(1<<7);//the iom328pb.h file does not include the ADC6D, 7D defines, and it cannot be saved over. ty atmel
	
	uart1_init(UART_BAUD_SELECT_DOUBLE_SPEED(BAUD, F_CPU));
	i2c_init();

	sei();
}

void init_display(void){
	//initialises the 7 segment display controller
	uint8_t reg[2] = {0, 0};
	i2cWritenBytes(reg, displayA.address, CONFIG_0, 2); //configures both ports as outputs
	i2cWritenBytes(reg, displayA.address, OUTPUT_0, 2); //sets both outputs to 0
}

void detect_reset(void){
	//read MCUSR and determine what reset the AVR
	uint8_t reg = MCUSR;
	MCUSR = 0x00;
	switch (reg){
		case 1:
			uart1_puts_P("Device Reset from Power On Reset\n");
		break;
		case 2:
			uart1_puts_P("Device Reset from External Reset\n");
		break;
		case 4:
			uart1_puts_P("Device Reset from Brown Out Detector\n");
		break;
		case 8:
			uart1_puts_P("Device Reset from Watchdog Timeout\n");
		break;
		default:
		uart1_puts_P("Device turned on\n");
		break;
	}
}


uint16_t mapRanges(uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax){
	return ((a-amin)*((omax-omin)/(amax-amin))) + omin; //maps from scale amin->amax to scale omin->omax
}

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
		OCR0A = MAP(percent, 0, 100, 0, RED_MAX);
	}
}

void greenLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_G);
	}else{
		DDRD |= (1<<LED_G);
		OCR0B = MAP(percent, 0, 100, 0, GREEN_MAX);
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

void update_dd7s(Display *display){//TODO: fix this up
	uint8_t reg[2]= {0, 0}; // MS, LS digits
    uint8_t value = display->value;
    int8_t  svalue;

    switch (display->mode) {
    case 0: // hex mode
        reg[0] = digit0[(value>>4) & 0x0f];
        reg[1] = digit1[value & 0x0f];
        break;
    case 1: // unsigned decimal, light up the decimal point
        if (value > 99)
            value = 99;
        reg[0] = digit0[(uint8_t)value/10];
        reg[1] = digit1[(uint8_t)value%10]|SEGMENTDP_1;
        break;
    case 2: // signed decimal -9 to +9
        svalue = (int8_t) value;
        if (svalue < -9)
            svalue = -9;
        else if (svalue > 9)
            svalue = 9;
        if (svalue < 0)
			reg[0] = SEGMENTG_0;    // minus sign
        reg[1] = digit1[abs(svalue)]|SEGMENTDP_1;
        break;
    }
    /*
	//determine the best way to display the value
	if(display->value != 0xFF){
		displayBase10(reg, display->value);
	}else{

	}
	//if(display->digit0 != 0xFF) reg[0] = digit0[display->digit0];//digit0 and digit1 will override value
	//if(display->digit1 != 0xFF) reg[1] = digit1[display->digit1];
    */

	i2cWritenBytes(reg, display->address, OUTPUT_0, 2);
}

void displayBase10(uint8_t *reg, int16_t value){
	if(value < 0){
		//negative numbers
		if(value > -1){
			//decimal

		}else if(value < -15){
			//too low to display
			reg[0] = SEGMENTG_0;
			reg[1] = DIGIT1_O;
		}else{
			reg[0] = SEGMENTG_0;
			reg[1] = digit1[abs(value)]|SEGMENTDP_1;
		}
	}else{
		//positive numbers
		if(value < 1){
			//decimal
			reg[0] = digit0[(uint8_t)value*10];
		}else if(value < 10){
			//can still display 1 decimal place
		}else if(value > 10){
			//only integers here
			reg[0] = digit0[(uint8_t)value/10];
			reg[1] = digit1[(uint8_t)value%10]|SEGMENTDP_1;
		}
	}
}


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

int8_t i2cWritenBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg); //where to write...
	for(uint8_t i = 0; i < n; i++){
		i2c_write(data[i]);
	}
	i2c_stop();
	return 1;//completed all writes without failure
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

uint8_t checkBuffer(void){
	uint16_t com = uart1_getc();
	
	if(com & UART_NO_DATA){
		// there's no data available
		return 0;
	} else {
		//check for errors
        if(com & UART_FRAME_ERROR){
            /* Framing Error detected, i.e no stop bit detected */
            uart1_puts_P("ERROR: Bad UART Frame\n");
			//flash Blue LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
        if(com & UART_OVERRUN_ERROR){
            /* 
                * Overrun, a character already present in the UART UDR register was 
                * not read by the interrupt handler before the next character arrived,
                * one or more received characters have been dropped
                */
            uart1_puts_P("ERROR: UART Buffer Overrun\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
        if(com & UART_BUFFER_OVERFLOW){
            /* 
                * We are not reading the receive buffer fast enough,
                * one or more received character have been dropped 
                */
            uart1_puts_P("ERROR: UART Buffer Overflow\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
		return com & 0xFF;//return lowbyte
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

void uart1putcs(uint8_t *datagram){
	uart1_putc(STARTBYTE);
	for(uint8_t i = 0; i < datagram[0]; i++){
		uart1_putc(datagram[i]);
	}
	//uart1_putc(STOPBYTE);
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

void parseDatagram(uint8_t *datagram){
	//flash RED LED
	ledR.state = 1;
	ledR.count = 1000;
	
	_delay_us(UART_INTERBYTE_WAIT);
	uint8_t dlen = checkBuffer();
	datagram[0] = dlen; //length of the datagram
	for(uint8_t i = 1; i < dlen; i++){
		_delay_us(UART_INTERBYTE_WAIT);
		
		datagram[i] = checkBuffer();
		if(i >= DGRAM_MAX_LENGTH){
			uart1_puts_P("ERROR: Datagram Buffer Overflow\n");
			ledB.state = 1;
			ledB.count = 1000;
			return;
		}
	}
	uint8_t crcDgram = datagram[dlen-1];
	datagram[dlen-1] = 0;
	uint8_t crcCalc = crc8(datagram, dlen-1);
	datagram[0] -= 1;	

	if(crcCalc != crcDgram){
		uart1_puts_P("ERROR: CRC Failed\n");
		ledB.state = 1;
		ledB.count = 1000;
		return;
	}
	
	switch(datagram[1]){
		case AD_MOTOR_A:
			parseMotorOp(datagram, &motorA);
		break;
		case AD_MOTOR_B:
			parseMotorOp(datagram, &motorB);
		break;
		
		case AD_SERVO_A:
			parseServoOp(datagram, &servoA);
		break;
		case AD_SERVO_B:
			parseServoOp(datagram, &servoB);
		break;
		
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

		case AD_BTN_A:
			parseButtonOp(datagram, &buttonA);
		break;
		case AD_BTN_B:
			parseButtonOp(datagram, &buttonB);
		break;
		case AD_BTN_C:
			parseButtonOp(datagram, &buttonC);
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
			uart1_puts_P("ERROR: Unknown Address\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

/* TODO:
  change the constant in datagram[0] test to a symbolic value
   ISINT
   ISFLOAT
   ISCHAR  etc
 */
void parseMotorOp(uint8_t *datagram, Motor *motor){
	switch(datagram[2]){
		//SETTERS
		case MOTOR_SET_SPEED_DPS:
			if(datagram[0] == 5){
				int16_t speed = (datagram[3]<<8) | datagram[4];
				motor->setSpeedDPS = abs(speed);
				if(speed > 0) motor->dir = 1;
				else if(speed < 0) motor->dir = -1;
				else motor->dir = 0;
				ledB.state = 1;
				ledB.count = 1000;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_DEGREES:
			if(datagram[0] == 5){
                cli();
                    motor->position = 0;
                sei();
                motor->degrees = 0;

				int16_t degrees = (datagram[3]<<8) | datagram[4];
				motor->setDegrees = degrees;
				if(degrees > 0) motor->dir = 1;
				else if(degrees < 0) motor->dir = -1;
				else motor->dir = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
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
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_P:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainP = readFloat(flMem) * PID_SCALE;
				motor->maxError = INT16_MAX / (motor->gainP + 1);				
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_I:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainI = readFloat(flMem) * PID_SCALE;
				motor->maxErrorSum = (INT32_MAX / 2) / (motor->gainI + 1);
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_D:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainD = readFloat(flMem) * PID_SCALE;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_ENC_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode > 2) motor->encoderMode = 1; //the default
				else motor->encoderMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_CONTROL_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				motor->controlMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case MOTOR_GET_SPEED_DPS:
			dgrammem.in = motor->speedDPS;
			formdatagram(datagramG, datagram[1], MOTOR_SET_SPEED_DPS, dgrammem, 'i');
			uart1putcs(datagramG);
		break;	
		case MOTOR_GET_DEGREES:
			dgrammem.in = motor->degrees;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DEGREES, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_DIRECTION:
			dgrammem.ch = motor->dir;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DIRECTION, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_GAIN_P:
			dgrammem.fl = motor->gainP/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_P, dgrammem, 'f');
			uart1putcs(datagramG);
		break;	
		case MOTOR_GET_GAIN_I:
			dgrammem.fl = motor->gainI/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_I, dgrammem, 'f');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_GAIN_D:
			dgrammem.fl = motor->gainD/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_D, dgrammem, 'f');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_ENC_MODE:
			dgrammem.ch = motor->encoderMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC_MODE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_CONTROL_MODE:
			dgrammem.ch = motor->controlMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_CONTROL_MODE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case MOTOR_GET_ENC:
            cli();
                dgrammem.in = motor->position;
            sei();
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		
		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;		
	}
}

void parseServoOp(uint8_t *datagram, Servo *servo){
	switch(datagram[2]){
		//SETTERS
		case SERVO_SET_POSITION:
			if(datagram[0] == 5){
				int16_t pos = (datagram[3]<<8)|datagram[4];
				if(pos > servo->maxRange) pos = servo->maxRange;
				else if(pos < servo->minRange) pos = servo->minRange;
				servo->setPos = pos;
				servo->state = 1;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case SERVO_SET_STATE:
			if(datagram[0] == 4){
				int8_t state = datagram[3];
				if(state >= 1) servo->state = 1;
				else servo->state = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case SERVO_SET_MIN_RANGE:
			if(datagram[0] == 5){
				int16_t min = (datagram[3]<<8)|datagram[4];
				if(min < 0) min = 0;
				if(min > 360) min = 360;
				if(min > servo->maxRange) min = servo->maxRange;
				servo->minRange = min;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case SERVO_SET_MAX_RANGE:
			if(datagram[0] == 5){
				int16_t max = (datagram[3]<<8)|datagram[4];
				if(max < 0) max = 0;
				if(max > 360) max = 360;
				if(max < servo->minRange) max = servo->minRange;
				servo->maxRange = max;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case SERVO_SET_MIN_PWM:
			if(datagram[0] == 5){
				uint16_t min = (datagram[3]<<8)|datagram[4];
				if(min < 0) min = 0;
				if(min > 30000) min = 30000;
				if(min > servo->maxPWM) min = servo->maxPWM;
				servo->minPWM = min;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case SERVO_SET_MAX_PWM:
			if(datagram[0] == 5){
				uint16_t max = (datagram[3]<<8)|datagram[4];
				if(max < 0) max = 0;
				if(max > 30000) max = 30000;
				if(max < servo->minPWM) max = servo->minPWM;
				servo->maxPWM = max;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;

		//GETTERS
		case SERVO_GET_POSITION:
			dgrammem.in = servo->setPos;
			formdatagram(datagramG, datagram[1], SERVO_SET_POSITION, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case SERVO_GET_STATE:
			dgrammem.ch = servo->state;
			formdatagram(datagramG, datagram[1], SERVO_SET_STATE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case SERVO_GET_MIN_RANGE:
			dgrammem.in = servo->minRange;
			formdatagram(datagramG, datagram[1], SERVO_SET_MIN_RANGE, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case SERVO_GET_MAX_RANGE:
			dgrammem.in = servo->maxRange;
			formdatagram(datagramG, datagram[1], SERVO_SET_MAX_RANGE, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case SERVO_GET_MIN_PWM:
			dgrammem.in = servo->minPWM;
			formdatagram(datagramG, datagram[1], SERVO_SET_MIN_PWM, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case SERVO_GET_MAX_PWM:
			dgrammem.in = servo->maxPWM;
			formdatagram(datagramG, datagram[1], SERVO_SET_MAX_PWM, dgrammem, 'i');
			uart1putcs(datagramG);
		break;

		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
	
}

void parseLEDOp(uint8_t *datagram, LED *led){
	switch(datagram[2]){
		//SETTERS
		case LED_SET_STATE:
			if(datagram[0] == 4){
				int8_t state = datagram[3];
				if(state >= 1) led->state = 1;
				else led->state = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case LED_SET_BRIGHTNESS:
			if(datagram[0] == 4){
				int8_t brightness = datagram[3];
				if(brightness > 100) led->brightness = 100;
				else if(brightness > 0) led->brightness = brightness;
				else led->brightness = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case LED_SET_COUNT:
			if(datagram[0] == 5){
				led->count = (datagram[3]<<8)|datagram[4];
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;

		//GETTERS
		case LED_GET_STATE:
			dgrammem.ch = led->state;
			formdatagram(datagramG, datagram[1], LED_SET_STATE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case LED_GET_BRIGHTNESS:
			dgrammem.ch = led->brightness;
			formdatagram(datagramG, datagram[1], LED_SET_BRIGHTNESS, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case LED_GET_COUNT:
			dgrammem.uin = led->count;
			formdatagram(datagramG, datagram[1], LED_SET_COUNT, dgrammem, 'i');
			uart1putcs(datagramG);
		break;

		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseDisplayOp(uint8_t *datagram, Display *display){
	switch(datagram[2]){
		//SETTERS
		case DISPLAY_SET_VALUE:
			if(datagram[0] == 4){
				display->value = datagram[3];
				display->draw = 1;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case DISPLAY_SET_DIGIT_0:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit0 = 15;
				else if(digit < -15) display->digit0 = -15;
				else display->digit0 = digit;
				display->draw = 1;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case DISPLAY_SET_DIGIT_1:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit1 = 15;
				else if(digit < -15) display->digit1 = -15;
				else display->digit1 = digit;
				display->draw = 1;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
        case DISPLAY_SET_MODE:
			if(datagram[0] == 4){
				display->mode = datagram[3];
				display->draw = 1;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case DISPLAY_GET_VALUE:
			dgrammem.ch = display->value;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_VALUE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_0:
			dgrammem.ch = display->digit0;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_0, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_1:
			dgrammem.ch = display->digit1;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_1, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case DISPLAY_GET_MODE:
			dgrammem.ch = display->mode;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_MODE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		
		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseButtonOp(uint8_t *datagram, Button *btn){
	switch(datagram[2]){
		//SETTERS
		case BUTTON_SET_PROGRAM_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode == 1) btn->programMode = 1;
				else if(mode == 0) btn->programMode = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case BUTTON_SET_PIN_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode == 1) btn->pinMode = 1;
				else if(mode == 0) btn->pinMode = 0;
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case BUTTON_GET_PROGRAM_MODE:
			dgrammem.ch = btn->programMode;
			formdatagram(datagramG, datagram[1], BUTTON_SET_PROGRAM_MODE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		case BUTTON_GET_PIN_MODE:
			dgrammem.ch = btn->pinMode;
			formdatagram(datagramG, datagram[1], BUTTON_SET_PIN_MODE, dgrammem, 'c');
			uart1putcs(datagramG);
		break;
		
		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseADCOp(uint8_t *datagram, AnalogIn *adc){
	switch(datagram[2]){
		//SETTERS
		case ADC_SET_SCALE:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				adc->scale = readFloat(flMem);
			}else{
				uart1_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case ADC_GET_SCALE:
			dgrammem.fl = adc->scale;
			formdatagram(datagramG, datagram[1], ADC_SET_SCALE, dgrammem, 'f');
			uart1putcs(datagramG);
		break;
		case ADC_GET_RAW:
			dgrammem.in = adc->raw;
			formdatagram(datagramG, datagram[1], ADC_SET_RAW, dgrammem, 'i');
			uart1putcs(datagramG);
		break;
		case ADC_GET_READING:
			dgrammem.fl = adc->value;
			formdatagram(datagramG, datagram[1], ADC_SET_READING, dgrammem, 'f');
			uart1putcs(datagramG);
		break;
		
		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseAllOp(uint8_t *datagram){
	switch(datagram[2]){
		case ALL_STOP:
			motorA.position = 0;
			motorA.setDegrees = 0;
			motorA.setSpeedDPS = 0;
			motorB.position = 0;
			motorB.setDegrees = 0;
			motorB.setSpeedDPS = 0;
			servoA.state = 0;
			servoB.state = 0;
			displayA.draw = 0;
			ledR.state = 0;
			ledG.state = 0;
			ledB.state = 0;
		break;
		case CLEAR_DATA:
			motorA = (Motor){0};
			motorB = (Motor){0};
			servoA = (Servo){0};
			servoB = (Servo){0};
			ledR = (LED){0};
			ledG = (LED){0};
			ledB = (LED){0};
			displayA = (Display){0};
			buttonA = (Button){0};
			buttonB = (Button){0};
			buttonC = (Button){0};
			vdiv = (AnalogIn){0};
			csense = (AnalogIn){0};
		break;
		
		default:
			uart1_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
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
