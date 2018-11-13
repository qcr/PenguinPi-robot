 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "uart.h"

#include "io.h"


void io_analog_filter_step(AnalogIn *chan, uint16_t value)
{
    // convert to physical unit
    chan->value = value * chan->scale;
    
    if (chan->initialized == 0) {
        chan->initialized = 1;
        chan->smooth = chan->value;
    } else 
        // apply first-order digital filter
        chan->smooth = chan->alpha * chan->smooth 
                     + (1.0 - chan->alpha) * chan->value;
}

void io_init(void)
{
//V2.0		//Power reduction register
//V2.0		PRR0 &= ~((1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRADC));

    cli();  // just to be sure

// DDR = 1 output pin
// DDR = 0 input pin (default)

	//Motor Pins
		DDRB |= (1<<MOTOR_A_PWM) | (1<<MOTOR_B_PWM);
		DDRB |= (1<<MOTOR_A_PHA) | (1<<MOTOR_B_PHA);

	//Motor PWM
		//V1 was OC1A and OC1B
		//V2  is OC0A and OC0B
		TCCR0A |= (1<<COM0A1)|(0<<COM0A0)| // Clear OC0A on Compare Match	Set OC0A on Bottom
				  (1<<COM0B1)|(0<<COM0B0)| // Clear OC0B on Compare Match	Set OC0B on Bottom
				  (1<<WGM01) |(1<<WGM00);  // Non-inverting, 8 bit fast PWM
				  
		TCCR0B |= (0<<WGM02) |
				  (0<<CS02)|(0<<CS01)|(1<<CS00);	// DIV1 prescaler


	//Encoders
		//V1 was PC2, PC3, PE0, PE1 ... PCINT 10,11,24,25
		//V2  is PA0, PA1, PA2, PA3 ... PCINT 0:3
		DDRA 	&= ~((1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2));		//Direction to INPUT
		PORTA 	|=   (1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2);		//Enable internal PULLUPs
		PCMSK0   =   (1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3);									//Enable Pin Change Mask
		PCICR 	|=   (1<<PCIE0);																		//Enable interrupt on pin change
			
	//LED's
		DDRC 	|= 0x2C;	//LEDs on C5, 3:2
		DDRD 	|= 0xE0;	//RGB on D7:5

        // turn all LEDs off
    for (uint8_t i=0; i<NLEDS; i++)
        LEDOff(i);

#ifdef notdef
		//GREEN ON A
			TCCR1A |= (1<<COM0A1)|(1<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(1<<WGM00); 			// inverting, 8 bit fast PWM
			TCCR1B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00);
		
        // 20MGHz 256/20
        // 0.0000128
		//RED on A and BLUE on B
			TCCR2A |= (1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM21)|(1<<WGM20); 			// inverting, 8 bit fast PWM
			TCCR2B |= (0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20);

        TCCR2A |= (0<<COM2A1)|(0<<COM2A0)|(0<<COM2B1)|(0<<COM2B0)|(0<<WGM21)|(0<<WGM20); 			// inverting, 8 bit fast PWM
        // timer 2 counts at 20MHz (no prescaler)
        TCCR2B |= (0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20);
	//8 bit controller timer
        // enable overflow interrupt
		TIMSK2 |= (1<<TOIE2);
#endif
        
    // millisecond timer, 16 bit timer 1
        TCCR1A |= (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(0<<WGM01)|(0<<WGM00);
        TCCR1B |= (0<<WGM13)|(1<<WGM12)|    // CTC mode
            (0<<CS02)|(0<<CS01)|(1<<CS00);  // no prescaler
        OCR1A = 20000;
		TIMSK1 |= (1<<OCIE1A);  // enable OCA interrupts

	//ADC
		DIDR0	= (1<<ADC6D)|(1<<ADC7D);
		ADMUX 	= (0<<REFS1)|(1<<REFS0)| //Vcc reference voltage with external cap on AREF
            (0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0); 	// multiplex to channel 6
		ADCSRA 	= (1<<ADEN) |  // enable ADC
            (1<<ADIE) |        // enable interrupt
            (1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); // prescaler of 64 (312kHz sample rate)
	
	//I2C
		i2c_init();

	//Enable Interrupts
		sei();
}

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


uint16_t mapRanges(uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax){
	return ((a-amin)*((omax-omin)/(amax-amin))) + omin; //maps from scale amin->amax to scale omin->omax
}

#ifdef notdef
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
#endif


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
void LEDOff(enum _leds led){
    switch (led) {
        case RED:
            DDRD &= ~0x80; break;
        case GREEN:
            DDRD &= ~0x20; break;
        case BLUE:
            DDRD &= ~0x40; break;
        case Y2:
            DDRC &= ~0x04; break;
        case Y3:
            DDRC &= ~0x08; break;
        case Y4:
            DDRC &= ~0x20; break;
        default:
            break;
    }
}

void LEDOn(enum _leds led){
    switch (led) {
        case RED:
            DDRD |= 0x80; break;
        case GREEN:
            DDRD |= 0x20; break;
        case BLUE:
            DDRD |= 0x40; break;
        case Y2:
            DDRC |= 0x04; break;
        case Y3:
            DDRC |= 0x08; break;
        case Y4:
            DDRC |= 0x20; break;
        default:
            break;
    }
}

#ifdef notdef
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
#endif
