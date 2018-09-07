 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "uart.h"

#include "PenguinPi.h"


void analogFilter(AnalogIn *chan, uint16_t value)
{
    // convert to physical unit
    chan->value = value * chan->scale;
    
    // apply first-order digital filter
    chan->smooth = chan->alpha * chan->smooth + (1.0 - chan->alpha) * chan->value;
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
