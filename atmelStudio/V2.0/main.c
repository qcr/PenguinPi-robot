/*
 * main.c
 *
 * NOTES
 * ========
 *
 * Motor, set_power() invokes MOTOR_SET_SPEED_DPS which sets motor->dir and motor->setSpeedDPS
 * set_degrees() invokes MOTOR_SET_DEGREES which sets motor->setDegrees which is the PID setpoint
 *
 * LED usage:
 *  red 	- valid command packet
 *  green 	- encoder ISR
 *  blue 	- bad command or framing error
 */

/*
 * cpu is ATmega644PA
 */
#define DEBUG 0 

 
#define BAUD  115200
#define F_CPU 20000000UL

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <util/atomic.h>

#include    "timer.h"
#include "i2cmaster.h"
#include "uart.h"

#include "PenguinPi.h"
#include "hat.h"

#include "global.h"

#define ERRMSGLEN   80

PidController pidA;
PidController pidB;

//Always have
Motor 		motorA;
Motor 		motorB;

LED     leds[6];

AnalogIn 	vdiv;
AnalogIn 	csense;


//HAT dependant
Hat_s		hat;

Display 	displayA;	//remove when parsing logic changed


//PID FLAG
uint8_t    pid_on = 3;

// PID TIMER
double pid_dt = 0;
// system wide flags
uint8_t  oled_update_now = 0;
uint8_t  low_voltage = 0;

//#################################################################################################
//
// ISRs
//
//#################################################################################################

ISR( PCINT0_vect ) {
	//All encoders are now on one PCINT vector
	//Externally on PCINT{3:0} which is PA3:0

	//detect which pin triggered the interrupt
		uint8_t PINA_val   = PINA;
		uint8_t enc_a1_val = ( PINA_val & (1<<MOTOR_A_ENC_1) )>>MOTOR_A_ENC_1; 	//store the current state
		uint8_t enc_a2_val = ( PINA_val & (1<<MOTOR_A_ENC_2) )>>MOTOR_A_ENC_2;
		uint8_t enc_b1_val = ( PINA_val & (1<<MOTOR_B_ENC_1) )>>MOTOR_B_ENC_1; 	//store the current state
		uint8_t enc_b2_val = ( PINA_val & (1<<MOTOR_B_ENC_2) )>>MOTOR_B_ENC_2;
		
	//Update Motor states
		fn_update_motor_states( &motorA, enc_a1_val, enc_a2_val ); 
		fn_update_motor_states( &motorB, enc_b1_val, enc_b2_val ); 
	
	//Update LEDs
        LED_DEBUG_G(5);
}

volatile uint16_t t1_count = 3;
const uint8_t   control_interval = 20; // ms
const uint8_t   adc_interval = 10; // ms
const uint8_t   oled_interval = 200; // ms


// initialize the counters so they are not in sync
volatile uint8_t control_counter = 0;
volatile uint8_t adc_counter = 10;
volatile uint8_t oled_counter = 5;


ISR( TIMER1_COMPA_vect ){
    // period of 1ms

    system_timer++;

    if (++t1_count > 1000) {
        t1_count = 0;
        leds[Y4].state = 1;
        leds[Y4].count = 10;
    }

    // update all LED counters
    for (uint8_t i=0; i<NLEDS; i++) {
        if (leds[i].count > 0)
            if (--leds[i].count == 0)
                leds[i].state = 0;      // flag this LED to be turned off now
    }

    // initiate an OLED update in the main loop
    if (++oled_counter > oled_interval) {
        oled_counter = 0;
        oled_update_now = 1;
    }

    // initiate ADC conversion
    if (++adc_counter > adc_interval) {
        adc_counter = 0;
    }

    // time for motor control?
    if (++control_counter < control_interval)
        return;

    control_counter = 0;
    if (pid_on == 0)
        return;

    // do motor control now

    // enable interrupts and carry on
    sei();  
   
    // Calculate motor speed control
    velocityPIDLoop(motorA.setSpeedDPS * motorA.dir, &motorA, &pidA);
    velocityPIDLoop(motorB.setSpeedDPS * motorB.dir, &motorB, &pidB);

    // Set the PWM values
    OCR0A = mapRanges( abs(pidA.motorCommand), 0, 100, 0, 255 );
    OCR0B = mapRanges( abs(pidB.motorCommand), 0, 100, 0, 255 );

    // Set motor driver polarities
    switch (motorA.dir) {
    case 1:
        PORTB &= ~(1<<MOTOR_A_PHA); break;
    case -1:
        PORTB |= (1<<MOTOR_A_PHA); break;
    case 0:
        OCR0A = 0; break;
    }

    switch (motorB.dir) {
    case 1:
        PORTB &= ~(1<<MOTOR_B_PHA); break;
    case -1:
        PORTB |= (1<<MOTOR_B_PHA); break;
    case 0:
        OCR0B = 0; break;
    }
}

ISR( ADC_vect ) {
    // conversion complete, 42us after initiation
	if(vdiv.count > 1){
		vdiv.count--;	// really this is just a counter to get a few readings before actually using the ADC value
		ADCSRA |= (1<<ADSC);
	}
	else if(vdiv.count == 1){
		vdiv.ready = 1;
		vdiv.count = 0;
	}
	if(csense.count > 1){
		csense.count--;
		ADCSRA |= (1<<ADSC);
	}
	else if(csense.count == 1){
		csense.ready = 1;
		csense.count = 0;
	}
}


//#################################################################################################
//                      Main polling loop
//#################################################################################################

    stats_t  loop_time;
int
main(void)
{
	 
	init_structs();
	init();	
	
	uart_puts_P("PenguinPi v2.0\n");
		
	detect_reset();	
	
	hat_init( &hat );
	
    test_leds();
	
	vdiv.count 	= ADC_COUNT;
	ADCSRA 		|= (1<<ADSC);		//start the first ADC conversion

    // Set up velocity PID
    pidA.kP = 1;
    pidA.motorCommand = 0;

    pidB.kP = 1;
    pidB.motorCommand = 0;

    timer_t t0, tf;

    t0 = timer_get();
    stats_init(&loop_time);
    debugmessage("starting main loop");
    while (1) 
    {
        // get timing stats for main loop
        tf = timer_get();
        stats_add(&loop_time, timer_diff(tf, t0) );
        t0 = tf;

        // Check for a new datagram
        check_datagram();
		
        // Update the hat
        hat_update(&oled_update_now);
		
#ifdef notdef
		//cleanup buffers
		com = 0;
		for(uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) datagramG[j] = 0;
		dgrammem.fl = 0;
#endif

        // Ratio is now 1
		motorA.degrees = motorA.position; // * DEGPERCOUNT;
		motorB.degrees = motorB.position; //  * DEGPERCOUNT;

        if (pid_on == 0) {
            OCR0A = mapRanges( abs(motorA.setSpeedDPS), 0, 100, 0, 255 );
            OCR0B = mapRanges( abs(motorB.setSpeedDPS), 0, 100, 0, 255 );
            
            // Set motor driver polarities
            switch (motorA.dir) {
            case 1:
                PORTB &= ~(1<<MOTOR_A_PHA); break;
            case -1:
                PORTB |= (1<<MOTOR_A_PHA); break;
            case 0:
                OCR0A = 0; break;
            }

            switch (motorB.dir) {
            case 1:
                PORTB &= ~(1<<MOTOR_B_PHA); break;
            case -1:
                PORTB |= (1<<MOTOR_B_PHA); break;
            case 0:
                OCR0B = 0; break;
            }
        }

		//LED update
        for (uint8_t i=0; i<NLEDS; i++) {
            LED *led = &leds[i];

            if(led->state > 0)
                LEDOn(i);
            else
                LEDOff(i);
        }

		//Analog update
		if(vdiv.ready && !(ADCSRA&(1<<ADSC))){
			vdiv.raw   = ADC;
			vdiv.value = vdiv.raw * vdiv.scale/1000;
			vdiv.ready = 0;

			//change multiplexer to csense
			ADMUX 		|= (1<<MUX0);//change mux
			csense.count = ADC_COUNT;
			ADCSRA 		|= (1<<ADSC); // start conversion
		}
		
		if(csense.ready && !(ADCSRA&(1<<ADSC))){
            // get the result
			csense.raw   = ADC;
			csense.value = csense.raw * csense.scale;
			csense.ready = 0;

			//change multiplexer to vsense
			ADMUX 	  &= ~(1<<MUX0);//change mux
			vdiv.count = ADC_COUNT;
			ADCSRA 	  |= (1<<ADSC); // start conversion
		}

		if(vdiv.value < battery.cutoff){
			if(battery.count < battery.limit) battery.count++;
			else{
                //FIXMEhat_lowvolts();
			}
		}else battery.count = 0;


        /*
        uint32_t  timer_counter_copy = 0;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            if (timer_counter > 100000) {
                timer_counter_copy = timer_counter;
                timer_counter = 0;
            }
        }
        if (timer_counter_copy > 0) {
            debugmessage("timer counter %lu %f", timer_counter_copy, pid_dt);
            debugmessage("mean %lu, var %lu, max %lu", 
                    stats_mean(&loop_time), stats_var(&loop_time), loop_time.max );
           leds[Y2].state = 1;
           leds[Y2].count = 50;
        }
        */
		
        //main_loop_debug();
    }
}

// central error message handler, sends to serial port and OLED
void errmessage(const char *fmt, ...)
{
    va_list ap;
    char    buf[ERRMSGLEN];

    va_start(ap, fmt);

    vsnprintf(buf, ERRMSGLEN, fmt, ap);
    uart_puts("ERR: ");
    uart_puts(buf);
    uart_puts("\n");

    hat_show_error(buf);

    va_end(ap);
}

void debugmessage(const char *fmt, ...)
{
    va_list ap;
    char    buf[ERRMSGLEN];

    va_start(ap, fmt);

    vsnprintf(buf, ERRMSGLEN, fmt, ap);
    uart_puts("DBG: ");
    uart_puts(buf);
    uart_puts("\n");

    //hat_show_error( buf);

    va_end(ap);
}

//#################################################################################################
//
// INITs
//
//#################################################################################################

void init_structs(void){
	float kP 				= 70.0;
	float kI 				= 0.0075;
	float kD 				= 2.0;
	
	motorA.which_motor		= 0;
	motorA.gainP 			= kP*PID_SCALE;
	motorA.gainI 			= kI*PID_SCALE;
	motorA.gainD 			= kD*PID_SCALE;
	motorA.maxError 		= INT16_MAX / (motorA.gainP + 1);
	motorA.maxErrorSum 		= (INT32_MAX / 2) / (motorA.gainI + 1);
	motorA.encoderMode 		= 0;
	
	motorB.which_motor		= 1;
	motorB.gainP 			= kP*PID_SCALE;
	motorB.gainI 			= kI*PID_SCALE;
	motorB.gainD 			= kD*PID_SCALE;
	motorB.maxError 		= INT16_MAX / (motorB.gainP + 1);
	motorB.maxErrorSum 		= (INT32_MAX / 2) / (motorB.gainI + 1);
	motorB.encoderMode 		= 0;
	
	vdiv.count 				= 0;
	vdiv.scale 				= 16.018497;//mV per div
	csense.count 			= 0;
	csense.scale 			= 3.2226562;//mA per div
	
	battery.cutoff 			= 6.5;
	battery.count			= 0;
	battery.limit 			= 5000;//about 1.5 seconds
	
    for (uint8_t i=0; i<NLEDS; i++) {
        leds[i].state = 0;
        leds[i].count = 0;
    }
	
}

void init(void){
//V2.0		//Power reduction register
//V2.0		PRR0 &= ~((1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRADC));

    cli();  // just to be sure

// DDR = 1 output pin
// DDR = 0 input pin (default)
	//UART
		uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(BAUD, F_CPU));

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

#ifdef notdef
void main_loop_debug()
{
        if (debug_loop_count++ > 1000) {
            debug_loop_count = 0;
			uart_puts_P("*");
        }
		if ( DEBUG==1 ) {
		
			if ( debug_loop_count==20 ) {
				
				debug_loop_count = 0;
				oled_next_screen ( &oled );			//remove irq from issue
			}
			else debug_loop_count ++ ;
			
		
			//to UART
				//OLed option
					sprintf(fstring, "oled: %d\n", oled.show_option );
					uart_puts(fstring);				
			
				//Voltage
					sprintf(fstring, "Voltage: %5.3f V\n", vdiv.value);
					uart_puts(fstring);				
				
				//Current
					sprintf(fstring, "Current: %5.3f mA\n", csense.value);
					uart_puts(fstring);	
			
				//Motors
					uart_puts_P("M:A\n");
						fn_dbg_motor ( &motorA );					
					uart_puts_P("M:B\n");
						fn_dbg_motor ( &motorB );				
						
				//Timers
					sprintf(fstring, "OC MA: %5d\n", OCR0B );	//Motor A on OCR0B
					uart_puts(fstring);			
					sprintf(fstring, "OC MB: %5d\n", OCR0A ); 	//Motor B on OCR0A
					uart_puts(fstring);		

				//Display
					sprintf(fstring, "DSP addr: %d : ", displayA.address );
					uart_puts(fstring);				
					sprintf(fstring, "draw: %d : ", displayA.draw );
					uart_puts(fstring);			
					sprintf(fstring, "value: %d : ", displayA.value );
					uart_puts(fstring);	
					sprintf(fstring, "dig0: %d : ", displayA.digit0 );
					uart_puts(fstring);	
					sprintf(fstring, "dig1: %d : ", displayA.digit1 );
					uart_puts(fstring);	
					sprintf(fstring, "mode: %d\n", displayA.mode );
					uart_puts(fstring);	

					i2cWriteByte( displayA.digit0, PCA6416A_0, 0x02 );									

					if ( i2c_check_device( I2C_IMU+I2C_WRITE ) == 0 ){
						uart_puts_P("IMU Present\n");
					}	
					else {
						uart_puts_P("IMU Not found\n");
					}					
		}
}
#endif

void
test_leds()
{
//TESTS
//	//LEDs
//		//Y0	C2
//		//Y1	C3
//		//Y2	C5

    for (uint8_t i=0; i<NLEDS; i++) {
        LEDOn(i);
        _delay_ms(300);	
        LEDOff(i);
        _delay_ms(200);	
    }
    /*
    PORTC = 0x2C;
    _delay_ms(500);			
    PORTC = 0x00;
    _delay_ms(500);	

    PORTC = PORTC | (1 << 2);
    _delay_ms(500);		
    PORTC = PORTC | (1 << 3);
    _delay_ms(500);		
    PORTC = PORTC | (1 << 5);
    _delay_ms(500);		
    */

    /*
	//RGB
	redLEDPercent(50);	
	_delay_ms(500);
	redLEDPercent(100);	
	_delay_ms(500);
	redLEDPercent(0);	
	greenLEDPercent(50);
	_delay_ms(500);
	greenLEDPercent(100);
	_delay_ms(500);	
	greenLEDPercent(0);
	blueLEDPercent(50);
	_delay_ms(500);
	blueLEDPercent(100);
	_delay_ms(500);	
	blueLEDPercent(0);
    */
}
