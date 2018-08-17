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
LED 		ledR;
LED 		ledG;
LED 		ledB;
AnalogIn 	vdiv;
AnalogIn 	csense;

volatile uint32_t    timer_counter;

//HAT dependant
Hat_s		hat;

Display 	displayA;	//remove when parsing logic changed


//PID FLAG
uint8_t    pid_on = 3;

// PID TIMER
volatile uint16_t pid_timer_counter = 0;
double pid_dt = 0;

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
        LED_DEBUG_G(100);
}

#define CONTROL_INTERVAL    1563
volatile uint16_t tcontrol;

ISR( TIMER2_OVF_vect ){
    // period of 12.8us

    // Motor control counter has been moved to the main while loop

    system_timer++;

    pid_timer_counter++;
    timer_counter++;
	
	if(ledR.count > 0) 	ledR.count--;
	else 				ledR.state = 0;
	
	if(ledG.count > 0) 	ledG.count--;
	else 				ledG.state = 0;
	
	if(ledB.count > 0) 	ledB.count--;
	else 				ledB.state = 0;
    //

    if (tcontrol++ < CONTROL_INTERVAL)
        return;
    tcontrol = 0;
    // enable interrupts and carry on
    sei();  
   
    
    // Work out motor speed for either PID or non-PID and then apply after
    
    if (pid_on == 0) {
        OCR0A = mapRanges( abs(motorA.setSpeedDPS), 0, 100, 0, 255 );
        OCR0B = mapRanges( abs(motorB.setSpeedDPS), 0, 100, 0, 255 );
    } else {
        // Only run PID when the timer flag is set (using motorA for both)
        if(motorA.pidTimerFlag == 1){
            // This measures the length of the pid loop
            pid_dt = pid_timer_counter * 0.0000128;
            pid_timer_counter = 0;

            // Calculate a new motor speed
            velocityPIDLoop(motorA.setSpeedDPS * motorA.dir, &motorA, &pidA);
            velocityPIDLoop(motorB.setSpeedDPS * motorB.dir, &motorB, &pidB);
            motorA.pidTimerFlag = 0;
        } 

        // Store the PID motor command
        OCR0A = mapRanges( abs(pidA.motorCommand), 0, 100, 0, 255 );
        OCR0B = mapRanges( abs(pidB.motorCommand), 0, 100, 0, 255 );
    }

    // Update motor states
    if( motorA.dir == 1 ){
        PORTB &= ~(1<<MOTOR_A_PHA);
    } else if(motorA.dir == -1) {
        PORTB |= (1<<MOTOR_A_PHA);
    } else {
        OCR0A = 0;
    }

    if( motorB.dir == 1 ) {			
        PORTB &= ~(1<<MOTOR_B_PHA);
    } else if(motorB.dir == -1) {
        PORTB |= (1<<MOTOR_B_PHA);
    } else {
        OCR0B = 0;
    }
}

ISR( ADC_vect ) {
    // period of 69.3333us for a standard ADC read, 2x longer for first
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

int
main(void)
{
	 
	init_structs();
	init();	
	
	uart_puts_P("PenguinPi v2.0\n");
		
	detect_reset();	
	
	hat_init( &hat );
	
//INIT Done	

//TESTS
//	//LEDs
//		//Y0	C2
//		//Y1	C3
//		//Y2	C5
    PORTC = 0x00;
    _delay_ms(500);	
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

	//RGB
    /*
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

	
#ifdef notdef
	//MOTORS	
	motorA.dir			= -1;
	motorA.setSpeedDPS	= 50;
	
	motorB.dir			= -1;
	motorB.setSpeedDPS	= 90;	
//END TESTS
#endif
	
	vdiv.count 	= ADC_COUNT;
	ADCSRA 		|= (1<<ADSC);		//start the first ADC conversion

    // Set up velocity PID
    pidA.kP = 1;
    pidA.motorCommand = 0;

    pidB.kP = 1;
    pidB.motorCommand = 0;

	static uint8_t motorControlCount = 0;
	
    timer_t t0, tf;

    t0 = timer_get();
    stats_t  loop_time;
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
		
        hat_update();

		
        if(motorControlCount < 250) {
            motorControlCount++;
        } else {
            //set PID flags
            motorA.pidTimerFlag = 1;
            motorB.pidTimerFlag = 1;
            motorControlCount   = 0;
        }
	
#ifdef notdef
		//cleanup buffers
		com = 0;
		for(uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) datagramG[j] = 0;
		dgrammem.fl = 0;
#endif

        // Ratio is now 1
		motorA.degrees = motorA.position; // * DEGPERCOUNT;
		motorB.degrees = motorB.position; //  * DEGPERCOUNT;


        // CONTROL WAS HERE
						
		//LED update
		if(ledR.state > 0) {
			if(ledR.brightness > 0) redLEDPercent(ledR.brightness);
			else 					LEDOn(LED_R);
		} else 						LEDOff(LED_R);

		if(ledG.state > 0) {
			if(ledG.brightness > 0) greenLEDPercent(ledG.brightness);
			else 					LEDOn(LED_G);
		} else 						LEDOff(LED_G);

		if(ledB.state > 0) { 
			if(ledB.brightness > 0) blueLEDPercent(ledB.brightness);
			else 					LEDOn(LED_B);
		} else 						LEDOff(LED_B);
		

		//Analog update
		if(vdiv.ready && !(ADCSRA&(1<<ADSC))){
			vdiv.raw   = ADC;
			vdiv.value = vdiv.raw * vdiv.scale/1000;
			vdiv.ready = 0;

			//change multiplexer to csense
			ADMUX 		|= (1<<MUX0);//change mux
			csense.count = ADC_COUNT;
			ADCSRA 		|= (1<<ADSC);//restart
		}
		
		if(csense.ready && !(ADCSRA&(1<<ADSC))){
			csense.raw   = ADC;
			csense.value = csense.raw * csense.scale;
			csense.ready = 0;

			//change multiplexer to csense
			ADMUX 	  &= ~(1<<MUX0);//change mux
			vdiv.count = ADC_COUNT;
			ADCSRA 	  |= (1<<ADSC);//restart
		}

		if(vdiv.value < battery.cutoff){
			if(battery.count < battery.limit) battery.count++;
			else{
                //FIXMEhat_lowvolts();
			}
		}else battery.count = 0;


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
        }
		
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
	
	
}

void init(void){
//V2.0		//Power reduction register
//V2.0		PRR0 &= ~((1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRADC));

	//UART
		uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(BAUD, F_CPU));

	//Motor Pins
		DDRB |= (1<<MOTOR_A_PWM) | (1<<MOTOR_B_PWM);
		DDRB |= (1<<MOTOR_A_PHA) | (1<<MOTOR_B_PHA);

	//Motor PWM
		//V1 was OC1A and OC1B
		//V2  is OC0A and OC0B
		TCCR0A |= (1<<COM0A1)|(0<<COM0A0)|			// Clear OC0A on Compare Match	Set OC0A on Bottom
				  (1<<COM0B1)|(0<<COM0B0)|			// Clear OC0B on Compare Match	Set OC0B on Bottom
				  (1<<WGM01) |(1<<WGM00); 			// Non-inverting, 8 bit fast PWM
				  
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
		
		//LEDR	was OC0A	now OC2A
		//LEDG	was OC0B	now OC1A
		//LEDB	was OC2B	now OC2B
	
		//GREEN ON A
			TCCR1A |= (1<<COM0A1)|(1<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(1<<WGM00); 			// inverting, 8 bit fast PWM
			TCCR1B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00);
		
        // 20MGHz 256/20
        // 0.0000128
		//RED on A and BLUE on B
			TCCR2A |= (1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM21)|(1<<WGM20); 			// inverting, 8 bit fast PWM
			TCCR2B |= (0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20);

	//8 bit controller timer
		TIMSK2 |= (1<<TOIE2);

	//ADC
		DIDR0	= (1<<ADC6D)|(1<<ADC7D);
		ADMUX 	= (0<<REFS1)|(1<<REFS0)|(0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0); 	//AVCC reference voltage with external cap on AREF, multiplex to channel 6
		ADCSRA 	= (1<<ADEN) |(1<<ADIE) |(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);					//Enable ADC, enable interrupt, prescaler of 64 (187.5kHz sample rate)
	
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
