/*
 * main.c
 *
 * NOTES
 * ========
 *
 * Motor, set_power() invokes MOTOR_SET_velocity_dPS which sets motor->dir and motor->setSpeedDPS
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

#include "i2cmaster.h"
#include "uart.h"

#include "global.h"
#include "hat.h"
#include "datagram.h"
#include "motor.h"
#include "timer.h"

// parameters
#define ERRMSGLEN   80

#define BATTERY_WARNING  7000
#define BATTERY_SHUTDOWN  6500

const uint8_t   control_interval = 20; // ms
const uint8_t   adc_interval = 10; // ms
const uint8_t   oled_interval = 200; // ms


// global variables
Motor 		motorR;
Motor 		motorL;
LED         leds[6];
AnalogIn 	vdiv;
AnalogIn 	csense;
Performance performance;

uint8_t    pid_on = 3;  // PID control is on

volatile uint8_t  oled_update_now = 0;
volatile uint8_t  second_now = 0;
volatile uint32_t seconds_counter;  // wraps every 18 hours

uint8_t  low_voltage = 0;
uint8_t  pishutdown_up = 0;


// forward defines
void test_leds();



//######################################################################
//
// ISRs
//
//######################################################################


// initialize the counters so they are not in sync
volatile uint8_t control_counter = 0;
volatile uint8_t adc_counter = 10;
volatile uint8_t oled_counter = 5;
volatile uint16_t milliseconds_counter = 12;

ISR( TIMER1_COMPA_vect ) {
    // period of 1ms

    // 1 second heartbeat on green LED
    if (++milliseconds_counter >= 1000) {
        milliseconds_counter = 0;
        seconds_counter++;
        second_now = 1;
    }

    // update all LED counters
    for (uint8_t i=0; i<NLEDS; i++) {
        if (leds[i].count > 0)
            if (--leds[i].count == 0)
                leds[i].state = 0;      // flag this LED to be turned off now
    }

    // initiate an OLED update in the main loop
    if (++oled_counter >= oled_interval) {
        oled_counter = 0;
        oled_update_now = 1;
    }

    // initiate ADC conversion
    if (++adc_counter > adc_interval) {
        adc_counter = 0;

        ADMUX = ADC_REF | vdiv.channel;    // select volts
		ADCSRA |= (1<<ADSC);            // start conversion
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
    motor_velocity_control(&motorR);
    motor_velocity_control(&motorL);

    // Set the PWM values
    OCR0A = mapRanges( abs(motorR.command), 0, 100, 0, 255 );
    OCR0B = mapRanges( abs(motorL.command), 0, 100, 0, 255 );

    // Set motor driver polarities
    if (motorR.command >= 0)
        PORTB &= ~(1<<MOTOR_A_PHA); 
    else if (motorR.command < 0)
        PORTB |= (1<<MOTOR_A_PHA); 

    if (motorL.command >= 0)
        PORTB &= ~(1<<MOTOR_B_PHA);
    else if (motorL.command < 0)
        PORTB |= (1<<MOTOR_B_PHA);
}

ISR( PCINT0_vect ) {
	//All encoders are now on one PCINT vector
	//Externally on PCINT{3:0} which is PA3:0

	//detect which pin triggered the interrupt
		uint8_t PINA_val   = PINA;
		uint8_t encA1 = ( PINA_val & (1<<MOTOR_A_ENC_1) )>>MOTOR_A_ENC_1; 	//store the current state
		uint8_t encB1 = ( PINA_val & (1<<MOTOR_A_ENC_2) )>>MOTOR_A_ENC_2;
		uint8_t encA2 = ( PINA_val & (1<<MOTOR_B_ENC_1) )>>MOTOR_B_ENC_1; 	//store the current state
		uint8_t encB2 = ( PINA_val & (1<<MOTOR_B_ENC_2) )>>MOTOR_B_ENC_2;
		
	//Update Motor states
		motor_encoder_update( &motorR, encA1, encB1 ); 
		motor_encoder_update( &motorL, encA2, encB2 ); 
	
	//Update LEDs
        LED_DEBUG_G(2);
}

uint16_t            adc[8];
volatile uint8_t   adc_done;

ISR( ADC_vect ) {
    // conversion complete, 42us after initiation

    uint8_t chan = ADMUX & 0x1f;    // which channel did we just read?

    adc[chan] = ADC;     // get the value and stash it


    if (chan == vdiv.channel) {
        // just read volts, ask for current
        ADMUX = ADC_REF | csense.channel;  // select current
        ADCSRA |= (1<<ADSC);            // start conversion
    } else
        adc_done = 1;
}


//#################################################################################################
//                      Main polling loop
//#################################################################################################


int
main(void)
{
    timer_t t0, tf;
	 
	init_structs();
	io_init();	
    // Set up velocity PID
    motor_init(&motorR, 0);
    motor_init(&motorL, 1);
	
	uart_puts_P("PenguinPi v2.0\n");
	detect_reset();	
	hat_init();
    test_leds();
    datagram_init();
	
    stats_init(&performance.loop_time);
    debugmessage("starting main loop");
    timer_get(&t0);
    while (1) 
    {
        // get timing stats for main loop
        timer_get(&tf);
        uint32_t dt = timer_diff_us(&tf, &t0);
        stats_add(&performance.loop_time, dt);
        /*
        if (dt > 2000) {
            debugmessage("dt = %lu", dt);
            debugmessage("%lu %u %u %lu %u %u", tf.sec, tf.ms, tf.clock, t0.sec, t0.ms, t0.clock);
        }
        */
        t0 = tf;

        // Check for a new datagram
        uint8_t c = datagram_poll();
        if (c)
            hat_usertext_add(c);
		
		//LED update
        for (uint8_t i=0; i<NLEDS; i++) {
            LED *led = &leds[i];

            if(led->state)
                LEDOn(i);
            else
                LEDOff(i);
        }

		//Analog update
        if (adc_done) {
            adc_done = 0;

            io_analog_filter_step(&vdiv, adc[6]);
            io_analog_filter_step(&csense, adc[7]);
        }

		if (vdiv.smooth < battery.mvolts_warning) {
            // battery volts are below warning level
            if (battery.warning == 0) {
                battery.warning = 1;
                oled_invert_display(1); // invert the display if not already done
            }

            if (vdiv.smooth < battery.mvolts_shutdown)
                hat_shutdown(" LOW VOLTAGE SHUTDOWN ");
		}

        if ((PINB & (1<<PB7)) == 0)
            LEDOff(Y4);
        else
            LEDOn(Y4);

        // has the RPi acknowledged a shutdown request?
        if ((PINB & (1<<PB7)) == 0) {
            // GPIO11 is low
            // EITHER shutdown daemon not running OR shutdown in progress
            if (pishutdown_up)  // daemon was running, must be shutdown
                hat_shutdown(" TURN RaspberryPI OFF SAFELY ");
        } else {
            // GPIO11 is high
            // shutdown daemon is running
            pishutdown_up = 1;
        }

        // Update the hat
        hat_update(&oled_update_now);
		
        if (pid_on == 0) {
            // no velocity control, command PWM directly
            OCR0A = mapRanges( abs(motorR.velocity_dmd), 0, 100, 0, 255 );
            OCR0B = mapRanges( abs(motorL.velocity_dmd), 0, 100, 0, 255 );
            
            // Set motor driver polarities
            if (motorR.command >= 0)
                PORTB &= ~(1<<MOTOR_A_PHA); 
            else if (motorR.command < 0)
                PORTB |= (1<<MOTOR_A_PHA); 

            if (motorL.command >= 0)
                PORTB &= ~(1<<MOTOR_B_PHA);
            else if (motorL.command < 0)
                PORTB |= (1<<MOTOR_B_PHA);
        }
        if (second_now) {
            second_now = 0;
            // debug message here
            //debugmessage("mean %lu max %lu", stats_mean(&performance.loop_time), performance.loop_time.max);
            if ((seconds_counter % 60) == 0) {
                stats_init(&performance.loop_time);
                performance.loop_time.max = 0;
                //debugmessage("reset stats");
            }
            if ((seconds_counter % 60) == 20) {
                //debugmessage("n=%lu, sx=%lu, sx2=%lu", performance.loop_time.n, performance.loop_time.sum, performance.loop_time.sum2);
                //debugmessage("mean=%lu, std=%lu, mx=%lu", 
                        //stats_mean(&performance.loop_time), stats_std(&performance.loop_time), performance.loop_time.max);
            }
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

    performance.errors++;
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
	vdiv.scale 				= 16.018497;//mV per div
    vdiv.channel = 6;
    vdiv.alpha = 0.95;
	csense.scale 			= 3.2226562;//mA per div
    csense.channel = 7;
    csense.alpha = 0.95;
	
	battery.mvolts_warning = BATTERY_WARNING;
    battery.warning = 0;
    battery.mvolts_shutdown = BATTERY_SHUTDOWN;
	
    for (uint8_t i=0; i<NLEDS; i++) {
        leds[i].state = 0;
        leds[i].count = 0;
    }
	
}

void
test_leds()
{
    for (uint8_t i=0; i<NLEDS; i++) {
        LEDOn(i);
        _delay_ms(300);	
        LEDOff(i);
        _delay_ms(200);	
    }
}
