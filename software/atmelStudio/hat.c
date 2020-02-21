#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include "lib/uart.h"
#include "lib/i2cmaster.h"

#include "global.h"
#include "hat.h"
#include "timer.h"
#include "datagram.h"

#include "PCA6416A.h"
#include "SSD1306.h"

#include "oled_data.h"

#define HAT_DIP_PI  1
#define HAT_DIP_BEACON 2

//OLED Screen Options
enum _oled_screen {
    OLED_IP_ADDR = 0,
    OLED_USER,
    OLED_BATTERY,
    OLED_ENCODERS,
    OLED_PID,
    OLED_PERFORMANCE,
    OLED_TIMING,
    OLED_ERROR,
    OLED_DATAGRAM,
    OLED_END,
    OLED_SHUTDOWN
};

enum _oled_polarity {
    NORMAL,
    INVERSE
};

// types
typedef struct _hat_s {
	int8_t 				config;				// -1 if no HAT present
	uint8_t				dir;				// Set bit to 1 if direction of bit needs to be an output
	uint8_t				int_07;				// Set bit to a 1 if interrupt enabled on HAT07
	uint8_t				has_oled;			// Set bit to a 1 if the I2C OLED is on the hat
    uint16_t             ledarray;        // the blue LED array
} Hat_s;

typedef struct {
	uint8_t 			show_option;		// Selects which screen to show
	
	//IP Addresses
	uint8_t				eth[4];
	uint8_t				wlan[4];
	uint8_t				wmac[6];
	
	//Error messages
	char				err_msg[3][OLED_LINELEN];

	// User messages
	char				user_msg[4][OLED_LINELEN];
} Hat_oled;

static Hat_s hat_status;
static Hat_oled	hat_oled;
uint8_t hat_user_button;
static uint8_t oled_frame[SSD1306_BUFFERSIZE];  // OLED screen buffer
uint8_t hat_DIP;

// forward defines
void init_oled(void);

void oled_clear_frame();
void oled_frame_divider();
void oled_write_frame_now();
void oled_write_frame( uint8_t *phase);
void oled_character(uint8_t x, uint8_t y, enum _oled_polarity, char character );
void oled_screen(Hat_oled *oled );
void oled_next_screen(Hat_oled *oled ); 
void oled_string(uint8_t x, uint8_t y, enum _oled_polarity, const char *fmt, ...) 
        __attribute__ ((format (printf, 4, 5)));
void parseOLEDOp(uint8_t *datagram, Hat_oled *hat_oled);
void usertext_init();

// local variables
static volatile uint8_t		hat_07_int_flag = 0;
static uint8_t oled_refresh_phase = 0;
static uint8_t scrollnext;

ISR( PCINT2_vect ) {
	//PCINT2 contains the interrupt from HAT07 on PCINT23 which is PC7
	
	if ( bit_is_clear( PINC, 7 ) ) {
        // detect falling edge on PC7
		//Need main loop to handle this as clearing INT will cause I2C clashes 
        //if not handled appropriately
		hat_07_int_flag = 1;
	}
}

void
hat_update(volatile uint8_t *oled_refresh)
{
    //Refresh OLED
    if ( hat_status.has_oled == 1 && *oled_refresh && oled_refresh_phase == 0) {
        // time to refresh
        *oled_refresh = 0;  // mark it as done
        // build the screen image
        oled_screen( &hat_oled);
        oled_refresh_phase = 1;  // start the progressive write to OLED
    }
    if (oled_refresh_phase)
        oled_write_frame(&oled_refresh_phase);    // progressive write

    //HAT Interrupt
    if ( hat_status.int_07 == 1 && hat_07_int_flag == 1 ) {
        uint8_t 	data_r[2]  			= {0, 0};
        //Have to act upon it here else we get I2C clashes between clearing 
        //the INT and OLED update
                    
        //Clear Interrupt by reading the device 
        i2cReadnBytes(data_r, PCA6416A_1, 0x00, 2 );					
            
        //data_r[0][7:4] contains the DIP switch which may have changed
        hat_DIP	= (data_r[0] & 0xF0) >> 4;

        // Bit 4 controls the global pid_on flag
        if (hat_DIP & HAT_DIP_PI)
            pid_on = 1; //PI ON!
        else
            pid_on = 0;

        //data_r[1][3:0] contains the buttons
        for (uint8_t i = 0; i < 4; i++) {
            if ( bit_is_clear( data_r[1], i ) ) {	

                switch ( i ) {
                    case 0 :	//Button S1 has been pressed
                        // move to next screen
                        oled_next_screen ( &hat_oled );
                        break;
                    case 1 : 	//Button S2 has been pressed -- ALL STOP
                        // stop all motors
                        motorR.velocity_dmd	=   0;
                        motorL.velocity_dmd	=   0;
                        break;
                    case 2 : 	//Button S3 has been pressed
                        // the function of this button depends on the screen state
                        switch ( hat_oled.show_option ) {
                        case OLED_ENCODERS:
                            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                                motorR.position = 0;
                                motorL.position = 0;
                                motorR.position_prev = 0;
                                motorL.position_prev = 0;
                            };
                            break;
                        case OLED_PID:
                            motorR.velocity_dmd	=  30;
                            motorL.velocity_dmd	= -30;
                            break;
                        case OLED_ERROR:
                            memset((void *)hat_oled.err_msg, 0, 3*OLED_LINELEN);
                            break;
                        case OLED_USER:
                            usertext_init();
                            break;
                        }
                        break;
                    case 3 : 	//Button S4 has been pressed
                        // free for user
                        hat_user_button++;
                        break;
                }
            }
        }		

        // Bit 3 controls the beacon
        if (hat_DIP & HAT_DIP_BEACON) {
            data_r[0] = 8; // display the beacon
            data_r[1] = 0x90;
        } else {
            data_r[0] = 0; // beacon is off
            data_r[1] = 0;
        }
        i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
        
        // clear the interrupt flag
        hat_07_int_flag = 0;
    }
}

uint8_t
hat_datagram(uint8_t *datagram)
{
	if (datagram[1] != AD_HAT)
        return 0;

	switch( datagram[2] ){
        // SETTERS
        case HAT_SET_SCREEN:
            datagram_validate(datagram, 1, "HAT_SET_IP_ETH");
            hat_oled.show_option = PAYLOAD(0);
            break;

        case HAT_SET_IP_ETH:
            datagram_validate(datagram, 4, "HAT_SET_IP_ETH");
            for (uint8_t i=0; i<4; i++)
                hat_oled.eth[i] = PAYLOAD(i);
            break;

        case HAT_SET_IP_WLAN:
            datagram_validate(datagram, 4, "HAT_SET_IP_WLAN");
            for (uint8_t i=0; i<4; i++)
                hat_oled.wlan[i] = PAYLOAD(i);
            break;

        case HAT_SET_MAC_WLAN:
            datagram_validate(datagram, 6, "HAT_SET_MAC_WLAN");
            for (uint8_t i=0; i<6; i++)
                hat_oled.wmac[i] = PAYLOAD(i);
            break;

        case HAT_SET_LEDARRAY:
            datagram_validate(datagram, 2, "HAT_SET_LEDARRAY");
            hat_status.ledarray = PAYLOAD16(0);
            i2cWritenBytes( (uint8_t *)&hat_status.ledarray, PCA6416A_0, 0x02, 2);		
            break;
            
        // GETTERS
        case HAT_GET_DIP:
            datagram_validate(datagram, 0, "HAT_GET_DIP");
			datagram_return(datagram, 'c', (uint8_t) hat_DIP);
            break;

        case HAT_GET_BUTTON:
            datagram_validate(datagram, 0, "HAT_GET_BUTTON");
			datagram_return(datagram, 'c', hat_user_button);
            hat_user_button = 0;
            break;

        case HAT_GET_LEDARRAY:
            datagram_validate(datagram, 0, "HAT_GET_LEDARRAY");
			datagram_return(datagram, 'i', hat_status.ledarray);
            break;

		default:
			errmessage("bad HAT opcode %d", datagram[2]);
            return 0;
            break;
	}	
    return 1;
}

void
hat_shutdown(char *msg)
{
    //display shutdown message on OLED
	oled_clear_frame();	
    oled_string( 0, 1, INVERSE, msg);
    oled_write_frame_now();
    _delay_ms(2000);

    //lockout most functions
    PRR0 |= (1<<PRTIM2)|(1<<PRTIM0)|(1<<PRTIM1);//|(1<<PRADC);
  
    //loop until the battery really is flat...
    while(1) {
        //send shutdown command
        uart_puts_P("Low battery shutdown\n");
        PORTB &= ~(1<<PB5);//pull the pin low
        _delay_ms(5000);
    }			
    // never return
}


//#################################################################################################

void hat_init( ) {
    Hat_s   *hat = &hat_status;
	
	hat_status.config				= -1;
	hat_status.dir					= 0x00;		//Inputs by default
	hat_status.int_07				= 0;		//No interrupts by default
	hat_status.has_oled			= 0;		//No OLED by default
	
	hat_oled.show_option	= OLED_IP_ADDR;	
    for (uint8_t i=0; i<4; i++) 
        hat_oled.wlan[i] = hat_oled.eth[i] = 0;
    for (uint8_t i=0; i<6; i++) 
        hat_oled.wmac[i] = 0;

	uint8_t data_r[2] = {0, 0};
	
	//Config always on PCA6416A_1
	//Read 2 bytes from address 0
	//Check if HAT responds on I2C - Bails out if not an ACK on first attempt - I2C will hang otherwise.
	
	if ( i2c_check_device( PCA6416A_1+I2C_WRITE ) == -1 ){
		//HAT not present or not ready
	}
	else {	
		i2cReadnBytes(data_r, PCA6416A_1, 0x00, 2 );
		hat->config	= data_r[0] & 0x0F;
	}
	
	//What to do now with each different HAT option
	switch( hat->config ) {
		case 1 : 
			uart_puts_P("HAT ID 1 : LEDs and OLED\n");			
			//This is the OLED and LED HAT made for EGB439
			//Has a DIP switch
				hat_DIP		= data_r[0] & 0xF0;
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
				hat_DIP		= data_r[0] & 0xF0;
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

    //Debug test to set LEDs on HAT	
	data_r[0]	= 0;
	data_r[1]	= 0;
	for(uint8_t j = 0; j < 8; j++) {
		data_r[0]	=1<<j;
		i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
		_delay_ms(50);		
	}
    data_r[0]	= 0;
	for(uint8_t j = 0; j < 8; j++) {
		data_r[1]	=1<<j;
		i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
		_delay_ms(50);		
	}

    data_r[0]	= 0;
    data_r[1]	= 0;
    i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
    _delay_ms(100);		

    usertext_init();
}

void hat_usertext_add(char c)
{
    switch (c) {
    case '\n':
        scrollnext = 1;
        return;
        break;
    case '\f':
        usertext_init();
        return;
        break;
    default:
        if (strlen(hat_oled.user_msg[3]) >= (OLED_LINELEN-1) || scrollnext) {
            scrollnext = 0;
            // scroll up
            memmove(hat_oled.user_msg[0], hat_oled.user_msg[1], 3*OLED_LINELEN);
            memset(hat_oled.user_msg[3], 0, OLED_LINELEN);
        }
        uint8_t k = strlen(hat_oled.user_msg[3]);
        hat_oled.user_msg[3][k] = c;
        hat_oled.user_msg[3][k+1] = 0;
        break;
    }
}

void usertext_init()
{
    strncpy((char *)hat_oled.user_msg, "USER TEXT", 4*OLED_LINELEN);

}

//#################################################################################################
//
// OLED
//
//#################################################################################################

void oled_screen(Hat_oled *oled)
{
	
	oled_clear_frame();	
	
	switch ( oled->show_option ) {
        case OLED_PID :
            oled_string( 0, 0, INVERSE, "VEL   L    R");
            oled_string( 15, 0, NORMAL, "On? %d", pid_on);

            oled_string( 0, 1, NORMAL, "v*  %3d", motorR.velocity_dmd);
            oled_string( 0, 2, NORMAL, "ve  %3d", motorR.verror);
            oled_string( 0, 3, NORMAL, "mc  %3d", motorR.command);

            oled_string( 9, 1, NORMAL, "%3d", motorL.velocity_dmd);
            oled_string( 9, 2, NORMAL, "%3d", motorL.verror);
            oled_string( 9, 3, NORMAL, "%3d", motorL.command);
            oled_string( 15, 3, INVERSE, "test");
            break;

		/* case OLED_DISPLAY :  */
		/* 	oled_string( 0, 0, "DISPLAY" ); */
		/* 	sprintf(fstring, "%d", display.address ); */
		/* 	oled_string ( 0,1,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display.draw ); */
		/* 	oled_string ( 10,1,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display.value ); */
		/* 	oled_string ( 0,2,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display.mode ); */
		/* 	oled_string ( 10,2,fstring ); 				 */
		/* 	sprintf(fstring, "%d", display.digit0 ); */
		/* 	oled_string ( 0,3,fstring ); 	 */
		/* 	sprintf(fstring, "%d", display.digit1 ); */
		/* 	oled_string ( 10,3,fstring ); 	 */
		/*  */
		/* 	break; */
		
		case OLED_BATTERY :
			oled_string( 0, 0, INVERSE, "BATTERY" ); 
			oled_string( 0, 2, NORMAL, "%.2fV", vdiv.smooth/1000);
			oled_string( 12, 2, NORMAL, "%.0fmA", csense.smooth);
			break;	

		case OLED_ENCODERS :
			oled_string( 0, 0, INVERSE, "ENCODERS" );

      //Encoders RAW
			oled_string( 0, 1, NORMAL, "encL: %6d", motorL.position);
			oled_string( 0, 2, NORMAL, "encR: %6d", motorR.position);
            oled_string( 12, 1, NORMAL, "%4d", motorL.velocity);
            oled_string( 12, 2, NORMAL, "%4d", motorR.velocity);
            oled_string( 15, 3, INVERSE, "reset");
			break;
      
		case OLED_IP_ADDR :
			oled_string( 0, 0, INVERSE, "IP Address" );
			oled_string( 0, 1, NORMAL, "eth  %3d.%3d.%3d.%3d",
                    hat_oled.eth[0],
                    hat_oled.eth[1],
                    hat_oled.eth[2],
                    hat_oled.eth[3]);
			oled_string( 0, 2, NORMAL, "wlan %3d.%3d.%3d.%3d",
                    hat_oled.wlan[0],
                    hat_oled.wlan[1],
                    hat_oled.wlan[2],
                    hat_oled.wlan[3]);
			oled_string( 0, 3, NORMAL, "wmac   %02x%02x%02x:%02x%02x%02x",
                    hat_oled.wmac[0],
                    hat_oled.wmac[1],
                    hat_oled.wmac[2],
                    hat_oled.wmac[3],
                    hat_oled.wmac[4],
                    hat_oled.wmac[5]);
			break;

        case OLED_PERFORMANCE:
            oled_string( 0, 0, INVERSE, "PERFORMANCE");
            oled_string( 0, 1, NORMAL, "up   %6lu s", seconds_counter);
            oled_string( 0, 2, NORMAL, "pkts %6ui %6uo", performance.packets_in, performance.packets_out);
            oled_string( 0, 3, NORMAL, "err  %6u", performance.errors);
            break;

        case OLED_TIMING: {
            oled_string( 0, 0, INVERSE, "LOOP TIMING");
            oled_string( 0, 1, NORMAL, "mean %12lu us", stats_mean(&performance.loop_time));
            uint32_t std = stats_std(&performance.loop_time);
            oled_string( 0, 2, NORMAL, "std  %12lu us", std);
            oled_string( 0, 3, NORMAL, "max  %12lu us", performance.loop_time.max);
            break;
        }

		case OLED_DATAGRAM: {
            uint8_t i, x = 0, y = 2;
            oled_string( 0, 0, INVERSE, "DATAGRAM (hex)" );
            for (i=0; i<=datagram_last[0]; i++) {
                oled_string (x, y, NORMAL, "%02x", datagram_last[i] );
                x += 3;
                if (x >= 20) {
                    x = 0; y++;
                }
            }
            break;
        }

		case OLED_ERROR:
			oled_string( 0, 0, INVERSE, "ERROR" );
            oled_string( 0, 1, NORMAL, hat_oled.err_msg[0] );
            oled_string( 0, 2, NORMAL, hat_oled.err_msg[1] );
            oled_string( 0, 3, NORMAL, hat_oled.err_msg[2] );
            oled_string( 15, 3, INVERSE, "clear");
			break;

        case OLED_USER:
            oled_string( 0, 0, NORMAL, hat_oled.user_msg[0] );
            oled_string( 0, 1, NORMAL, hat_oled.user_msg[1] );
            oled_string( 0, 2, NORMAL, hat_oled.user_msg[2] );
            oled_string( 0, 3, NORMAL, hat_oled.user_msg[3] );
            break;
			
		case OLED_SHUTDOWN: 
			oled_string( 0, 0, INVERSE, "SHUTDOWN" );
			break;
	}			
}

void init_oled () {
	
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
    memcpy(oled_frame, splash_frame, SSD1306_BUFFERSIZE);
    oled_write_frame_now();
    _delay_ms(2000);
}

void oled_invert_display(uint8_t inv)
{
	i2cWriteByte( SSD1306_DISPLAYOFF, SSD1306_DEFAULT_ADDRESS, 0x00 );
    if (inv)
        i2cWriteByte( SSD1306_INVERTDISPLAY, SSD1306_DEFAULT_ADDRESS, 0x00 );
    else
        i2cWriteByte( SSD1306_NORMALDISPLAY, SSD1306_DEFAULT_ADDRESS, 0x00 );
    i2cWriteByte( SSD1306_DISPLAYON, SSD1306_DEFAULT_ADDRESS, 0x00 );
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


void oled_write_frame_now()
{
    uint8_t phase = 1;

    oled_write_frame(&phase);
    while (phase)
        oled_write_frame(&phase);
}

void oled_write_frame (uint8_t *phase) {
	
	uint8_t frame_i2c_buffer[16];
	
    if (*phase == 0)
        return;
    else if (*phase == 1) {
        // send the setup
        i2cWriteByte( SSD1306_COLUMNADDR, SSD1306_DEFAULT_ADDRESS, 0x00 );
        // Column start address (0 = reset)
        i2cWriteByte( 0, 				  SSD1306_DEFAULT_ADDRESS, 0x00 );
        // Column end address (127 = reset)
        i2cWriteByte( SSD1306_WIDTH-1,    SSD1306_DEFAULT_ADDRESS, 0x00 );
        i2cWriteByte( SSD1306_PAGEADDR,   SSD1306_DEFAULT_ADDRESS, 0x00 );
        // Page start address (0 = reset)
        i2cWriteByte( 0, 			      SSD1306_DEFAULT_ADDRESS, 0x00 );
        // Page end address 		32 => 3, 64 => 7
        i2cWriteByte( 3, 				  SSD1306_DEFAULT_ADDRESS, 0x00 );
        (*phase)++;
    } else {
        // phase = 2:34, send a 16 byte chunk
        uint8_t i = *phase - 2;
		
		memcpy( frame_i2c_buffer, &oled_frame[16*i], 16 );
		i2cWritenBytes( frame_i2c_buffer, SSD1306_DEFAULT_ADDRESS, 0x40, 16 );	

        (*phase)++; // next chunk
        if (++i>=SSD1306_BUFFERSIZE/16) // are we done yet?
            *phase = 0; // indicate done
	}
}

void oled_string(uint8_t x, uint8_t y, enum _oled_polarity polarity, const char *fmt, ...)
{
    va_list ap;
    char    buf[33];

    va_start(ap, fmt);

    vsnprintf(buf, 33, fmt, ap);

    if (x > 21) {
        errmessage("oled_string: x=%d too big", x);
        return;
    }
    if (y > 3) {
        errmessage("oled_string: y=%d too big", y);
        return;
    }
    
	//128 x 32 pixels on screen
	//Characters are 6 pixels by 8
	//Character display is therefore 21 x 4
	//x and y inputs are intended to be on these terms, so y=3 is bottom line of characters
	
    for (uint8_t i=0; buf[i]; i++)
		oled_character(6*(x+i), y, polarity, buf[i]);	
    va_end(ap);
}

void oled_character( uint8_t x, uint8_t y, enum _oled_polarity polarity, char character )
{
	int 		char_in_ascii 	= character;
    //128 added as oled_frame is 512 bytes or 128 columns x 4 rows
	uint16_t 	frame_addr 		= x + (128*y);
	
	//Take 5 entries from the ASCII array and put into the frame
    switch (polarity) {
    case NORMAL:
        for (int index = 0; index < 5; index++)
            oled_frame[ frame_addr + index ]	= ASCII[ char_in_ascii - 0x20][index];
        //space between characters
        oled_frame[ frame_addr + 5 ] 			= 0x00;		
        break;
    case INVERSE:
        for (int index = 0; index < 5; index++)
            oled_frame[ frame_addr + index ]	= ~ASCII[ char_in_ascii - 0x20][index];
        //space between characters
        oled_frame[ frame_addr + 5 ] 			= 0xFF;		
        break;
    }
}

void oled_next_screen ( Hat_oled *oled ) {
	
	if ( ++oled->show_option >= OLED_END)
		oled->show_option = 0;		
}

void  
hat_show_error( char *msg )
{
     Hat_oled *oled = &hat_oled;
	
	uint8_t char_count = 0;
	
	while ( *msg ) {
		if ( char_count>63) {
			//ERROR TOO LONG => ignore rest			
		}
		else if ( char_count<21 ) {
			oled->err_msg[0][char_count]	= *msg++;
		}
		else if ( char_count<42 ) {
			oled->err_msg[1][char_count-21]	= *msg++;
		}
		else {
			oled->err_msg[2][char_count-42]	= *msg++;
		}
		
		char_count++;
	}
	
	oled->show_option = OLED_ERROR;
}
