#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "PenguinPi.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lib/uart.h"
#include "lib/i2cmaster.h"

#include "global.h"
#include "hat.h"
#include "timer.h"

#include "PCA6416A.h"
#include "SSD1306.h"

//#################################################################################################
//
// HATs
//

//OLED Screen Options
enum _oled_screen {
    OLED_IP_ADDR = 0,
    OLED_USER,
    OLED_BATTERY,
    OLED_MOTORS, 
    OLED_ENCODERS,
    OLED_POSITION,
    OLED_PID,
    OLED_TIMING,
    OLED_ERROR,
    OLED_SHUTDOWN,
    OLED_END
};

void oled_string(uint8_t x, uint8_t y, const char *fmt, ...);


typedef struct {
	uint8_t 			show_option;		// Selects which screen to show
	
	//IP Addresses
	uint8_t				eth[4];
	uint8_t				wlan[4];
	
	//Error messages
	uint8_t				err_line_1[21];
	uint8_t 			err_line_2[21];
	uint8_t				err_line_3[21];

	// User messages
	uint8_t				user_msg[4][21];
	
} Hat_oled;

#define OLED_REFRESH 1000	//How often should the main loop run before OLED refresh. If too fast information cannot be seen

Hat_oled	hat_oled;
static uint8_t oled_frame[ SSD1306_BUFFERSIZE ];
void parseOLEDOp( uint8_t *datagram, Hat_oled *hat_oled );
static const uint8_t ASCII[][5];

Hat_s hat_status;

void 	init_hat			( Hat_s *hat );
void 	init_oled			( void );

void 	oled_clear_frame	( );
void    oled_frame_divider  ( );
void 	oled_write_frame_now( );
void 	oled_write_frame	( uint8_t *phase);
void 	oled_character		( uint8_t x, uint8_t y, char character );

void 	oled_screen    		( Hat_oled *oled, AnalogIn *vdiv, AnalogIn *csense, Motor *motorA, Motor *motorB, Display *display, uint8_t *datagram, PidController *pidA, PidController *pidB, uint8_t pid_on, double pid_dt);
void    oled_next_screen	( Hat_oled *oled ); 

uint8_t		hat_07_int_flag = 0;

ISR( PCINT2_vect ) {
	//PCINT2 contains the interrupt from HAT07 on PCINT23 which is PC7
	
	if ( bit_is_clear( PINC, 7 ) ) { 							// detect falling edge on PC7
		
		//Need main loop to handle this as clearing INT will cause I2C clashes if not handled appropriately
		hat_07_int_flag = 1;
	}
}

uint8_t oled_refresh_phase = 0;

void
hat_update(uint8_t *oled_refresh)
{
    //Refresh OLED
    if ( hat_status.has_oled == 1 && oled_refresh && oled_refresh_phase == 0) {
        // time to refresh
        *oled_refresh = 0;  // mark it as done
        LED_DEBUG_B(10);
        // build the screen image
        float pid_dt = 0.0;//FIXME
        oled_screen( &hat_oled, &vdiv, &csense, &motorA, &motorB,
                &displayA, datagram_last, &pidA, &pidB, pid_on, pid_dt);
        oled_refresh_phase = 1;  // start the progressive write to OLED
    }
    if (oled_refresh_phase)
        oled_write_frame(&oled_refresh_phase);    // progressive write

    //HAT Interrupt
    if ( hat_status.int_07 == 1 && hat_07_int_flag == 1 ) {
        uint8_t 	data_r[2]  			= {0, 0};
        //Have to act upon it here else we get I2C clashes between clearing the INT and OLED update
                    
        //Clear Interrupt by reading the device 
            i2cReadnBytes(data_r, PCA6416A_1, 0x00, 2 );					
            
        //data_r[0][7:4] contains the DIP switch which may have changed
            hat_status.dip			= data_r[0] & 0xF0;

            // Bit 4 controls the global pid_on flag
            if ((hat_status.dip & 0x10) == 0x10) {
                //PID ON!
                pid_on = 1;
            } else {
                pid_on = 0;
            }
        
        //data_r[1][3:0] contains the buttons
            for(uint8_t i = 0; i < 4; i++) {
                if ( bit_is_clear( data_r[1], i ) ) {	

                    switch ( i ) {
                        case 0 :	//Button S1 has been pressed											
                            oled_next_screen ( &hat_oled );
                            break;
                        case 1 : 	//Button S2 has been pressed											
                            motorA.dir			=   0;
                            motorA.setSpeedDPS	=   0;
                            motorB.dir			=   0;
                            motorB.setSpeedDPS	=   0;
                            break;
                        case 2 : 	//Button S3 has been pressed											
                            motorA.dir			=  -1;
                            motorA.setSpeedDPS	=  50;
                            break;
                        case 3 : 	//Button S4 has been pressed											
                            motorB.dir			=   1;
                            motorB.setSpeedDPS	= 100;
                            break;
                    }
                }
            }		
                                    
        hat_07_int_flag = 0;
    }
}

uint8_t
hat_datagram(uint8_t *datagram)
{
	switch( datagram[1] ){
		case AD_OLED:
			parseOLEDOp( datagram, &hat_oled );

//DELETE		case AD_BTN_A:
//DELETE			parseButtonOp(datagram, &buttonA);
//DELETE		break;
//DELETE		case AD_BTN_B:
//DELETE			parseButtonOp(datagram, &buttonB);
//DELETE		break;
//DELETE		case AD_BTN_C:
//DELETE			parseButtonOp(datagram, &buttonC);
            return 1;   // datagram was processed
        default:
            return 0;   // no datagram processed here
    }
}

void
hat_lowvolts()
{
    //lockout most functions
    PRR0 |= (1<<PRTIM2)|(1<<PRTIM0)|(1<<PRTIM1);//|(1<<PRADC);
    //display low battery warning
    uint8_t reg[2] = {0, 0};
    reg[0] = DIGIT0_B;
    reg[1] = DIGIT1_F;
    i2cWritenBytes(reg, displayA.address, OUTPUT_0, 2);
                    
    while(1){
        //send shutdown command
        uart_puts_P("Low battery shutdown request");
        PORTB &= ~(1<<PB5);//pull the pin low
        //loop until the battery really is flat...
    }			
}

void parseOLEDOp	( uint8_t *datagram, Hat_oled *hat_oled ) {

	switch( datagram[2] ){
		//SETTERS
		case OLED_SET_IP_ETH_1:
			if( datagram[0] == 5 ){
				hat_oled->eth[0] = (datagram[3]<<8) | datagram[4];
			}
			else			
				errmessage("OLED_SET_IP1: incorrect type %d", datagram[0]);		
		break;
		case OLED_SET_IP_ETH_2:
			if( datagram[0] == 5 ){
				hat_oled->eth[1] =  (datagram[3]<<8) | datagram[4];
			} else
				errmessage("OLED_SETIP2: incorrect type %d", datagram[0]);
		break;
		case OLED_SET_IP_ETH_3:
			if( datagram[0] == 5 ){
				hat_oled->eth[2] = (datagram[3]<<8) | datagram[4];
			} else
				errmessage("OLED_SETIP3: incorrect type %d", datagram[0]);
		break;
		case OLED_SET_IP_ETH_4:
			if( datagram[0] == 5 ){
				hat_oled->eth[3] = (datagram[3]<<8) | datagram[4];
			} else
				errmessage("OLED_SETIP4: incorrect type %d", datagram[0]);

		break;
		case OLED_SET_IP_WLAN_1:
			if( datagram[0] == 5 ){
				hat_oled->wlan[0] = (datagram[3]<<8) | datagram[4];
			}
			else				
				errmessage("OLED_SETIPWLAN1: incorrect type %d", datagram[0]);			
		break;
		case OLED_SET_IP_WLAN_2:
			if( datagram[0] == 5 ){
				hat_oled->wlan[1] =  (datagram[3]<<8) | datagram[4];
			}else
				errmessage("OLED_SETIPWLAN2: incorrect type %d", datagram[0]);

		break;
		case OLED_SET_IP_WLAN_3:
			if( datagram[0] == 5 ){
				hat_oled->wlan[2] = (datagram[3]<<8) | datagram[4];
			}else
				errmessage("OLED_SETIPWLAN3: incorrect type %d", datagram[0]);

		break;
		case OLED_SET_IP_WLAN_4:
			if( datagram[0] == 5 ){
				hat_oled->wlan[3] = (datagram[3]<<8) | datagram[4];
			}else
				errmessage("OLED_SETIPWLAN4: incorrect type %d", datagram[0]);

		break;	
        case OLED_SET_IP_ETH:
			if( datagram[0] == 7 ){
				for (uint8_t i=0; i<4; i++)
					hat_oled->eth[i] = datagram[i];
			}else
				errmessage("OLED_SETIPETH: incorrect type %d", datagram[0]);

		break;
        case OLED_SET_IP_WLAN:
			if( datagram[0] == 7 ){
				for (uint8_t i=0; i<4; i++)
					hat_oled->wlan[i] = datagram[i];
			}else
				errmessage("OLED_SETIPWLAN: incorrect type %d", datagram[0]);
		break;
		
		default:
			errmessage("bad OLED opcode %d", datagram[2]);
		break;
	}	
}


//#################################################################################################

void hat_init( ) {
    Hat_s   *hat = &hat_status;
	
	hat_status.config				= -1;
	hat_status.dip	    			= -1;
	hat_status.dir					= 0x00;		//Inputs by default
	hat_status.int_07				= 0;		//No interrupts by default
	hat_status.has_oled			= 0;		//No OLED by default
	
	hat_oled.show_option	= OLED_IP_ADDR;	
	hat_oled.eth[0] 		= 0;
	hat_oled.eth[1] 		= 0;
	hat_oled.eth[2] 		= 0;
	hat_oled.eth[3] 		= 0;
	hat_oled.wlan[0] 		= 0;
	hat_oled.wlan[1] 		= 0;
	hat_oled.wlan[2] 		= 0;
	hat_oled.wlan[3] 		= 0;
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
    oled_write_frame_now();
    _delay_ms(2000);
	
	
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

void oled_screen   ( Hat_oled *oled, AnalogIn *vdiv, AnalogIn *csense, Motor *motorA, Motor *motorB, Display *display, uint8_t *datagram, PidController *pidA, PidController *pidB, uint8_t pid_on, double pid_dt) {
	
	oled_clear_frame();	
	
	switch ( oled->show_option ) {
        case OLED_PID :
            oled_string( 0, 0, "   A    B  On? %d", pid_on);

            oled_string( 0, 1, "e  %d", pidA->error );
            oled_string( 0, 2, "mc %d", pidA->motorCommand );
            oled_string( 0, 3, "de %d", pidA->dt );

            oled_string( 8, 1, "%d", pidB->error );
            oled_string( 8, 2, "%d", pidB->motorCommand );
            oled_string( 8, 3, "%d", pidB->dt );

            oled_string( 11, 3, "dt:%f", pid_dt );
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
			oled_string( 0, 1, "  %1.3f V", vdiv->value);
			oled_string( 0, 2, "  %3.3f mA", csense->value);
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
			oled_string( 5, 2, "dps: %6d", motorA->setSpeedDPS);
            oled_string ( 13,2, "%6d", motorB->setSpeedDPS);
		
            oled_frame_divider(); //Divide screen
			break;

		case OLED_ENCODERS :
			oled_string( 0, 0, "ENCODERS" );

      //Encoders RAW
			oled_string( 0, 1, "enc1: %6d", motorA->enc_raw1);
            oled_string ( 13,1, "%6d", motorB->enc_raw1);

			oled_string( 0, 2, "enc2: %6d", motorA->enc_raw2);
            oled_string ( 13,2, "%6d", motorB->enc_raw2);
        	
            oled_frame_divider();	//Divide screen
			break;
      
		case OLED_POSITION :
			oled_string( 0, 0, "POSITION" );
        
			//position
			oled_string( 0, 1, "pos: %6d", motorA->position);
            oled_string(13, 1, "%6d", motorB->position);

			//degrees
			oled_string( 0, 2, "deg: %6d", motorA->degrees);
            oled_string(13, 2, "%6d", motorB->degrees);
				
			//Divide screen
            oled_frame_divider();				
			break;
			
		case OLED_IP_ADDR :
			oled_string( 0, 0, "IP Addresses" );
			oled_string( 0, 1, "eth  %3d.%3d.%3d.%3d",
                    oled->eth[0],
                    oled->eth[1],
                    oled->eth[2],
                    oled->eth[3]);
			oled_string( 0, 2, "wlan %3d.%3d.%3d.%3d",
                    oled->wlan[0],
                    oled->wlan[1],
                    oled->wlan[2],
                    oled->wlan[3]);
			break;

        case OLED_TIMING:
            oled_string( 0, 0, "Timing stats");
            oled_string( 0, 1, "mean %lu", stats_mean(&loop_time));
            oled_string( 0, 2, "var  %lu", stats_var(&loop_time));
            oled_string( 0, 3, "max  %lu", loop_time.max);
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
}

void oled_string(uint8_t x, uint8_t y, const char *fmt, ...)
{
    va_list ap;
    char    buf[33];

    va_start(ap, fmt);

    vsnprintf(buf, 33, fmt, ap);
    
	//128 x 32 pixels on screen
	//Characters are 6 pixels by 8
	//Character display is therefore 21 x 4
	//x and y inputs are intended to be on these terms, so y=3 is bottom line of characters
	
    for (uint8_t i=0; buf[i]; i++)
		oled_character(x+6*i, y, buf[i]);	

    va_end(ap);
}

//OLED FRAME	
static uint8_t oled_frame[ SSD1306_BUFFERSIZE ] = {
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


static const uint8_t ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

