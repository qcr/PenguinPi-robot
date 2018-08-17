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

#include "PCA6416A.h"
#include "SSD1306.h"

//#################################################################################################
//
// HATs
//


typedef struct {
	uint8_t 			show_option;		// Selects which screen to show
	
	//IP Addresses
	// int16_t				eth_addr_1;	
	// int16_t				eth_addr_2;
	// int16_t				eth_addr_3;
	// int16_t				eth_addr_4;
	
	// int16_t				wlan_addr_1;	
	// int16_t				wlan_addr_2;
	// int16_t				wlan_addr_3;
	// int16_t				wlan_addr_4;

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

Hat_s hat_status;

void 	init_hat			( Hat_s *hat );
void 	init_oled			( void );

void 	oled_clear_frame	( );
void    oled_frame_divider  ( );
void 	oled_write_frame_now( );
void 	oled_write_frame	( uint8_t *phase);
void 	oled_character		( uint8_t x, uint8_t y, char character );
void 	oled_string			( uint8_t x, uint8_t y, char *string );

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
uint16_t 	oled_refresh_count 	= 0;

void
hat_update()
{
    //Refresh OLED
    if ( hat_status.has_oled == 1 ) {
        if ( oled_refresh_count >= OLED_REFRESH) {
            // time to refresh
            oled_refresh_count	= 0;    // reset the refresh counter
            LED_DEBUG_B(500);
            // build the screen image
            float pid_dt = 0.0;//FIXME
            oled_screen( &hat_oled, &vdiv, &csense, &motorA, &motorB,
                    &displayA, datagram_last, &pidA, &pidB, pid_on, pid_dt);
            oled_refresh_phase = 1;  // start the progressive write to OLED
        }
        else
            oled_refresh_count++;  // increment the refresh counter

        if (oled_refresh_phase)
            oled_write_frame(&oled_refresh_phase);    // progressive write
    }		

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
		_delay_ms(100);		
	}
	for(uint8_t j = 0; j < 8; j++) {
		data_r[1]	=1<<j;

		i2cWritenBytes( data_r, PCA6416A_0, 0x02, 2);		
		_delay_ms(100);		
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
	
    oled->show_option += 1;     // next screen
	if ( oled->show_option > OLED_LAST ) 
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
            sprintf(fstring, "   A    B  On? %d", pid_on);
            oled_string( 0, 0, fstring);
            oled_string( 0, 0, "   A    B  " ); 

			sprintf(fstring, "e  %d", pidA->error );
            oled_string( 0, 1, fstring);
			sprintf(fstring, "mc %d", pidA->motorCommand );
            oled_string( 0, 2, fstring);
            sprintf(fstring, "de %d", pidA->dt );
            oled_string( 0, 3, fstring);



			sprintf(fstring, "%d", pidB->error );
            oled_string( 8, 1, fstring);
			sprintf(fstring, "%d", pidB->motorCommand );
            oled_string( 8, 2, fstring);
            sprintf(fstring, "%d", pidB->dt );
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
