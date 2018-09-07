#ifndef __hat_h__
#define __hat_h__
void hat_init();
uint8_t hat_datagram(uint8_t *datagram);
void hat_update(volatile uint8_t *update);
void hat_lowvolts();
void hat_show_error(char *s);
void oled_invert_display(uint8_t inv);

typedef struct _hat_s {
	int8_t 				config;				// -1 if no HAT present
	int8_t 				dip;				// Status of the DIP switches 
	uint8_t				dir;				// Set bit to 1 if direction of bit needs to be an output
	uint8_t				int_07;				// Set bit to a 1 if interrupt enabled on HAT07
	uint8_t				has_oled;			// Set bit to a 1 if the I2C OLED is on the hat
} Hat_s;

typedef struct {
	uint8_t 			show_option;		// Selects which screen to show
	
	//IP Addresses
	uint8_t				eth[4];
	uint8_t				wlan[4];
	
	//Error messages
	char				err_line_1[21];
	char     			err_line_2[21];
	char				err_line_3[21];

	// User messages
	char				user_msg[4][21];
	
} Hat_oled;

extern Hat_s hat_status;

#endif
