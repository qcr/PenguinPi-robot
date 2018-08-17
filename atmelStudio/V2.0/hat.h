
void hat_init();
uint8_t hat_datagram(uint8_t *datagram);
void hat_update();
void hat_lowvolts();
void hat_show_error(char *s);

typedef struct _hat_s {
	int8_t 				config;				// -1 if no HAT present
	int8_t 				dip;				// Status of the DIP switches 
	uint8_t				dir;				// Set bit to 1 if direction of bit needs to be an output
	uint8_t				int_07;				// Set bit to a 1 if interrupt enabled on HAT07
	uint8_t				has_oled;			// Set bit to a 1 if the I2C OLED is on the hat
} Hat_s;

extern Hat_s hat_status;
