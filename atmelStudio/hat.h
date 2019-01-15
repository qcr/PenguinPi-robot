#ifndef __hat_h__
#define __hat_h__

#include <stdint.h>

#define OLED_LINELEN 21

// exported functions
void hat_init();
uint8_t hat_datagram(uint8_t *datagram);
void hat_update(volatile uint8_t *update);
void hat_shutdown(char *msg);
void hat_show_error(char *s);
void oled_invert_display(uint8_t inv);
void hat_usertext_add(char c);

extern uint8_t hat_user_button;
extern uint8_t hat_DIP;

#endif
