
BIN=main
OBJS=main.o lib/uart.o lib/twimaster.o lib/PenguinPi.o
MCU=atmega644p

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=20000000UL -mmcu=${MCU} -Wall -I./lib -std=gnu99



${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex $< $@

${BIN}.elf: ${OBJS}
	${CC} -mmcu=${MCU} -Wl,-u,vfprintf -Wl,-lprintf_flt -Wl,-lm -o $@ $^

load: ${BIN}.hex
	avrdude -v -c pi_isp -p ${MCU} -U flash:w:$<
	
clear_gpio:
	echo 24 > /sys/class/gpio/unexport
	
clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
	
	
	
