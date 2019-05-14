BIN=main
OBJS=main.o datagram.o motor.o hat.o timer.o io.o lib/uart.o lib/twimaster.o 
MCU=atmega644p

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=20000000UL -mmcu=${MCU} -Wall -I./lib -std=gnu99

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex $< $@

${BIN}.elf: ${OBJS}
	${CC} -mmcu=${MCU} -Wl,-u,vfprintf -Wl,-lprintf_flt -Wl,-lm -o $@ $^

load: ${BIN}.hex
	sudo avrdude -v -c pi_isp -p ${MCU} -U flash:w:$<
	
# needed for a brand new PPI board
fuse:
	sudo avrdude -c pi_isp -p atmega644p -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

# do this to clear an error about GPIO24
gpio24:
	echo 24 > /sys/class/gpio/unexport
	
clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
