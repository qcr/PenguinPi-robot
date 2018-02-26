SRC =  main.c lib/twimaster.c lib/uart.c
DEVFLAGS = -mmcu=avr5  -D__AVR_ATmega328PB__
CFLAGS = -fpack-struct -funsigned-bitfields -funsigned-char -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums $(DEVFLAGS) -I./lib -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"
OBJ = main.o lib/uart.o lib/twimaster.o

main.hex: $(OBJ)
	@echo Building target
	avr-gcc -o main.elf $< -Wl,-Tdata,0x801000 -Wl,-Map=main.map -Wl,-u,vfprintf -Wl,--start-group -Wl,-lprintf_flt -Wl,-lm  -Wl,--end-group -Wl,-L"../lib"  -Wl,--gc-sections -mmcu=avr5 

	avr-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures main.elf main.hex
	avr-objcopy -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex main.elf main.eep
	avr-objdump -h -S main.elf > main.lss
	avr-size -B main.elf

clean:
	-rm $(OBJ) main.hex main.elf

load:
	avrdude -c avrisp2 -p ATmega328P -F -U flash:w:main.hex -U eeprom:w:main.eep

restore:
	avrdude -c avrisp2 -p ATmega328P -F -U flash:w:PenguinPi-original.hex

%.o:	%.c
	@echo Building $<
	avr-gcc $(CFLAGS) -c -o $@ $<

lib/%.o: lib/%.c
	@echo Building $<
	avr-gcc $(CFLAGS) -c -o $@ $<
