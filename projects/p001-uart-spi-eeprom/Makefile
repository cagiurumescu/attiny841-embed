AVRGCCMCU=attiny841
AVRDUDEMCU=t841
CC=avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=$(AVRGCCMCU)
OBJ2HEX=avr-objcopy
AVRDUDE=/usr/bin/avrdude
TARGET=serial

all:
	$(CC) $(CFLAGS) $(TARGET).c -o $(TARGET)
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex

erase:
	-$(AVRDUDE) -e -F -V -c linuxspi -p $(AVRDUDEMCU) -P /dev/spidev0.0 
	echo 25 >> /sys/class/gpio/export

# -D to not erase the eeprom
wrflash: $(TARGET).c $(TARGET)
	-$(AVRDUDE) -D -F -V -c linuxspi -p $(AVRDUDEMCU) -P /dev/spidev0.0 -U flash:w:$(TARGET).hex:i
	echo 25 >> /sys/class/gpio/export

wreeprom:
	$(AVRDUDE) -F -V -c linuxspi -p $(AVRDUDEMCU) -P /dev/spidev0.0 -U eeprom:w:eeprom.bin:r
	echo 25 >> /sys/class/gpio/export

rdeeprom:
	$(AVRDUDE) -c linuxspi -p $(AVRDUDEMCU) -P /dev/spidev0.0 -U eeprom:r:eeprom.hex:i
	echo 25 >> /sys/class/gpio/export
