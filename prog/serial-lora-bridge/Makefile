CFLAGS=-I../../lmic
LDFLAGS=-lwiringPi

serial-lora-bridge: main.cpp
	cd ../../lmic && $(MAKE)
	$(CC) $(CFLAGS) -o serial-lora-bridge main.cpp ../../lmic/*.o $(LDFLAGS)

all: serial-lora-bridge

run: ./serial-lora-bridge

.PHONY: clean

clean:
	rm -f *.o ttn-abp-send
