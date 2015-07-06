PROGRAM := linux-quadrature-encoder.bin

CC = g++
CXXFLAGS += -O2 -g -fPIC -Wall -pedantic -std=c++0x
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = main.cpp quadrature-encoder.cpp
OBJECTS = main.o quadrature-encoder.o

SOURCES += HighLatencyGPIO/GPIO.cc
OBJECTS += HighLatencyGPIO/GPIO.o

CXXFLAGS += -DCONFIG_GPIO_PIN_A=15 -DCONFIG_GPIO_PIN_B=16 -DCONFIG_GPIO_PIN_Z=17

all: $(SOURCES) $(OBJECTS) HighLatencyGPIO/GPIO.cc
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

HighLatencyGPIO/GPIO.cc:
	git clone -q https://github.com/tweej/HighLatencyGPIO

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
