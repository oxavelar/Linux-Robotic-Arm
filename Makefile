PROGRAM := linux-robotic-arm.bin
TARGET_BOARD_PLATFORM := Galileo

CC = g++
CXXFLAGS += -O3 -g -fPIC -Wall -pedantic -std=c++0x -pipe
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = demo.cpp RoboticArm.cpp
OBJECTS = demo.o RoboticArm.o

SOURCES += HighLatencyGPIO/GPIO.cc \
           Linux-Quadrature-Encoder/QuadratureEncoder.cpp \

OBJECTS += HighLatencyGPIO/GPIO.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o


ifeq ($(TARGET_BOARD_PLATFORM), Galileo)
CXXFLAGS += -march=pentium -mtune=pentium -m32
endif


all: $(SOURCES) $(OBJECTS) HighLatencyGPIO/GPIO.cc
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

HighLatencyGPIO/GPIO.cc:
	git clone -q https://github.com/tweej/HighLatencyGPIO

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
