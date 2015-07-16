PROGRAM := linux-robotic-arm.bin
TARGET_BOARD_PLATFORM := Galileo

CC = g++
CXXFLAGS += -O3 -g -fPIC -Wall -pedantic -std=c++0x -pipe
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = demo.cpp RoboticArm.cpp
OBJECTS = demo.o RoboticArm.o

SOURCES += HighLatencyGPIO/GPIO.cc \
           HighLatencyPWM/PWM.cc \
           Linux-Quadrature-Encoder/QuadratureEncoder.cpp \

OBJECTS += HighLatencyGPIO/GPIO.o \
           HighLatencyPWM/PWM.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o


ifeq ($(TARGET_BOARD_PLATFORM), Galileo)
CXXFLAGS += -march=pentium -mtune=pentium -m32
endif


all: $(SOURCES) $(OBJECTS) HighLatencyGPIO/GPIO.cc
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

HighLatencyGPIO/GPIO.cc:
	git clone -q https://github.com/tweej/HighLatencyGPIO

HighLatencyPWM/PWM.cc:
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
