PROGRAM := linux-robotic-arm.bin

CC = g++
CXXFLAGS += -O2 -g -fPIC -Wall -Wextra -Werror -pedantic-errors -std=c++14 -pipe
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = demo.cpp RoboticArm.cpp
OBJECTS = demo.o RoboticArm.o

SOURCES += HighLatencyGPIO/GPIO.cc \
           HighLatencyPWM/PWM.cc \
           Linux-Quadrature-Encoder/QuadratureEncoder.cpp \

OBJECTS += HighLatencyGPIO/GPIO.o \
           HighLatencyPWM/PWM.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o



all: $(SOURCES) $(OBJECTS) HighLatencyGPIO/GPIO.cc
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

HighLatencyGPIO/GPIO.cc:
	git clone -q https://github.com/oxavelar/HighLatencyGPIO

HighLatencyPWM/PWM.cc:
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
