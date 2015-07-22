PROGRAM := linux-robotic-arm.app

CC = g++
CXXFLAGS += -O3 -fPIC -Wall -Wextra -Werror -std=c++11 -pipe
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = demo.cpp RoboticArm.cpp
OBJECTS = demo.o RoboticArm.o

SOURCES += HighLatencyGPIO/GPIO.cc \
           HighLatencyPWM/PWM.cc \
           Linux-DC-Motor/Motor.cpp \
           Linux-Quadrature-Encoder/QuadratureEncoder.cpp \

OBJECTS += HighLatencyGPIO/GPIO.o \
           HighLatencyPWM/PWM.o \
           Linux-DC-Motor/Motor.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o

DEPS = HighLatencyGPIO/GPIO.cc HighLatencyPWM/PWM.cc

CXXFLAGS += -DRT_PRIORITY
CXXFLAGS += -DBASE_PWM_FREQUENCY_HZ=200 -DBASE_PWM_DUTYCYCLE=50


all: $(DEPS) $(SOURCES) $(OBJECTS)
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

HighLatencyGPIO/GPIO.cc:
	git clone -q https://github.com/oxavelar/HighLatencyGPIO

HighLatencyPWM/PWM.cc:
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	rm -rf $(PROGRAM) $(OBJECTS)

