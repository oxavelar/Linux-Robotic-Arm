PROGRAM := linux-robotic-arm.app
TARGET_BOARD := Galileo

CC = g++
CXXFLAGS += -O3 -Wall -Wextra -Werror -std=c++11 -pipe -march=native -flto -fomit-frame-pointer
LDFLAGS += -Wl
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

DEPS += HighLatencyGPIO
DEPS += HighLatencyPWM

CXXFLAGS += -DRT_PRIORITY
CXXFLAGS += -DBASE_PWM_FREQUENCY_HZ=50000 -DBASE_PWM_DUTYCYCLE=50
CXXFLAGS += -DDEBUG



all:
	$(MAKE) -j1 build

build: $(DEPS) $(SOURCES) $(OBJECTS)
	$(CC) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

$(DEPS):
	git clone -q https://github.com/oxavelar/HighLatencyGPIO
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	-rm -rf $(PROGRAM) $(OBJECTS)


