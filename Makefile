PROGRAM := linux-robotic-arm.app

CC = g++
CXXFLAGS += -O3 -std=c++11 -Wall -Wextra -Werror -Wno-reorder -fomit-frame-pointer -pipe -march=native -flto
LDLIBS += -lpthread -lboost_system -lboost_filesystem -lncurses

SOURCES = demo.cpp RoboticArm.cpp
OBJECTS = demo.o RoboticArm.o

SOURCES += HighLatencyGPIO/GPIO.cc \
           HighLatencyPWM/PWM.cc \
           Linux-DC-Motor/Motor.cpp \
           Linux-Quadrature-Encoder/QuadratureEncoder.cpp \
           Linux-Visual-Encoder/VisualEncoder.cpp \

 
OBJECTS += HighLatencyGPIO/GPIO.o \
           HighLatencyPWM/PWM.o \
           Linux-DC-Motor/Motor.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o \
           Linux-Visual-Encoder/VisualEncoder.o \


DEPS += HighLatencyGPIO
DEPS += HighLatencyPWM

CXXFLAGS += -DNO_RT_PRIORITY
CXXFLAGS += -DBASE_PWM_FREQUENCY_HZ=8000 -DBASE_PWM_DUTYCYCLE=0
CXXFLAGS += -DNO_VISUAL_ENCODER
CXXFLAGS += -DNO_DEBUG -DDEBUG_LEVEL=50


all:
	$(MAKE) -j1 $(DEPS) > /dev/null
	$(MAKE) build

build: $(DEPS) $(SOURCES) $(OBJECTS)
	$(CC) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

$(DEPS):
	git clone -q https://github.com/oxavelar/HighLatencyGPIO
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	-rm -rf $(PROGRAM) $(OBJECTS)


