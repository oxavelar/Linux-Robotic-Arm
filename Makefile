PROGRAM := linux-robotic-arm.app

CC = g++
CXXFLAGS += -O3 -std=c++11 -Wall -Wextra -Werror -pipe -march=native -flto -fomit-frame-pointer -Wno-error=reorder
LDFLAGS += -Wl
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

CXXFLAGS += -DRT_PRIORITY
CXXFLAGS += -DBASE_PWM_FREQUENCY_HZ=1000 -DBASE_PWM_DUTYCYCLE=50
CXXFLAGS += -DDEBUG


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


