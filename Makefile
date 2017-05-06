CC = g++
CXXFLAGS += -g -O3 -std=c++11 -Wall -Wextra -Werror -Wno-reorder -fomit-frame-pointer -pipe -ftree-vectorize -mfpmath=sse -march=native -mtune=native -flto
LDLIBS += -lpthread -lboost_system -lboost_filesystem -lboost_timer -lncurses
LDFLAGS += -O1 -std=c++11 -Wall -flto --hash-style=gnu --as-needed

SOURCES = RoboticArm.cpp
OBJECTS = RoboticArm.o
 
OBJECTS += HighLatencyGPIO/GPIO.o \
           HighLatencyPWM/PWM.o \
           Linux-DC-Motor/Motor.o \
           Linux-Quadrature-Encoder/QuadratureEncoder.o \
           Linux-Visual-Encoder/VisualEncoder.o \

DEMOS = Examples/Robot_Diagnostics.o \
        Examples/Robot_Keyboard.o \
        Examples/Robot_Playback.o \
        Examples/Robot_Recorder.o \

DEPS += HighLatencyGPIO \
        HighLatencyPWM \

CXXFLAGS += -DRT_PRIORITY=5 -DRT_POLICY=SCHED_RR
CXXFLAGS += -DBASE_PWM_FREQUENCY_HZ=250 -DBASE_PWM_DUTYCYCLE=0
CXXFLAGS += -DNO_VISUAL_ENCODER
CXXFLAGS += -DDEBUG -DDEBUG_LEVEL=5


all:
	$(MAKE) -j1 $(DEPS) > /dev/null
	$(MAKE) build

build: $(DEPS) $(OBJECTS) $(DEMOS)
	# To build all of our demos as separate binaries
	$(CC) $(LDLIBS) $(OBJECTS) Examples/Robot_Diagnostics.o  -o linux-robotic-arm-diagnostics.app
	$(CC) $(LDLIBS) $(OBJECTS) Examples/Robot_Keyboard.o     -o linux-robotic-arm-keyboard.app
	$(CC) $(LDLIBS) $(OBJECTS) Examples/Robot_Playback.o     -o linux-robotic-arm-playback.app
	$(CC) $(LDLIBS) $(OBJECTS) Examples/Robot_Recorder.o     -o linux-robotic-arm-recorder.app


$(DEPS):
	git clone -q https://github.com/oxavelar/HighLatencyGPIO
	git clone -q https://github.com/oxavelar/HighLatencyPWM

clean:
	-rm -rf $(OBJECTS) $(DEMOS)
	-rm -rf linux-robotic-arm-*.app


