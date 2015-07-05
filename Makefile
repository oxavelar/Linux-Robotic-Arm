PROGRAM := linux-quadrature-encoder.bin

CC = g++
CXXFLAGS += -O2 -g -fPIC -Wall -std=c++0x
LDLIBS += -lpthread -lboost_system -lboost_filesystem

SOURCES = main.cpp quadrature-encoder.cpp
OBJECTS = main.o quadrature-encoder.o

SOURCES += HighLatencyGPIO/GPIO.cc
OBJECTS += HighLatencyGPIO/GPIO.o


all: $(SOURCES) $(OBJECTS)
	$(CC) $(CXXFLAGS) $(LDLIBS) $(OBJECTS) -o $(PROGRAM)

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
