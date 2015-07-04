PROGRAM := linux-quadrature-encoder.bin

CC = g++
CXXFLAGS += -pedantic -O0 -g -fPIC
LDFLAGS +=

SOURCES = main.cpp quadrature-encoder.cpp
OBJECTS = main.o quadrature-encoder.o

all: $(SOURCES) $(OBJECTS)
	$(CC) $(CXXFLAGS) $(OBJECTS) -o $(PROGRAM)

clean:
	rm -rf $(PROGRAM) $(OBJECTS)
