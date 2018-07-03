TARGET = SensorA_interval_read

CC = gcc
#CC = g++
LIBS =  -lm -lSSM -lpthread
CFLAGS = -g -Wall -I../../include -I../ -L../../lib 
CXXFLAGS = $(CFLAGS)
 
OBJS = $(TARGET).o

all: $(TARGET) 

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(TARGET) 

clean:
	rm -f *.o *~ def/*~ core
install:
	cp $(TARGET) ../examples/
