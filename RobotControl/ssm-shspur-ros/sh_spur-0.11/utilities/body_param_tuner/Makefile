TARGET = calibration
PROGRAM = calibration

CC = gcc
LIBS = -lpthread -lm -lSHSpur -lCoordinateSystem2D -lSSM

CFLAGS = -O -Wall -I/usr/local/include -L../../lib \
	-I../../../SSM03/include -L/usr/local/lib 

OBJS = $(TARGET).o

all: $(PROGRAM) 

$(PROGRAM): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(PROGRAM) 
	
clean:
	rm -f *.o *~ def/*~ core

	
