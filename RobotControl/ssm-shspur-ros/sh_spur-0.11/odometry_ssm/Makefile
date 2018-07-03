TARGET = odometry_viewer

CC = gcc
LIBS =  -lm -lSSM -lpthread 

CFLAGS = -O -g -Wall -I../../SSM-0.32/include -I../include\
	 -L../../SSM-0.32/lib -L../lib -L/usr/local/lib

OBJS = $(TARGET).o

all: $(TARGET) 

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(TARGET) 

clean:
	rm -f *.o *~ def/*~ core
