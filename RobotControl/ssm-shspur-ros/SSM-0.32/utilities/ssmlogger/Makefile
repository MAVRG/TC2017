TARGET = ssmlogger

CC = gcc
LIBS =  -lm -lSSM -lpthread
CFLAGS = -g -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -Wall -I../../include -L../../lib 

OBJS = $(TARGET).o

all: $(TARGET) 

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(TARGET) 

clean:
	rm -f *.o *~ def/*~ core

install:
	cp $(TARGET) ../../bin/
	cp $(TARGET) /usr/local/bin/
