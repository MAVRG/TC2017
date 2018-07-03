#===========================================================================
# Makefile for behavior_program
#---------------------------------------------------------------------------
# [Update log]
#===========================================================================

TARGET  = spur_test
CC	= gcc
SYSTEM	= linux
LIB_DIR	= -L/usr/local/lib
LIBS	= -lm -lpthread -lSSM -lSHSpur
INCDIR	= -I/usr/local/include
CFLAGS	= -g -O2 -Wall  $(INCDIR)

OBJS	= spur_test.o

all:	$(SYSTEM)

linux:	$(OBJS)
	$(CC) $(OBJS) $(LIB_DIR) $(LIBS) -o $(TARGET) 
	rm -f  *.o *~ defs/*~ core

clean:
	rm -f $(TARGET) *.o *~ defs/*~ core

install:
	cp $(TARGET) ../../bin
