#===========================================================================
# Makefile for viewer
#---------------------------------------------------------------------------
# [Update log]
#===========================================================================

TARGET = joystick
CC	= gcc
SYSTEM	= linux
LIB_DIR	= -L/usr/local/lib -L./ -L../../../SSM03/lib -L../../lib
LIBS	= -lm -lpthread -lSSM -lSHSpur -lCoordinateSystem2D
SH_DIR = ../../
INCDIR	= -I./include-I/usr/local/include -I$(SH_DIR)/include \
	-I../../../SSM03/include -I../../include

#DEBUG
CFLAGS	= -g -O2 -Wall -Wmissing-prototypes $(INCDIR)

#COMPLETE
#CFLAGS	= -O2 -Wall -Werror -Wmissing-prototypes $(INCDIR)

OBJS	= joystick.o

all:	$(SYSTEM)

linux:	$(OBJS)
	$(CC) $(OBJS) $(LIB_DIR) $(LIBS) -o $(TARGET) 
	rm -f  *.o *~ defs/*~ core

clean:
	rm -f $(TARGET) *.o *~ defs/*~ core

install:
	cp $(TARGET) ../../bin
