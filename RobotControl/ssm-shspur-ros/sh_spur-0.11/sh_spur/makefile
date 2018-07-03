TARGET = sh_spur
CC	= gcc

SYSTEM	= shspur
LIB_DIR	= -L/usr/local/lib -L./ -L../lib
LIBS	= -lm -lpthread -lSSM -lCoordinateSystem2D
INCDIR	= -I./include -I/usr/local/include -I../include -I../include/sensor
CFLAGS	= -g -O2 -Wall -Wmissing-prototypes $(INCDIR) -D SSM_ENABLE
OBJS	= main.o serial.o param.o control.o command.o \
	command_run.o command_set.o command_get.o command_param.o

all:	$(SYSTEM)

shspur:	$(OBJS)
	$(CC) $(OBJS) $(LIB_DIR) $(LIBS) -o $(TARGET) 
	rm -f  *.o *~ defs/*~ core

clean:
	rm -f $(TARGET) *.o *~ defs/*~ core

install:
	cp $(TARGET) ../bin
	cp $(TARGET) /usr/local/bin/
#	mkdir /usr/local/share/sh_spur
#	cp M1_param /usr/local/share/sh_spur
#	ln -s /usr/local/share/sh_spur/M1_param /usr/local/share/sh_spur/robot.param
