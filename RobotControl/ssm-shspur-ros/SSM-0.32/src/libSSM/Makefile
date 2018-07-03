#===========================================================================
# Makefile for libSSM
#===========================================================================

SSM     = libSSM.a
SSM_OBJ = libSSM.o

CC = gcc

INCDIR = -I../../include 

CFLAGS = -O2 -Wall -Werror -Wmissing-prototypes $(INCDIR)

all: $(SSM)

$(SSM): $(SSM_OBJ)
	$(AR) $(ARFLAGS) $@ $(SSM_OBJ)
	cp $(SSM) ../../lib/
clean:
	rm -f  *.o *~ core 
install:
	cp $(SSM) ../../lib/
	cp $(SSM) /usr/local/lib/
