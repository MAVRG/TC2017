LIB_PKG     = libCoordinateSystem2D.a
LIB_OBJ = libCoordinateSystem2D.o

CC = gcc

INCDIR = -I../../include 

CFLAGS = -O2 -Wall -Werror -Wmissing-prototypes $(INCDIR) 

all: $(LIB_PKG)

$(LIB_PKG): $(LIB_OBJ)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJ)

clean:
	rm -f *.a *.o *~ core 
install:
	cp $(LIB_PKG) ../../lib/
	cp $(LIB_PKG) /usr/local/lib/
