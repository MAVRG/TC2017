LIB_PKG     = libOdometrySSM.a
LIB_OBJ = libOdometrySSM.o

CC = gcc

INCDIR = -I../../include -I../../../SSM03/include/ -L../../../SSM03/lib 
LIBDIR = 
LIB = 
CFLAGS = -O2 -Wall -Werror -Wno-error=unused-but-set-variable -Wmissing-prototypes $(INCDIR)

all: $(LIB_PKG)

$(LIB_PKG): $(LIB_OBJ)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJ)

clean:
	rm -f *.a *.o *~ core 
install:
	cp $(LIB_PKG) ../../lib/
	cp $(LIB_PKG) /usr/local/lib/
