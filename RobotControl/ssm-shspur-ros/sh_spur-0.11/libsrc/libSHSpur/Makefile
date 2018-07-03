LIBSHSPUR     = libSHSpur.a
LIBSHSPUR_OBJ = libSHSpur.o

CC = gcc

INCDIR = -I../../include
LIBDIR = 
LIB = 
CFLAGS = -O2 -Wall -Werror -Wmissing-prototypes $(INCDIR)
#CFLAGS = -O2 -Wall -Werror $(INCDIR)

all: $(LIBSHSPUR)


$(LIBSHSPUR): $(LIBSHSPUR_OBJ)
	$(AR) cru  $@ $(LIBSHSPUR_OBJ)
	ranlib $(LIBSHSPUR)
clean:
	rm -f *.a *.o *~ core 
install:
	cp $(LIBSHSPUR) ../../lib/
	cp $(LIBSHSPUR) /usr/local/lib/
