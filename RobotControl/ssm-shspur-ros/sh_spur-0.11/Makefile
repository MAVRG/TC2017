DIRS = libsrc sh_spur utilities

INSTDIR = ..

all: create

create:
	for i in $(DIRS); do cd $$i && make; cd ..; done

clean:
	for i in $(DIRS); do cd $$i && make clean; cd ..; done
	rm -rf *~ \#* include/*~ include/\#~

install:
	for i in $(DIRS); do cd $$i ; make install ; cd ../ ; done
	cp ./include/*.h /usr/local/include/
	cp -r ./include/sensor /usr/local/include/
