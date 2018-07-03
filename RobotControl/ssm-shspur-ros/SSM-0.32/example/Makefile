DIRS = SensorA_driver SensorA_interval_read SensorA_continual_read\
	 SensorA_past_read SensorAB_sync SensorB_driver

INSTDIR = ..

all: create

create:
	for i in $(DIRS); do cd $$i && make; cd ..; done

clean:
	for i in $(DIRS); do cd $$i && make clean; cd ..; done
	rm -rf *~ \#* include/*~ include/\#~

install:
	for i in $(DIRS); do cd $$i ; make install ; cd ../ ; done

