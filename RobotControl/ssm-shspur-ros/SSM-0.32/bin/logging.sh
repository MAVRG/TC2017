#! /bin/sh
if [ -d $1 ]; then
 echo "Directory $1 already exists."
else  
mkdir $1
./ssmlogger -n spur_odometry -i 0 -l $1/odm.log &
#./ssmlogger -n spur_odometry -i 1 -l $1/godm.log &
#./ssmlogger -n spur_global   -i 0 -l $1/gl.log &
./ssmlogger -n spur_global   -i 2 -l $1/nav200.log &
#./ssmlogger -n pws_motor     -i 0 -l $1/pws.log &
./ssmlogger -n keyboard      -i 0 -l $1/keyboard.log &
#./ssmlogger -n URG_data      -i 0 -l $1/urg.log &
#./ssmlogger -n us_data       -i 0 -l $1/us.log &
#./ssmlogger -n GPS            -i 0 -l $1/gps.log &
./ssmlogger -n URG_SCIP2.0    -i 0 -l $1/urg.log &
#./ssmlogger -n GPS           -i 0 -l $1/gps.log &
#./ssmlogger -n GPS_NMEA           -i 0 -l $1/gps_nmea.log & 
#./ssmlogger -n dynamixel      -i 0 -l $1/dynamixel.log  &
echo "logging_start"
fi
