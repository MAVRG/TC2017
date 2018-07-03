#! /bin/sh
if [ -d $1 ]; then
 echo "Directory $1 already exists."
else  
mkdir $1
./ssmlogger -n spur_odometry -i 0 -l $1/odm.log &
./ssmlogger -n pws_motor   -i 0 -l $1/pws.log &
#./ssmlogger -n segway_rmp     -i 0 -l $1/rmp.log &
#./ssmlogger -n keyboard      -i 0 -l $1/keyboard.log &
#./ssmlogger -n URG_data      -i 0 -l $1/urg.log &
#./ssmlogger -n us_data       -i 0 -l $1/us.log &
#./ssmlogger -n GPS            -i 0 -l $1/gps.log &
./ssmlogger -n URG_SCIP2.0    -i 0 -l $1/turg1.log 
#./ssmlogger -n URG_SCIP2.0    -i 1 -l $1/turg2.log &
#./ssmlogger -n URG_SCIP2.0    -i 2 -l $1/urg1.log &
#./ssmlogger -n URG_SCIP2.0    -i 3 -l $1/urg2.log &
#./ssmlogger -n xbow           -i 0 -l $1/xbow.log 
#./ssmlogger -n dynamixel      -i 0 -l $1/dynamixel_1.log & 
##./ssmlogger -n dynamixel      -i 1 -l $1/dynamixel_2.log &
#./ssmlogger -n dynamixel      -i 2 -l $1/dynamixel_3.log 
fi
