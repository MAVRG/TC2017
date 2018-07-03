#!/bin/bash

source ~/TC2017/RobotControl/ssm-shspur-ros/catkin_ws/devel/setup.bash

killall ssm
killall sh_spur
sleep 2

ssm&
sleep 2

cd ~/TC2017/RobotControl/ssm-shspur-ros/sh_spur-0.11/sh_spur/
sh_spur -p Chinouka_okugai_robo.param&
sleep 2

cd ~/TC2017/RobotControl/ssm-shspur-ros/catkin_ws
rosrun auto_sh_spur auto_sh_spur
