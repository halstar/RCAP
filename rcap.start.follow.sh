#!/bin/bash

killall ros2 python3 2>/dev/null


cd mpu9250
source install/setup.bash 

cd ..
cd robot_car
source install/setup.bash 

cd ..
cd rplidar
source install/setup.bash 

cd ..
source install/setup.bash 

ros2 launch rcap      rcap.physical.launch.py &
ros2 launch robot_car wall_follower.py

