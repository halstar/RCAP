#!/bin/bash

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

ros2 launch rcap rcap.physical.launch.py | grep -v full
