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

python3                  tools/cameraStream.py &
ros2 launch rcap         rcap.physical.nav.launch.py &
ros2 launch nav2_bringup navigation_launch.py params_file:=robot_car/config/nav2.physical.yaml

