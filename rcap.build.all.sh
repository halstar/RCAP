#!/bin/bash

cd mpu9250
colcon build

cd ..
cd robot_car
colcon build

cd ..
cd rplidar
colcon build

cd ..
colcon build
