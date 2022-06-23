#!/bin/bash

cd mpu9250
rm -Rf build install log

cd ..
cd robot_car
rm -Rf build install log

cd ..
cd rplidar
rm -Rf build install log

cd ..
rm -Rf build install log
