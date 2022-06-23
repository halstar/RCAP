#!/bin/bash

ros2 launch rcap rcap.physical.launch.py | grep -v full
