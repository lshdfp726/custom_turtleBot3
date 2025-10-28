#!/bin/bash

rm -rf build/ install/ log/

colcon build

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 pkg list | grep $1
