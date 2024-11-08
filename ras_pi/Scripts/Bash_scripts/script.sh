#!/bin/bash

echo "hello"

. ~/ros2_jazzy/install/setup.bash
source ~/ros2_ws/install/setup.bash
source /home/thesis/ros2_ws/install/local_setup.bash
source ~/ldlidar_ros2_ws/install/local_setup.bash

cd ~/ldlidar_ros2_ws
sudo chmod 777 /dev/ttyUSB0

cd ~/ldlidar_ros2_ws
source install/local_setup.bash

ros2 launch ldlidar_ros2 ld06.launch.py

/bin/bash
