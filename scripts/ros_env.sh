#!/usr/bin/env bash
# Must bash source this file
source /opt/ros/jazzy/setup.bash

cd /ros2_ws

if [ -f rail-interface/install/setup.bash ]; then
  source rail-interface/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [ -f "$HOME/.ros/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://$HOME/.ros/cyclonedds.xml"
fi

# i didnt wanna make a launch file lol
alias launch-rail-drive="ros2 run rail_interface rail-drive --ros-args   -p serial.port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF3433524B3043072235-if02 &"

alias launch-rail-teleop="ros2 run rail_interface rail-teleop --ros-args   -p serial.port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 &"

echo "Sourced ros2 environment."
