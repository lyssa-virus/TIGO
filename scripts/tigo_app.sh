#!/bin/bash
# Script para iniciar o Rosbridge e a ponte do Arduino na Raspberry Pi
# sem depender do ROS_DISCOVERY_SERVER configurado no ~/.bashrc.

set -e

unset ROS_DISCOVERY_SERVER
source /opt/ros/humble/setup.bash

cd ~/tigo_ws
source install/setup.bash

ros2 daemon stop || true
ros2 daemon start

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

cleanup() {
    kill "$ROSBRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run tigo_base arduino_bridge
