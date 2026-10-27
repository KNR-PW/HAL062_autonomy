#!/bin/bash

echo "|"
echo "| + HAL062 - Isaac ROS dev container"
echo "| + Release: 5.0"
echo "|"

echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
echo "[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash" >> ~/.bashrc

exec bash