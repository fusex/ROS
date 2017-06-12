#!/bin/bash
chmod 777 /dev/ttyACM3
gnome-terminal -x sh -c "roscore"
source ../../../devel/setup.bash
roslaunch trajecto.launch
