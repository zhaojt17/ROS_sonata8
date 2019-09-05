#! /bin/bash

source ./devel/setup.bash
mknod --mode=a+rw /dev/can0 c 52 0
roslaunch startup dbg_can.launch
