#!/usr/bin/env bash
source /home/lrm/ros/setup.bash
export ROS_MASTER_URI=http://master:11311
exec "$@"

