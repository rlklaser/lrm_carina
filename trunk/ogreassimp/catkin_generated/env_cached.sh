#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/rlklaser/Work/ros/sandbox/ogreassimp/catkin_generated', type 'exit' to leave"
  . "/home/rlklaser/Work/ros/sandbox/ogreassimp/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/rlklaser/Work/ros/sandbox/ogreassimp/catkin_generated'"
else
  . "/home/rlklaser/Work/ros/sandbox/ogreassimp/catkin_generated/setup_cached.sh"
  exec "$@"
fi
