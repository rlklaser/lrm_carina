#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/rlklaser/Work/ros/sandbox/ogreassimp/devel', type 'exit' to leave"
  . "/home/rlklaser/Work/ros/sandbox/ogreassimp/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/rlklaser/Work/ros/sandbox/ogreassimp/devel'"
else
  . "/home/rlklaser/Work/ros/sandbox/ogreassimp/devel/setup.sh"
  exec "$@"
fi
