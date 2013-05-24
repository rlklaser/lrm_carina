#!/bin/bash

rosservice call gazebo/apply_joint_effort '{joint_name: joint_back_left_wheel, effort: 5.0, duration: -2 }'
rosservice call gazebo/apply_joint_effort '{joint_name: joint_front_left_wheel, effort: 5.0, duration: -2 }'
rosservice call gazebo/apply_joint_effort '{joint_name: joint_back_right_wheel, effort: -5.0, duration: -2 }'
rosservice call gazebo/apply_joint_effort '{joint_name: joint_front_right_wheel, effort: -5.0, duration: -2 }'

#rosservice call gazebo/apply_joint_effort '{joint_name: joint_back_left_wheel, effort: 6, duration: -2,
#                                            joint_name: joint_front_left_wheel, effort: 6, duration: -2,
#                                            joint_name: joint_back_right_wheel, effort: -6, duration: -2, 
#                                            joint_name: joint_front_right_wheel, effort: -6, duration: -2}'
                                             


