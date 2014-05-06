roslaunch lrm_bringup robot_model.launch

rosrun tf tf_echo /base_footprint /imu_link
rosrun tf tf_echo /base_footprint /stereo_camera
rosrun tf tf_echo /base_footprint /base_odometry

