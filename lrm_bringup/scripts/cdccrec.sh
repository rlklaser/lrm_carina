rosbag record /stereo/left/image_raw /stereo/left/camera_info /stereo/right/image_raw /stereo/right/camera_info /encoders_controller/encoders -e "/lse_xsens_mti/xsens/(.*)" -e "/move_base/(.*)" /octomap/octomap_point_cloud_centers /navigation/cmd_vel /planning/cmd_vel /wheel_odometry/odom /map /tf /joint_states /throttle_commands