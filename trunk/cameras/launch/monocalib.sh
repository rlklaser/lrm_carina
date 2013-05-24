#rosrun camera_calibration cameracalibrator.py --size 6x4 --square 0.005 image:=/stereo/left/image_raw camera:=/stereo/left
rosrun camera_calibration cameracalibrator.py --size 6x4 --square 0.005 image:=/stereo/right/image_raw camera:=/stereo/right
