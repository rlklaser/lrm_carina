/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2009, University of British Columbia
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of British Columbia, nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string>
#include <cstdio>


#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>


#include <triclops.h>

using namespace std;

// colormap for disparities
static unsigned char dmap[768] =
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
  };



#define HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      ROS_ERROR( \
	 "ERROR: %s reported %s.", \
	 "function", \
	 triclopsErrorToString( error ) ); \
     ros::shutdown(); \
     exit(3); \
   } \
} \


class BumblebeeStereo
{
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter image_sub_l, image_sub_r;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_l, info_sub_r;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
										sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

	ros::Subscriber calibration_sub_;


	image_transport::Publisher pub_mono_l_;
	image_transport::Publisher pub_rect_l_;
	image_transport::Publisher pub_color_l_;
	image_transport::Publisher pub_rect_color_l_;
	image_transport::Publisher pub_mono_r_;
	image_transport::Publisher pub_rect_r_;
	image_transport::Publisher pub_color_r_;
	image_transport::Publisher pub_rect_color_r_;
	image_transport::Publisher pub_disparity_;
	image_transport::Publisher pub_disparity_image_;
	ros::Publisher pub_pts_;
	ros::Publisher pub_pts_grid_;

	// parameters
	bool do_rectify_;
	bool do_stereo_;
	bool do_calc_points_;
	bool do_keep_coords_;
	int stereo_width_;
	int stereo_height_;
	bool subpixel_interpolation_;
	double texture_threshold_;
	double uniqueness_threshold_;
	int min_disparity_;
	int max_disparity_;
	string calibration_path_;

	string  calibration_settings_;
	bool has_calibration_;
	bool initialized_;

	TriclopsContext      triclops_;
	TriclopsContext      triclops_full_res_;
	TriclopsCamera       camera_;
	TriclopsInput        triclopsInput_;
    TriclopsColorImage   colorImage_;

    TriclopsImage16 depthImage16_;
//    TriclopsImage depthImage;


    IplImage* left_;
    IplImage* right_;
    IplImage* left_mono_;
    IplImage* right_mono_;
    IplImage* left_rect_;
    IplImage* right_rect_;
    IplImage* disp_;
    IplImage* disp_8u_;

    IplImage* right_rect_small_;

    sensor_msgs::PointCloud cloud_;
    sensor_msgs::PointCloud cloud_grid_;

    IplImage* red_;
    IplImage* green_;
    IplImage* blue_;
    IplImage* red_header_;
    IplImage* green_header_;
    IplImage* blue_header_;

    int width_;
	int height_;

public:
	BumblebeeStereo() : it_(nh_), sync_(3), has_calibration_(false), initialized_(false)
	{
		ros::NodeHandle pivate_nh("~");
		pivate_nh.param("do_rectify", do_rectify_, true);
		pivate_nh.param("do_stereo", do_stereo_, true);
		pivate_nh.param("do_calc_points", do_calc_points_, true);
		pivate_nh.param("do_keep_coords", do_keep_coords_, false);

		pivate_nh.param("stereo_width", stereo_width_, 320);
		pivate_nh.param("stereo_height", stereo_height_, 240);
		pivate_nh.param("subpixel_interpolation", subpixel_interpolation_, true);
		pivate_nh.param("texture_threshold", texture_threshold_, 1.1);
		pivate_nh.param("uniqueness_threshold", uniqueness_threshold_, 0.8);
		pivate_nh.param("min_disparity", min_disparity_, 0);
		pivate_nh.param("max_disparity", max_disparity_, 96);
		pivate_nh.param<string>("calibration_path", calibration_path_, ".");

	    // set up stereo structures
	    if (do_keep_coords_) {
	    	ROS_INFO("I'm keeping the image coordinates in the point cloud");
	    }
	    // Must do stereo if calculating points
	    do_stereo_ = do_stereo_ || do_calc_points_;
	    // Must rectify if doing stereo
	    do_rectify_ = do_rectify_ || do_stereo_;

	    std::string cam_name_l = nh_.resolveName("camera") + "/left/";
	    std::string cam_name_r = nh_.resolveName("camera") + "/right/";
	    std::string cam_name_s = nh_.resolveName("camera") + "/";

	    pub_mono_l_ = it_.advertise(cam_name_l+"image_mono", 1);
		pub_mono_r_ = it_.advertise(cam_name_r+"image_mono", 1);
		if (do_rectify_) {
			pub_rect_l_ = it_.advertise(cam_name_l+"image_rect", 1);
			pub_rect_r_ = it_.advertise(cam_name_r+"image_rect", 1);
		}
		pub_color_l_ = it_.advertise(cam_name_l+"image_color", 1);
		pub_color_r_ = it_.advertise(cam_name_r+"image_color", 1);
		if (do_rectify_) {
			pub_rect_color_l_ = it_.advertise(cam_name_l+"image_rect_color", 1);
			pub_rect_color_r_ = it_.advertise(cam_name_r+"image_rect_color", 1);
		}
		if (do_stereo_)	{
			pub_disparity_image_ = it_.advertise(cam_name_s+"image_disparity", 1);
			pub_disparity_ = it_.advertise(cam_name_s+"disparity", 1);
		}

		if (do_calc_points_) {
			pub_pts_ = nh_.advertise<sensor_msgs::PointCloud>(cam_name_s+"points", 1);
			pub_pts_grid_ = nh_.advertise<sensor_msgs::PointCloud>( cam_name_s+"points_grid",1);
		}

		// Subscribe to synchronized Image & CameraInfo topics
		image_sub_l.subscribe(it_, cam_name_l + "image_raw", 1);
		info_sub_l.subscribe(nh_, cam_name_l + "camera_info", 1);
		image_sub_r.subscribe(it_, cam_name_r + "image_raw", 1);
		info_sub_r.subscribe(nh_, cam_name_r + "camera_info", 1);
		sync_.connectInput(image_sub_l, info_sub_l, image_sub_r, info_sub_r);
		sync_.registerCallback(boost::bind(&BumblebeeStereo::imageCallback, this, _1, _2, _3, _4));
		calibration_sub_ = nh_.subscribe(cam_name_s+"calibration", 1, &BumblebeeStereo::calibrationCallback, this);

	}

	~BumblebeeStereo()
	{
		cvReleaseImage(&red_);
		cvReleaseImage(&green_);
		cvReleaseImage(&blue_);

		cvReleaseImageHeader(&red_header_);
		cvReleaseImageHeader(&green_header_);
		cvReleaseImageHeader(&blue_header_);

		cvReleaseImage(&left_);
		cvReleaseImage(&right_);

		if (do_rectify_) {
			cvReleaseImage(&left_rect_);
			cvReleaseImage(&right_rect_);
		}

		if (do_stereo_) {
			cvReleaseImage(&disp_);
		}
	}


	void initTriclopsStructures()
	{
		TriclopsError te;

		camera_ = TriCam_REFERENCE;
		/* Fill in stuct fields */
		triclopsInput_.nrows=height_;
		triclopsInput_.ncols=width_;
		triclopsInput_.inputType=TriInp_RGB;
		triclopsInput_.rowinc=width_;

		size_t pos = calibration_settings_.find("SerialNumber ");

		if (pos==string::npos) {
			ROS_ERROR("Invalid calibration settings");
			ros::shutdown();
			exit(1);
		}
		pos += 13;
		size_t pos2 = calibration_settings_.find_first_not_of("0123456789",pos);
		string serial_no = calibration_settings_.substr(pos, pos2-pos);

		ROS_INFO("Serial no: %s", serial_no.c_str());

		string filename = calibration_path_+"/"+serial_no+".cal";
		ROS_INFO("Writing calibration to file: %s", filename.c_str());
		FILE* fh = fopen(filename.c_str() ,"w");
		if (fh==NULL) {
			ROS_ERROR("Cannot open calibration settings file");
			ros::shutdown();
			exit(1);
		}
		fwrite(calibration_settings_.c_str(), 1, calibration_settings_.size(), fh);
		fclose(fh);

		ROS_INFO("Initialize triclops context from: %s", filename.c_str());

		te =  triclopsGetDefaultContextFromFile( &triclops_, const_cast<char*>(filename.c_str()));
		HANDLE_TRICLOPS_ERROR(triclopsGetDefaultContextFromFile, te)

		te =  triclopsGetDefaultContextFromFile( &triclops_full_res_, const_cast<char*>(filename.c_str()));
		HANDLE_TRICLOPS_ERROR(triclopsGetDefaultContextFromFile, te)

		te = triclopsSetSubpixelInterpolation( triclops_, subpixel_interpolation_ ) ;
		HANDLE_TRICLOPS_ERROR(triclopsSetSubpixelInterpolation, te)

		te = triclopsSetResolutionAndPrepare( triclops_, stereo_height_, stereo_width_, width_, height_);
		HANDLE_TRICLOPS_ERROR(triclopsSetResolutionAndPrepare, te)

		te = triclopsSetResolutionAndPrepare( triclops_full_res_, height_, width_, width_, height_);
		HANDLE_TRICLOPS_ERROR(triclopsSetResolutionAndPrepare, te)

        te = triclopsSetTextureValidationThreshold( triclops_, texture_threshold_ );
		HANDLE_TRICLOPS_ERROR(triclopsSetTextureValidationThreshold, te)

        te = triclopsSetUniquenessValidationThreshold( triclops_, uniqueness_threshold_ );
		HANDLE_TRICLOPS_ERROR(triclopsSetUniquenessValidationThreshold, te)

        te = triclopsSetDisparity( triclops_, min_disparity_, max_disparity_ );
		HANDLE_TRICLOPS_ERROR(triclopsSetDisparity, te)
	}

	void initialize()
	{
		if ( fabs(double(width_)/height_ - double(stereo_width_)/stereo_height_)>1e-8) {
			ROS_ERROR("Wrong aspect ratio for the stereo_width, stereo_height parameters");
			exit(1);
		}

		red_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,1);
        green_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,1);
        blue_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,1);

		left_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,3);
        right_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,3);

		left_mono_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,1);
        right_mono_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,1);

        red_header_ = cvCreateImageHeader(cvSize(width_,height_),IPL_DEPTH_8U,1);
        green_header_ = cvCreateImageHeader(cvSize(width_,height_),IPL_DEPTH_8U,1);
        blue_header_ = cvCreateImageHeader(cvSize(width_,height_),IPL_DEPTH_8U,1);

        if (do_rectify_) {
        	left_rect_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,3);
        	right_rect_ = cvCreateImage(cvSize(width_,height_),IPL_DEPTH_8U,3);
        }

        if (do_stereo_) {
            disp_ = cvCreateImage(cvSize(stereo_width_,stereo_height_),IPL_DEPTH_16U,1);
            disp_8u_ = cvCreateImage(cvSize(stereo_width_,stereo_height_),IPL_DEPTH_8U,3);
        }

        if (do_calc_points_) {
        	right_rect_small_ = cvCreateImage(cvSize(stereo_width_,stereo_height_),IPL_DEPTH_8U,3);

        	cloud_.points.reserve(stereo_width_*stereo_height_);
        	cloud_grid_.points.reserve(stereo_width_*stereo_height_);
        	if (do_keep_coords_) {
        		cloud_.channels.resize(4);
        		cloud_grid_.channels.resize(4);
        	}
        	else {
        		cloud_.channels.resize(2);
        		cloud_grid_.channels.resize(2);
        	}
    		cloud_.channels[0].name = "rgb";
    		cloud_grid_.channels[0].name = "rgb";
    		cloud_.channels[1].name = "disparity";
    		cloud_grid_.channels[1].name = "disparity";
        	cloud_.channels[0].values.reserve(stereo_width_*stereo_height_);
        	cloud_grid_.channels[0].values.reserve(stereo_width_*stereo_height_);
        	cloud_.channels[1].values.reserve(stereo_width_*stereo_height_);
        	cloud_grid_.channels[1].values.reserve(stereo_width_*stereo_height_);
        	if (do_keep_coords_) {
        		cloud_.channels[2].name = "x";
        		cloud_grid_.channels[2].name = "x";
        		cloud_.channels[3].name = "y";
        		cloud_grid_.channels[3].name = "y";
            	cloud_.channels[2].values.reserve(stereo_width_*stereo_height_);
            	cloud_grid_.channels[2].values.reserve(stereo_width_*stereo_height_);
            	cloud_.channels[3].values.reserve(stereo_width_*stereo_height_);
            	cloud_grid_.channels[3].values.reserve(stereo_width_*stereo_height_);
        	}
        }

		initTriclopsStructures();
	}

	void calibrationCallback(const std_msgs::StringConstPtr& calibration)
	{
		calibration_settings_ = calibration->data;
		has_calibration_ = true;
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& raw_image_l,
			const sensor_msgs::CameraInfoConstPtr& cam_info_l,
			const sensor_msgs::ImageConstPtr& raw_image_r,
			const sensor_msgs::CameraInfoConstPtr& cam_info_r)
	{
		if (!has_calibration_) return;

		if (!initialized_) {
			width_ = raw_image_l->width;
			height_ = raw_image_l->height;
			initialize();
			initialized_ = true;
		}

		ros::Time stamp = raw_image_l->header.stamp;
		string frame_id_l = raw_image_l->header.frame_id;
		string frame_id_r = raw_image_r->header.frame_id;

		// debayer left
		if ((pub_color_l_ && pub_color_l_.getNumSubscribers() > 0) ||
			(pub_rect_color_l_ && pub_rect_color_l_.getNumSubscribers() > 0) ||
			(pub_mono_l_ && pub_mono_l_.getNumSubscribers() > 0) ||
			(pub_rect_l_ && pub_rect_l_.getNumSubscribers() > 0) ||
			(pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) ||
			(pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) ||
    		(pub_pts_ && pub_pts_.getNumSubscribers() > 0) ||
    		(pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0))
		{

			sensor_msgs::CvBridge bridge;
			sensor_msgs::Image img = *raw_image_l;
			img.encoding = "mono8";
			bridge.fromImage(img);
	        cvCvtColor(bridge.toIpl(), left_, CV_BayerBG2RGB);
	        publishImage(pub_color_l_, left_, frame_id_l, stamp);

	        if (pub_mono_l_ && pub_mono_l_.getNumSubscribers() > 0) {
	        	cvCvtColor(left_,left_mono_, CV_RGB2GRAY);
	        	publishImage(pub_mono_l_, left_mono_, frame_id_l, stamp);
	        }
		}

		if ((pub_color_r_ && pub_color_r_.getNumSubscribers() > 0) ||
			(pub_rect_color_r_ && pub_rect_color_r_.getNumSubscribers() > 0) ||
			(pub_mono_r_ && pub_mono_r_.getNumSubscribers() > 0) ||
			(pub_rect_r_ && pub_rect_r_.getNumSubscribers() > 0) ||
			(pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) ||
			(pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) ||
    		(pub_pts_ && pub_pts_.getNumSubscribers() > 0) ||
    		(pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0))
		{
			sensor_msgs::CvBridge bridge;
			sensor_msgs::Image img = *raw_image_r;
			img.encoding = "mono8";
			bridge.fromImage(img);
			cvCvtColor(bridge.toIpl(),right_, CV_BayerBG2RGB);
	        publishImage(pub_color_r_, right_, frame_id_r, stamp);

	        if (pub_mono_r_ && pub_mono_r_.getNumSubscribers() > 0) {
	        	cvCvtColor(right_,right_mono_, CV_RGB2GRAY);
	        	publishImage(pub_mono_r_, right_mono_, frame_id_r, stamp);
	        }
		}


	    if (do_rectify_ && (
	    		(pub_rect_color_l_ && pub_rect_color_l_.getNumSubscribers() > 0) ||
	    		(pub_rect_l_ && pub_rect_l_.getNumSubscribers() > 0) ||
	    		(pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) ||
	    		(pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) ||
	    		(pub_pts_ && pub_pts_.getNumSubscribers() > 0) ||
	    		(pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0)))
	    {
	    	rectifyImage(left_, left_rect_, TriCam_LEFT);
        	publishImage(pub_rect_color_l_, left_rect_, frame_id_l, stamp);

	    	if (pub_rect_l_ && pub_rect_l_.getNumSubscribers() > 0) {
	        	cvCvtColor(left_rect_,left_mono_, CV_RGB2GRAY);
	        	publishImage(pub_rect_l_, left_mono_, frame_id_l, stamp);
	    	}
	    }

	    if (do_rectify_ && (
	    		(pub_rect_color_r_ && pub_rect_color_r_.getNumSubscribers() > 0) ||
   	    		(pub_rect_r_ && pub_rect_r_.getNumSubscribers() > 0) ||
	    		(pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) ||
	    		(pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) ||
	    		(pub_pts_ && pub_pts_.getNumSubscribers() > 0) ||
	    		(pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0)))
	    {
	    	rectifyImage(right_, right_rect_, TriCam_RIGHT);
        	publishImage(pub_rect_color_r_, right_rect_, frame_id_r, stamp);

	    	if (pub_rect_r_ && pub_rect_r_.getNumSubscribers() > 0) {
	        	cvCvtColor(right_rect_, right_mono_, CV_RGB2GRAY);
	        	publishImage(pub_rect_r_, right_mono_, frame_id_r, stamp);
	    	}
	    }

	    if (do_stereo_ && (
	    		(pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) ||
	    		(pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) ||
	    		(pub_pts_ && pub_pts_.getNumSubscribers() > 0) ||
	    		(pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0)))
	    {
	    	processStereo();

	    	if (pub_disparity_ && pub_disparity_.getNumSubscribers() > 0) {
	    		publishImage(pub_disparity_, disp_, frame_id_r, stamp);
	    	}
	    	if (pub_disparity_image_ && pub_disparity_image_.getNumSubscribers() > 0) {
	    		publishImage(pub_disparity_image_, disp_8u_, frame_id_r, stamp);
	    	}
	    	if (do_calc_points_ && (pub_pts_ && pub_pts_.getNumSubscribers() > 0)) {
	    		cloud_.header.stamp = stamp;
	    		cloud_.header.frame_id = frame_id_r;
	    		pub_pts_.publish(cloud_);
	    	}
	    	if (do_calc_points_ && (pub_pts_grid_ && pub_pts_grid_.getNumSubscribers() > 0)) {
	    		cloud_grid_.header.stamp = stamp;
	    		cloud_grid_.header.frame_id = frame_id_r;
	    		pub_pts_grid_.publish(cloud_grid_);
	    	}
	    }

	}


	void rectifyImage(IplImage* src, IplImage* dst, TriclopsCamera camera)
	{
        cvSplit( src, red_, green_, blue_,0);

        triclopsInput_.u.rgb.red   = red_->imageData;
        triclopsInput_.u.rgb.green = green_->imageData;
        triclopsInput_.u.rgb.blue  = blue_->imageData;

        /* The actual rectification call */
        TriclopsError te = triclopsRectifyColorImage( triclops_full_res_, camera, &triclopsInput_, &colorImage_ );
        HANDLE_TRICLOPS_ERROR(triclopsRectifyColorImage, te)

        red_header_->imageData = (char*)colorImage_.red;
        green_header_->imageData = (char*)colorImage_.green;
        blue_header_->imageData = (char*)colorImage_.blue;

        cvMerge( red_header_, green_header_, blue_header_, 0, dst );
	}


	void processStereo()
	{
		if (do_stereo_) {
			TriclopsError te;
			// using green channel since it contains
			// approx. 70% of the monochrome signal
			cvSplit( right_, 0, red_, 0,0);
			cvSplit( left_, 0, green_, 0,0);

			triclopsInput_.u.rgb.red   = red_->imageData;
			triclopsInput_.u.rgb.green = green_->imageData;
			triclopsInput_.u.rgb.blue  = blue_->imageData;

			te = triclopsPreprocess( triclops_, &triclopsInput_ );
	        HANDLE_TRICLOPS_ERROR(triclopsPreprocess, te)

			te = triclopsStereo( triclops_ );
	        HANDLE_TRICLOPS_ERROR(triclopsStereo, te)

			te = triclopsGetImage16( triclops_, TriImg16_DISPARITY, TriCam_REFERENCE, &depthImage16_ );
	        HANDLE_TRICLOPS_ERROR(triclopsGetImage16, te)

	        unsigned short* disp16_ptr = (unsigned short*)disp_->imageData;
	        char* disp_ptr = disp_8u_->imageData;
	        unsigned short* disparityPixel = depthImage16_.data;

        	for( int i=0; i<depthImage16_.nrows*depthImage16_.ncols; ++i, ++disparityPixel, ++disp16_ptr, disp_ptr+=3){
        		if ( *disparityPixel < 0xFF00 && *disparityPixel>0) {
        			*disp16_ptr = *disparityPixel;
        			int k = 3 * (0xff & (*disparityPixel)>>8);
        			disp_ptr[0] = dmap[k];
        			disp_ptr[1] = dmap[k+1];
        			disp_ptr[2] = dmap[k+2];
        		}
        		else {
        			*disp16_ptr = 0;
        			 disp_ptr[0] = disp_ptr[1] = disp_ptr[2] = 0;
        		}
        		if ( i%depthImage16_.ncols==0) {
        			disp_ptr += (disp_8u_->widthStep-depthImage16_.ncols*3);
        			disp16_ptr += (disp_->widthStep/2-depthImage16_.ncols);
        		}
        	}
			//triclopsSaveImage( &depthImage, "depth.pgm" );
			//triclopsSaveImage16( &depthImage16, "depth.pgm" );

	        if (do_calc_points_) {
	        	cvResize(right_rect_, right_rect_small_, CV_INTER_NN );

	        	cloud_.points.clear();
	        	cloud_grid_.points.clear();
		        cloud_.channels[0].values.clear();
		        cloud_grid_.channels[0].values.clear();
		        cloud_.channels[1].values.clear();
		        cloud_grid_.channels[1].values.clear();
		        if (do_keep_coords_) {
			        cloud_.channels[2].values.clear();
			        cloud_grid_.channels[2].values.clear();
			        cloud_.channels[3].values.clear();
			        cloud_grid_.channels[3].values.clear();
		        }
	        	unsigned short* disparityPixel = depthImage16_.data;
	        	unsigned char* rectPixel = (unsigned char*)right_rect_small_->imageData;

	        	geometry_msgs::Point32 p;

	        	for( int row=0; row<depthImage16_.nrows; ++row ){
	        		for( int col=0; col<depthImage16_.ncols; ++col ){
	        			if ( *disparityPixel < 0xFF00 && *disparityPixel>0) {

	        				te = triclopsRCD16ToXYZ( triclops_, row, col, *disparityPixel, &p.x, &p.y, &p.z );
	        		        HANDLE_TRICLOPS_ERROR(triclopsRCD16ToXYZ, te)

	        				cloud_.points.push_back(p);
	        		        cloud_grid_.points.push_back(p);
	        		        int rgb = (rectPixel[2] << 16) | (rectPixel[1] << 8) | rectPixel[0];
	        		        float float_rgb = *(float*)&rgb;
	        		        cloud_.channels[0].values.push_back(float_rgb);
	        		        cloud_grid_.channels[0].values.push_back(float_rgb);
	        		        cloud_.channels[1].values.push_back(*disparityPixel);
	        		        cloud_grid_.channels[1].values.push_back(*disparityPixel);
	        		        if (do_keep_coords_) {
	        		        	cloud_.channels[2].values.push_back(col);
	        		        	cloud_grid_.channels[2].values.push_back(col);
	        		        	cloud_.channels[3].values.push_back(row);
	        		        	cloud_grid_.channels[3].values.push_back(row);
	        		        }
	        			}
	        			else{
	        				p.x = 0.0;
	        				p.y = 0.0;
	        				p.z = 0.0;
	        				cloud_grid_.points.push_back(p);
	        				cloud_grid_.channels[0].values.push_back(0xFFFF);
	        				cloud_grid_.channels[1].values.push_back(0xFFFF);

	        				if (do_keep_coords_) {
	        					cloud_grid_.channels[2].values.push_back(col);
	        					cloud_grid_.channels[3].values.push_back(row);
	        				}
	        			}

	        			disparityPixel++;
	        			rectPixel += 3;
	        		}
	        		rectPixel += (right_rect_small_->widthStep-depthImage16_.ncols*3);
	        	}
	        }
		}
	}


	void publishImage(const image_transport::Publisher& pub, IplImage* img, string frame_id, const ros::Time& stamp)
	{
		sensor_msgs::Image image;
		sensor_msgs::CvBridge::fromIpltoRosImage(img, image);
		image.header.stamp = stamp;
		image.header.frame_id = frame_id;
		pub.publish(image);
	}
};





int main( int argc, char** argv )
{
	ros::init(argc, argv, "bumblebee_stereo");

	BumblebeeStereo bbs;

	ros::spin();

    return 0;
}
