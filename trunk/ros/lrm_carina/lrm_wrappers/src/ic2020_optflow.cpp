/************************************************************************
 * 
 * This source code is part of the IC2020 SLAM System
 * 
 * IC2020 Copyright (C) 2011
 * Sean Anderson
 * Kirk MacTavish
 * 
 * IC2020 is licenced under the Creative Commons License
 * Attribution-NonCommercial-ShareAlike 2.5 Canada
 * (CC BY-NC-SA 2.5).
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 ************************************************************************/
 
/* 
 * <ic2020_optflow/src/ic2020_optflow.cpp>
 * 
 * revision 1.0
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

#include "opencv/highgui.h"
#include "opencv/cv.h"

#include <iostream>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

#include "ic2020_vodom/keyframe.h"
#include "VisualOdometry.h"
#include "Keyframe.h"
#include "CornerHelper.h"

/////////////////////
// Fixed Constants //
/////////////////////

#define BW_IPL_PXL_BYTES 1
#define CLR_IPL_PXL_BYTES 3

///////////////////////////
// Adjustable Parameters //
///////////////////////////

// METHOD 1 (NOT USED)
// one screen width is approx 1.2m at m dist
#define KEYFRAME_BREAK_DIST 0.2 //average distance (m) that points travel before breaking to new keyframe

// METHOD 2 (USED)
#define KEYFRAME_BREAK_TRANS 0.2 //average distance (m) that camera travels to break keyframe
#define KEYFRAME_BREAK_PX    75 //100 //average distance (px) that features travel to break keyframe

const bool bw_not_color = false;
int IPL_PXL_BYTES = bw_not_color?BW_IPL_PXL_BYTES:CLR_IPL_PXL_BYTES;
std::string IPL_IMG_TYPE = bw_not_color?"mono8":"bgr8";

CornerHelper cornHelp;

static CvScalar colors[] = {
	{{0,0,255}},
	{{0,128,255}},
	{{0,255,255}},
	{{0,255,0}},
	{{255,128,0}},
	{{255,255,0}},
	{{255,0,0}},
	{{255,0,255}},
	{{255,255,255}}
};

// ROS Image Pipeline stuff
sensor_msgs::CvBridge bridge;

// Get an OpenCV camera handle
IplImage* view_im;

// Point Cloud Data
Keyframe* keyframeA;
Keyframe* keyframeB;

bool new_B = false;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // free mem
    if (keyframeB->points != NULL) { delete [] keyframeB->points; }
    keyframeB->points = NULL;
    
    keyframeB->numberOf3DPoints = msg->width*msg->height;
    if (keyframeB->numberOf3DPoints > 0)
    {
        keyframeB->point_step = msg->width;
        keyframeB->points = new PointColor[keyframeB->numberOf3DPoints];
        memcpy(&keyframeB->points[0], &msg->data[0], keyframeB->numberOf3DPoints*sizeof(PointColor));              
    }   
    
    // Switch colors
    uint8_t temp;
    for (unsigned int i = 0; i < keyframeB->numberOf3DPoints; i++)
    {
            
        temp = keyframeB->points[i].b;
        keyframeB->points[i].b = keyframeB->points[i].r;
        keyframeB->points[i].r = temp;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    // Get Image Data
    IplImage* temp = NULL;
    try { temp = bridge.imgMsgToCv(msg_ptr, IPL_IMG_TYPE.c_str()); }
    catch (sensor_msgs::CvBridgeException error) { ROS_ERROR("error"); }

    // If first time, allocate space for image
    if (temp != NULL && keyframeB->im == NULL) {
        CvSize s = cvGetSize(temp);
        keyframeB->im = cvCreateImage(s, 8, IPL_PXL_BYTES);
    }

    // Copy Data Over
    keyframeB->height = keyframeB->im->height;
    keyframeB->width = keyframeB->im->width;
    cvCopy(temp, keyframeB->im);
    new_B = true;
}

void imuCallback(const sensor_msgs::Imu& msg_ptr)
{
    keyframeB->imux = msg_ptr.linear_acceleration.x;
    keyframeB->imuy = msg_ptr.linear_acceleration.y;
    keyframeB->imuz = msg_ptr.linear_acceleration.z;
}

void BlockWhileWaitingForVideo()
{
    printf("Optical flow waiting to make connection with kinect video stream...\n");     
    while (1) {
        ros::spinOnce();
        if (keyframeB->im != NULL)
            break;
    }
    printf("Optical flow video streams found...\n"); 
    
    printf("Optical flow waiting to make connection with kinect depth stream...\n");     
    while (1) {
        ros::spinOnce();
        if (keyframeB->numberOf3DPoints > 0)
            break;
    }
    printf("Optical flow depth stream found...\n"); 
}

int main(int argc, char **argv)
{
    int keyframe_num = 0;
    
    // Initialize ROS
    ros::init(argc, argv, "ic2020_optflow");
    ros::NodeHandle n;
    keyframeB = new Keyframe();
    keyframe_num++;
    keyframeB->keyframe_num = keyframe_num;
        
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    //image_sub = it.subscribe("/kinect/rgb/image_rect_color", 1, &imageCallback);
    image_sub = it.subscribe("/kinect/rgb/image_raw", 1, &imageCallback);
    ros::Subscriber pc_sub = n.subscribe("/kinect/rgb/points2", 5, pointcloudCallback);  
    ros::Subscriber imu_sub = n.subscribe("/kinect/imu", 5, &imuCallback); 
    
    ros::Publisher flow_pub = n.advertise<ic2020_vodom::keyframe>("/flow/keyframes", 5);
    ros::Publisher contflow_pub = n.advertise<ic2020_vodom::keyframe>("/flow/contframes", 5);
    
    // Wait for video streams to be up
    BlockWhileWaitingForVideo();   
    
    // Init Stuff
    cornHelp.Init(keyframeB->im);
	view_im = cvCreateImage( cvSize(keyframeB->im->width, keyframeB->im->height), 8, 3 );
    cvNamedWindow("OpticalFlow", CV_WINDOW_AUTOSIZE);
    
    // Publish initial message
    for (unsigned int i = 0; i < 9; i++) { keyframeB->rotation[i] = 0.0f; }
    for (unsigned int i = 0; i < 3; i++) { keyframeB->translation[i] = 0.0f; }
    keyframeB->PublishKeyframe(&contflow_pub);
    
    keyframeA = keyframeB;
    keyframeB = new Keyframe();
    keyframe_num++;
    keyframeB->keyframe_num = keyframe_num;
    
    float avg_length = 0;
    
    std::vector<feature> cornA;
    std::vector<feature> cornB;
    std::vector<char> pairs;
    
    // Main loop
    printf("Optical flow starting main loop ...\n");
    bool force_break = false;
    while (ros::ok())
    {
        char c = cvWaitKey(10);
        if (c == 'Q' || c == 'q')
            break;
        else if (c == 'K' || c=='k')
            force_break = true;
    
        //fprintf(stderr,"\n\n");
        //printf ("\33[2J");
        printf("Curr: keyframe %i ...\n", keyframe_num);
        
        // Get Images
        ros::spinOnce();

        if ( keyframeB->im == NULL || !(keyframeB->numberOf3DPoints > 0) )
            continue;

        // Copy image for viewing and grayscale
        cvCopy( keyframeB->im, view_im );
        
        // Check that there is a new frame
        if (new_B == true) {

            // Get Corners on New Image
            cornHelp.FindFeatures(keyframeA, keyframeB, &cornA, &cornB, &pairs);
            
            // Copy Corner Info Into Keyframes
            keyframeA->numCorn2 = cornA.size();
            keyframeA->corn2 = cornA;
            keyframeB->numCorn1 = cornB.size();
            keyframeB->corn1 = cornB;
            keyframeB->status = pairs;
            
            // RANSAC
            bool ransac_passed = true;
            std::vector<unsigned int> filtered_corn_pairs;
            if (keyframeA->numCorn2 == keyframeB->numCorn1) {
                if (!VisualOdometry::RANSAC6DFastCorn(&keyframeA->corn2[0], &keyframeB->corn1[0], 
                                                      &keyframeB->status[0], keyframeB->numCorn1, &filtered_corn_pairs,
                                                      keyframeB->im->width, keyframeB->im->height, 10, 10, 1))
                {
                    printf("RANSAC MATCHES FEATURE # AREN'T EQUAL OR LESS THAN 10 FEATURES \n");
                    ransac_passed = false;
                }
            } else {
                printf("WTF KEYFRAME A's FORWARD ST FEATURES != KEYFRAME B's BACK ST FEATURES \n");
                ransac_passed = false;
            }

            // CREATE VECTOR OF MATCHES
            std::vector<feature> matchesA2;
            std::vector<feature> matchesB2;
            if (ransac_passed)
            {
                // ADD IN CORNER MATCHES
                for(unsigned int i = 0; i < keyframeB->numCorn1; i++ )
                {
                    if (filtered_corn_pairs[i] > 0) {
                        matchesA2.push_back(keyframeA->corn2[i]);
                        matchesB2.push_back(keyframeB->corn1[i]);
                    } else {
                        keyframeB->status[i] = 0;
                    }
                }
                
                if (matchesA2.size() < 10)
                {
                    printf("POST RANSAC MATCHES ARE LESS THAN 10 \n");
                    ransac_passed = false;
                }
                
                if (ransac_passed)
                {
                    // Least Squares
                    double rdata[9];
                    double translation[3];
                    VisualOdometry::ArunLeastSquares(&matchesA2, &matchesB2, rdata, translation);      
                    for (unsigned int i = 0; i < 9; i++) { keyframeB->rotation[i] = rdata[i]; }
                    for (unsigned int i = 0; i < 3; i++) { keyframeB->translation[i] = translation[i]; }
                    
                    // Publish Frame B
                    keyframeB->PublishKeyframe(&contflow_pub);
                }
            }
            
            // Go through corners and get distance travelled stuff
            double total_length = 0.0;
            double number_of_corners = 0.0;
            for( int i = 0; i < cornA.size(); i++ ) {
                if( pairs[i] == 0 ) { continue; }

                CvPoint p0 = cvPoint(cvRound( cornA[i].point2D[0] ), cvRound( cornA[i].point2D[1] ));
                CvPoint p1 = cvPoint(cvRound( cornB[i].point2D[0] ), cvRound( cornB[i].point2D[1] ));
                cvLine( view_im, p0, p1, CV_RGB(255,0,0),2 );
	            
                /*double dx = (double)(cornA[i].point3D[0] - cornB[i].point3D[0]);
	            double dy = (double)(cornA[i].point3D[1] - cornB[i].point3D[1]);
	            double dz = (double)(cornA[i].point3D[2] - cornB[i].point3D[2]);
                double length = sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));
                length = length/sqrt((double)(cornA[i].point3D[2]+cornB[i].point3D[2])/2.0);*/
                
                double dx = (double)(cornA[i].point2D[0] - cornB[i].point2D[0]);
	            double dy = (double)(cornA[i].point2D[1] - cornB[i].point2D[1]);
                double length = sqrt(pow(dx,2.0)+pow(dy,2.0));
                
                total_length = total_length + length;
                number_of_corners = number_of_corners + 1.0;
            }
            // Calculate Average 3D distance
            if (number_of_corners > 0.0f)
                avg_length = total_length/number_of_corners;
            else
                avg_length = 0.0f;
            
            if (ransac_passed)
            {
                // Print Green Lines Over RANSAC Points
                for (unsigned int i = 0; i < matchesA2.size(); i++) {
                    float lx = matchesB2[i].point2D[0];
                    float ly = matchesB2[i].point2D[1];
                    float lx2 = matchesA2[i].point2D[0];
                    float ly2 = matchesA2[i].point2D[1];
                    CvPoint p0 = cvPoint(cvRound(lx), cvRound(ly));
                    CvPoint p1 = cvPoint(cvRound(lx2), cvRound(ly2));
                    cvLine( view_im, p0, p1, CV_RGB(0,255,0),2 );
                }
            }
            
            // Calculate delDist travelled by camera
            double distTrans = sqrt(pow(keyframeB->translation[0],2.0)+pow(keyframeB->translation[1],2.0)+pow(keyframeB->translation[2],2.0));

            // Avg Length is Greater Than Breaking Point
            //if (avg_length > KEYFRAME_BREAK_DIST)
            if (avg_length > KEYFRAME_BREAK_PX || distTrans > KEYFRAME_BREAK_TRANS || force_break)
            {
                force_break = false;
                
                // Copy Corner Info Into Keyframes
                keyframeA->PublishKeyframe(&flow_pub);
                delete keyframeA;
                keyframeA = keyframeB;
                keyframeB = new Keyframe();
                keyframe_num++;
                keyframeB->keyframe_num = keyframe_num;
            }
        }

        cvShowImage("OpticalFlow", view_im); 
    }
    
    return 0;
}

