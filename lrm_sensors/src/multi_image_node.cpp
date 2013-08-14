#include <string.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv_bridge;
using namespace cv;

#define LB_WIDTH	512
#define LB_HEIGHT	384
#define NUM_IMGS    6

image_transport::Publisher pub[NUM_IMGS];
int seq = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    bool have_subsc = false;
    for(int i=0; i<NUM_IMGS; i++) {
        if(pub[i].getNumSubscribers()>0) {
            have_subsc = true;
            break;
        }
    }
    if(have_subsc) {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = toCvShare(msg, "mono8");
            int sz = 0;

            sensor_msgs::Image omsg;
            omsg.header.frame_id = "/ladybug";
            omsg.header.seq = msg->header.seq;
            omsg.header.stamp = msg->header.stamp;

            for(int i=0; i<NUM_IMGS; i++) {

                const cv::Mat_<uint8_t> img_0(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
                sz+=(LB_WIDTH*LB_HEIGHT);
                const cv::Mat_<uint8_t> img_1(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
                sz+=(LB_WIDTH*LB_HEIGHT);
                const cv::Mat_<uint8_t> img_2(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
                sz+=(LB_WIDTH*LB_HEIGHT);
                const cv::Mat_<uint8_t> img_3(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
                sz+=(LB_WIDTH*LB_HEIGHT);

                if(pub[i].getNumSubscribers()>0) {
                    omsg.width = LB_HEIGHT;
                    omsg.height = LB_WIDTH;
                    omsg.encoding = "rgb8";
                    omsg.is_bigendian = false;
                    omsg.step = omsg.width*3;
                    omsg.data.resize(LB_WIDTH*LB_HEIGHT*3);
                    CvImagePtr cp = toCvCopy(omsg, "rgb8");
                    Mat src[] = {img_3, img_1, img_0};
                    cv::merge(src, 3, cp->image);
                    pub[i].publish(cp->toImageMsg());
                }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            //ROS_ERROR("error:" << e);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_color");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    pub[0] = it.advertise("/camera/image_color/0", 1);
    pub[1] = it.advertise("/camera/image_color/1", 1);
    pub[2] = it.advertise("/camera/image_color/2", 1);
    pub[3] = it.advertise("/camera/image_color/3", 1);
    pub[4] = it.advertise("/camera/image_color/4", 1);
    pub[5] = it.advertise("/camera/image_color/5", 1);

    ros::spin();
    exit(0);
}

