#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {




    ros::init(argc, argv, "intial_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    joint_state.name.push_back(std::string("joint_back_left_wheel"));
    joint_state.position.push_back(0);
    joint_state.name.push_back(std::string("joint_back_right_wheel"));
    joint_state.position.push_back(0);
    joint_state.name.push_back(std::string("joint_front_left_wheel"));
    joint_state.position.push_back(0);
    joint_state.name.push_back(std::string("joint_front_right_wheel"));
    joint_state.position.push_back(0);


    while (ros::ok()) {
        //update joint_state
        joint_state.header.frame_id="/base_link";
        joint_state.header.stamp = ros::Time::now();

        /* joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="swivel";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="tilt";
        joint_state.position[1] = tilt;
        joint_state.name[2] ="periscope";
        joint_state.position[2] = height;
        */

        joint_state.name.push_back(std::string("joint_back_left_wheel"));
        joint_state.position.push_back(0);
        joint_state.name.push_back(std::string("joint_back_right_wheel"));
        joint_state.position.push_back(0);
        joint_state.name.push_back(std::string("joint_front_left_wheel"));
        joint_state.position.push_back(0);
        joint_state.name.push_back(std::string("joint_front_right_wheel"));
        joint_state.position.push_back(0);

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);
        
        // This will adjust as needed per iteration
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

