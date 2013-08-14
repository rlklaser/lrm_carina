/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ihm/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ihm {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ){


    ros::init(argc,argv,"ihm");
    ros::NodeHandle n;

    nh = new ros::NodeHandle(n);

    nh_priv = new ros::NodeHandle("~");

    // Add your ros communications here.
    can_sub = nh->subscribe<atuacao::VehicleState>("vehicle_state", 10,	&QNode::canCallback, this);
    //throttle_sub = nh->subscribe<atuacao::Throttle>("throttle_commands", 10,	&QNode::throttleCallback, this);
    brake_sub = nh->subscribe<atuacao::Brake>("brake_commands", 10,	&QNode::brakeCallback, this);


    velocity = 0.0;
    brake = 0.0;
    throttle =0.0;
    handBrake = 0;
    gearShift = 0;

    emit velocitySig(velocity);
    emit throttleSig((int)throttle);
    emit brakeSig((int)brake);
    emit handBrakeSig((int)handBrake);
    emit engineSpeedSig((int)engine_speed);
    emit gearShiftSig((int)gearShift);
    start();
}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
    delete nh;
    delete nh_priv;
}




void QNode::run() {
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::canCallback(const atuacao::VehicleState::ConstPtr& state){


    if (velocity != state->velocity) {
        velocity = state->velocity;
        emit velocitySig(velocity);
    }

    if (throttle != state->throttle) {
        throttle = state->throttle;
        emit throttleSig((int)throttle);
    }

    if (brake != state->brake) {
        if ((brake == 0) || (brake==1)) {
            brake = state->brake;
            emit brakeSig((int)brake*100);
        }
    }

    if (handBrake != state->handbrake) {
        handBrake = state->handbrake;
        emit handBrakeSig(handBrake);
    }

    if (engine_speed != state->engine_speed) {
        engine_speed = state->engine_speed;
        emit engineSpeedSig(engine_speed);
    }

    if (gearShift != state->car_gear) {
        gearShift = state->car_gear;
        emit gearShiftSig(gearShift);
    }

}

void QNode::throttleCallback(const atuacao::Throttle::ConstPtr& state){
    if (throttle != state->value) {
        throttle = state->value;
        emit throttleSig((int)throttle);
    }


}

void QNode::brakeCallback(const atuacao::Brake::ConstPtr& state){
    if (brake != state->value) {
        brake = state->value;
        emit brakeSig((int)brake);
    }


}


}  // namespace ihm
