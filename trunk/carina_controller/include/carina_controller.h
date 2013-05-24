/*
 * carina_controller
 * 
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
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
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
 * 
 * \brief Control direction axes pair 2/4 and traction motors 4W for the summit Ackerman single/dual direction drive kinematics
    associate direction/traction wheel joints with motors, apply the control correction in closed loop for the motors and set the commands received by the        
    joystick/keyboard teleop node.
 */

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
//#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define CARINA_D_WHEELS_M           1.64     // distance from front to back axis, car-like kinematics
#define CARINA_WHEEL_DIAMETER	    0.48     // wheel avg diameter - check 
					      

namespace carina_controller_ns
{
/*! \class
\brief This class inherits from Controller and implements the actual controls.
*/
class CarinaControllerClass: public pr2_controller_interface::Controller
{
 private:
  // Joint states
  pr2_mechanism_model::JointState* joint_state_flw_dir_;
  pr2_mechanism_model::JointState* joint_state_frw_dir_;
  //pr2_mechanism_model::JointState* joint_state_blw_dir_;
  //pr2_mechanism_model::JointState* joint_state_brw_dir_;
  pr2_mechanism_model::JointState* joint_state_flw_;
  pr2_mechanism_model::JointState* joint_state_frw_;
  pr2_mechanism_model::JointState* joint_state_blw_;
  pr2_mechanism_model::JointState* joint_state_brw_;

  pr2_mechanism_model::JointState* joint_state_drw_;

  double init_pos_drw_;
  // Joint Positions
  double init_pos_flw_dir_;
  double init_pos_frw_dir_;
  //double init_pos_blw_dir_;
  //double init_pos_brw_dir_;
  double init_pos_blw_;
  double init_pos_flw_;
  double init_pos_brw_;
  double init_pos_frw_;
  // Joint Speeds
  double init_vel_blw_;
  double init_vel_flw_;
  double init_vel_brw_;
  double init_vel_frw_;
   
  // Robot Speeds
  double linearSpeedMps_;
  double alfaRad_;
  double angularSpeedRads_;  // probablemente no haga falta

  // Robot Positions
  double robot_pose_px_;
  double robot_pose_py_;
  double robot_pose_pa_;

  // External references are alfa_ref, v_ref 
  double alfa_ref_;
  double v_ref_;

  // Control mode flags
  bool dual_mode_;
  bool paralel_mode_;  

  // Left-right reference
  int left_right;

  // Frequency
  double frequency;


  /*!
   * \brief callback message, used to remember where the base is commanded to go
  */
  geometry_msgs::Twist base_vel_msg_;
  sensor_msgs::Joy joy_msg_;

  ros::NodeHandle node_;
  
  ros::Subscriber cmd_sub_;  // To handle references
  ros::Subscriber joy_sub_;  // To handle buttons

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
  
  //set control action (base velocity command)
  void setCommand(const geometry_msgs::Twist &cmd_vel);

  // set control mode 
  void setControlMode(const sensor_msgs::Joy &joy_msg);

  //deal with Twist commands 
  void commandCallback(const geometry_msgs::TwistConstPtr &msg);

  //deal with Joy commands
  void joystickCallback(const sensor_msgs::JoyConstPtr &msg);

private:
  double saturation(double u, double min, double max);

};
} 

