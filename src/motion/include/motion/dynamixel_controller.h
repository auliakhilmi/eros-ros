/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>

#include <vector>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <sensor_msgs/JointState.h>

namespace dynamixel
{
#define ITERATION_FREQUENCY  (25)
#define JOINT_NUM   4

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states_sub_;

  // ROS Service Server

  // ROS Service Client

  // Dynamixel Workbench Parameters
  std::string robot_name_;
  float protocol_version_;

  DynamixelWorkbench *joint_controller_;

  std::vector<uint8_t> joint_id_;

  std::string joint_mode_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool control_loop();

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void getDynamixelInst();
  void setOperatingMode();
  void setSyncFunction();
  void readPosition(double *value);
  void readVelocity(double *value);
  void updateJointStates();

  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif //DYNAMIXEL_CONTROLLER_H
