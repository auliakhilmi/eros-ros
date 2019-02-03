#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <sstream>
using namespace std; 

ros::Publisher joint1,joint3,joint5;
std_msgs::Float64 pos1, pos3, pos5;

void number_callback(const sensor_msgs::JointState& msg)
{
	//ROS_INFO("Recieved  nama [%s]",msg->name[1]);
	//ROS_INFO("Recieved  [%d]",msg->number);
	cout << msg.position[0] << " " << msg.position[1] << " " << msg.position[2]  << endl;
	pos1.data = msg.position[0];
	pos3.data = msg.position[1];
	pos5.data = msg.position[2];

	joint1.publish(pos1);
	joint3.publish(pos3);
	joint5.publish(pos5);
}

int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"lengank_publisher");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	ros::NodeHandle node_obj1,node_obj3, node_obj5;
	joint1 = node_obj1.advertise<std_msgs::Float64>("/joint1_controller/command",10);
	joint3 = node_obj3.advertise<std_msgs::Float64>("/joint3_controller/command",10);
	joint5 = node_obj5.advertise<std_msgs::Float64>("/joint5_controller/command",10);
	//Create a publisher object
	ros::Subscriber joint_subscriber = node_obj.subscribe("joint_states",10,number_callback);
	//Spinning the node
	ros::spin();
	return 0;
}


