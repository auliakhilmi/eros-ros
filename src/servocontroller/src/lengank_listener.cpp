#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <sstream>
using namespace std; 

int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"lengank_listener");
	//Created a nodehandle object
	ros::NodeHandle node_obj1,node_obj3, node_obj5;
	ros::Subscriber joint_subscriber = node_obj.subscribe("joint_states",10,number_callback);
}
