//============================================================================
// Name        : robotcontrol.cpp
// Author      : khilmi@eros
// Version     : 8.0
// Copyright   : EROS Re-BORN
// Description : Dummy file for library
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/ros.h"
#include <robotcontrol/Razor.h>
#include <robotcontrol/Motion.h>
#include <robotcontrol/Trajectory.h>
#include <std_msgs/Int8.h>

using namespace std;

int main(int argc, char **argv){

	ros::init(argc, argv, "robotcontrol");

  	ros::NodeHandle n;
	ros::Rate rate(100);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
