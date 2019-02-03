#ifndef MOTION_DUMMY_H
#define MOTION_DUMMY_H

#include <ros/ros.h>
#include <robotcontrol/Motion.h>
#include <motion/definition.h>

class MotionDummy{
	private:
		ros::NodeHandle node_handle_;
		ros::NodeHandle priv_node_handle_;

		ros::Publisher motion_pub_;
	public:
		MotionDummy();
		~MotionDummy();
		void pubMotion();
};

#endif