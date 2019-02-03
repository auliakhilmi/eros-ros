#include "motion/motion_dummy.h"

robotcontrol::Motion motion;
bool tick;

MotionDummy::MotionDummy()
	:node_handle_(""),
	priv_node_handle_("~")
{
	motion_pub_ = node_handle_.advertise<robotcontrol::Motion>("/robotcontrol/motion", 100);
	ROS_INFO("motion_dummy [node] : Init OK!");
}

MotionDummy::~MotionDummy(){
	motion.motion=0;
	motion.state=READY;
	motion.headX=2048;
	motion.headY=2048;
	motion.tick=0;
	pubMotion();
	ROS_INFO("motion_dummy .. Shutdown!");
	ros::shutdown();
}

void MotionDummy::pubMotion(){
	motion_pub_.publish(motion);
}

int main(int argc,char** argv){
	ros::init(argc, argv, "motion_dummy");
	MotionDummy motion_dummy;
	ros::NodeHandle node_handle("");
	int t;

	ros::Rate rate(1000);

	motion.motion=10;
	motion.state=WALK;
	motion.headX=2048;
	motion.headY=2048;

	t=0;
	while(ros::ok()){
		motion_dummy.pubMotion();
		if(t<4000){
			motion.motion=10;
		}else
		if(t<9000){
			motion.state=KICK;
			motion.motion=6;
		}else{
			motion.state=WALK;
			motion.motion=0;
		}
		t++;
		ROS_INFO("t:%d",t);
		rate.sleep();
		ros::spinOnce();
	}
}