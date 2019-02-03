#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
int main(int argc, char **argv)
{
	ros::init(argc, argv,"lengankanan");
	ros::NodeHandle node_obj3, node_obj5;
	ros::Publisher joint3 = node_obj3.advertise<std_msgs::Float64>("/joint3_controller/command",10);
	ros::Publisher joint5 = node_obj5.advertise<std_msgs::Float64>("/joint5_controller/command",10);
    ros::Rate loop_rate(60);
    int step = 0;
    float datasservo = 0.0;

	while (ros::ok())
	{
        std_msgs::Float64 msg, msg2;
		msg.data = datasservo;//number_count;
        msg2.data = datasservo;//number_count;

		ROS_INFO("%f",msg.data);
		joint3.publish(msg);
        joint5.publish(msg2);
		ros::spinOnce();
        if(step == 0) datasservo += 0.1;
        else datasservo -= 0.1;

        if(datasservo > 1.0) step = 1;
        if(datasservo < -1.5) step = 0;
			
		loop_rate.sleep();
	}
return 0;
}
