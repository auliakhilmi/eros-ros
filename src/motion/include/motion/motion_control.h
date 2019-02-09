#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <ros/ros.h>
#include "ros/master.h"
#include <vector>
#include <string>
#include <fstream>
#include <sys/stat.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <robotcontrol/Razor.h>
#include <robotcontrol/Trajectory.h>
#include <robotcontrol/Motion.h>
#include "motion/motion4step.h"


#define degtorad 0.01745329252

#define ITERATION_FREQUENCY  (120)
#define JOINT_NUM   6

class MotionControl{
	private:
		//Node Handler
		ros::NodeHandle node_handle_;
		ros::NodeHandle priv_node_handle_;

		//Publisher
		ros::Publisher joint_states_pub_;
		ros::Publisher frequency_counter_;
		ros::Publisher time_counter_;

		//Subscriber
		ros::Subscriber razor_sub_;
		ros::Subscriber motion_sub_;
		ros::Subscriber dservo;

		Motion4step m4;

		double pub_period_;

		std::string robot_name_;

		int secs,last_secs,freq_count,time_count;

		unsigned char iii,dat[68],y=0;
		int fdGrk;
		struct Data{
			unsigned char data;
		};

	public:
		MotionControl();
		~MotionControl();
		
		void publishCallback(const ros::TimerEvent&);
		double getPublishPeriod(){return pub_period_;}

		void ChangeState();
		void State();

		void StandUp();
		void StandUp_End();

		void InertialFeedback();
		void basicMotion();
		void syncWrite();
		void setDXLData(unsigned int ID, unsigned int datae, unsigned int GoalPos);
		void setDXLData1(unsigned int ID, unsigned int datae, unsigned int GoalPos);
		bool impUSBRX(){
			char file[]={"/home/eros/param/USBRX.eros"};
			ifstream buka(file);
			string line;
			if (buka.is_open()){
				if (buka.good()){
					char buffer[50]={};
					getline(buka,line);
					for(int i=0; i<line.length(); i++){
						buffer[i]=line[i];
					}
					memcpy(USBRX,buffer,sizeof(USBRX));
					printf("iki>>%s--\n\n",buffer);
				}
				buka.close();
				return 1;
			} else return 0;
		}

	private:
		void initPublisher();
		void initSubscriber();
		void initSerial();

		void sendAllServo();
		void getDynamixelInst();
		void setOperatingMode();
		void setSyncFunction();
		void razorCallback(const robotcontrol::Razor::ConstPtr &msg);
		void motionCallback(const robotcontrol::Motion::ConstPtr &msg);
		void setservo(const std_msgs::Int32::ConstPtr &msg);
		int configure_port(int fd);
};

#endif //MOTION_CONTROL_H