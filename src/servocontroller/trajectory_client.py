#!/usr/bin/env python
import roslib
roslib.load_manifest('servocontroller')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            char = self.name[0] #either 'f' or 'b'
            goal.trajectory.joint_names = ['servo_2', 'servo_3','servo_4']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(2)
            point2 = JointTrajectoryPoint()
            #goal.trajectory.points.append(point)
            point2.positions = [1,1.0,1.0]
            point2.time_from_start = rospy.Duration(4)
            point3 = JointTrajectoryPoint()
            point3.positions = [-1,-1.0,-1]
            point3.time_from_start = rospy.Duration(6)
            point4 = JointTrajectoryPoint()
            point4.positions = [1.5,1.5,-1.5]
            point4.time_from_start = rospy.Duration(8)
            goal.trajectory.points = [point,point2,point3,point4]
            self.jta.send_goal_and_wait(goal)
            print 'done'  

def main():
            arm = Joint('arm')
            arm.move_joint([0.0,0.0,0.0])
	    #arm.move_joint([1.61,0.0,0.0])
	    #arm.move_joint([1.61,0.0,-1.57])
	    #arm.move_joint([1.61,0.0,-0.59])
            #arm.move_joint([1.61,0.0,-1.57])
            #arm.move_joint([1.61,0.0,-0.59])
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
