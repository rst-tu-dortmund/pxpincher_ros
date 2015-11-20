#!/usr/bin/env python

# Author Christoph Roesmann

import rospy
import roslib;
import actionlib
from trajectory_msgs.msg import *
from control_msgs.msg import *


def traj_pub():
  p = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)


  msg = JointTrajectory()
  msg.joint_names = ["arm_elbow_flex_joint", "arm_shoulder_lift_joint", "arm_shoulder_pan_joint", "arm_wrist_flex_joint"]
  
  point = JointTrajectoryPoint()
  point.positions = [1,0.6,-0.3,-0.4]
  point.time_from_start.secs = 1
  
  
  msg.points.append(point)
  msg.header.stamp = rospy.Time.now()
  
  #p.publish(msg) # publish once
  
  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.5)
    
    
    
    
    
def traj_action_client():
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    client.wait_for_server()
    
    # construct goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["arm_elbow_flex_joint", "arm_shoulder_lift_joint", "arm_shoulder_pan_joint", "arm_wrist_flex_joint"]
  
    point = JointTrajectoryPoint()
    #point.positions = [1,0.6,-0.3,-0.4]
    point.positions = [0,1.2,0,0]
    point.velocities = [0,0,0,0]
    point.accelerations = [0,0,0,0]
    point.time_from_start.secs = 2
  
    goal.trajectory.points.append(point)
    
  
    # send goal
    client.send_goal(goal)
    
    # wait for server to finish performing the action
    client.wait_for_result()
    
    result = client.get_result()
    # print results
    if result.error_code == 0:
        print "Return: Succesful"
    elif result.error_code == -1:
        print "Return: Invalid goal"
    elif result.error_code == -2:
        print "Return: Invalid joints"
    elif result.error_code == -3:
        print "Return: Old header timestamp"
    elif result.error_code == -4:
        print "Return: Path tolerance violated"
    elif result.error_code == -5:
        print "Return: Goal tolerance violated"
    
    


if __name__ == '__main__':
    rospy.init_node("test_joint_controller")
    
    ## test simple message interface
    #traj_pub()
    
    ## test action interface
    traj_action_client()

