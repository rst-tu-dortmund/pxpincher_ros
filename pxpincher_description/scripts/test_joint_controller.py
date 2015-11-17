#!/usr/bin/env python

# Author Christoph Roesmann

import rospy
from trajectory_msgs.msg import *


def traj_pub():
  rospy.init_node("test_joint_controller")
  p = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)


  msg = JointTrajectory()
  msg.joint_names = ["arm_elbow_flex_joint", "arm_shoulder_lift_joint", "arm_shoulder_pan_joint", "arm_wrist_flex_joint"]
  
  point = JointTrajectoryPoint()
  point.positions = [1,0.6,-0.3,-0.4]
  point.time_from_start.secs = 3
  
  
  msg.points.append(point)
  msg.header.stamp = rospy.Time.now()
  
  #p.publish(msg) # publish once
  
  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.5)



if __name__ == '__main__':
    traj_pub()

