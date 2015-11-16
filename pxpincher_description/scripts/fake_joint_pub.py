#!/usr/bin/env python

# Original code https://github.com/corot/turtlebot_arm.git
# Modified by Christoph Roesmann

import rospy
from sensor_msgs.msg import JointState


FACTOR_GRIPPER_TRANSL = 0.01
OFFSET_GRIPPER_TRANSL = 0.029

def callback(data):
  global gripper_value
  try:
    idx = data.name.index("gripper_joint")
  except ValueError:
    return
  gripper_value = data.position[idx]


def joint_pub():
  rospy.init_node("fake_joint_pub")
  p = rospy.Publisher('joint_states', JointState, queue_size=5)
  rospy.Subscriber("joint_states", JointState, callback) # we need the other joint_state msg to get information the gripper_joint

  # store gripper_joint value
  global gripper_value

  msg = JointState()
  msg.name = ["gripper_link_joint","gripper2_joint","gripper_joint_prismatic1"]
  #msg.position = [0.0 for name in msg.name]
  msg.position = [0.0, -FACTOR_GRIPPER_TRANSL*gripper_value-OFFSET_GRIPPER_TRANSL, FACTOR_GRIPPER_TRANSL*gripper_value+OFFSET_GRIPPER_TRANSL]
  msg.velocity = [0.0 for name in msg.name]

  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.position = [0.0, -0.018*gripper_value-0.0205, 0.018*gripper_value+0.0205]
    #rospy.loginfo("gripper: %f",gripper_value)
    p.publish(msg)
    rospy.sleep(0.1)



if __name__ == '__main__':
    gripper_value = 0
    joint_pub()

