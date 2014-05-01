#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_gripper_reactive_approach")
import rospy
from pr2_gripper_reactive_approach import controller_manager
from pr2_gripper_reactive_approach.controller_manager import ControllerManager
from object_manipulator.convert_functions import *
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion


# This uses the controller manager from pr2_gripper_reactive_approach
# Find controller src at
# /opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_reactive_approach/src/pr2_gripper_reactive_approach/controller_manager.py


if __name__ == '__main__':

  rospy.init_node('controller_manager', anonymous=True)
  point =   [.65, 0.0, .65, 0.0, 0.0, 0.0, 1.0]
  
  cm = ControllerManager('r')
  cm.switch_to_cartesian_mode()

  cm.command_cartesian(point)
  result = cm.wait_cartesian_really_done(point, .01, .1, rospy.Duration(30.0), rospy.Duration(15.0))
  
  max_effort = 10
  raw_input("open gripper?")
  open_position= .1
  max_effort = 10
  cm.command_gripper(open_position, max_effort)

  raw_input("close?")
  close_position = 0.0
  cm.command_gripper(close_position, max_effort)
  
  

 
