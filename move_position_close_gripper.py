#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_gripper_reactive_approach")
import rospy
from pr2_gripper_reactive_approach import controller_manager
from pr2_gripper_reactive_approach.controller_manager import ControllerManager
from object_manipulator.convert_functions import *
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion

if __name__ == '__main__':

  rospy.init_node('controller_manager', anonymous=True)
  points =[ 
    [.62, -.05, .65, 0.0, 0.0, 0.0, 1.0],
    [.62, -.05, .65, 0.0, 1.0, 0.0, 0.0]
  ]
  cm = ControllerManager('r')
  cm.switch_to_cartesian_mode()

  cm.command_cartesian(points[0])
  result = cm.wait_cartesian_really_done(points[0], .01, .1, rospy.Duration(30.0), rospy.Duration(15.0))
  raw_input("press enter to continue")

  cm.command_cartesian(points[1])
  result = cm.wait_cartesian_really_done(points[1], .01, .1, rospy.Duration(30.0), rospy.Duration(15.0))
 
