#!/usr/bin/env python
#title echo.py
#author ariel anders

import rospy
import roslib; roslib.load_manifest("pr2_controllers_msgs")
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

def callback(data):
  #print "I heard %s" % data
  print data.actual.positions[0]

def listener():
  rospy.init_node('listener')
  rospy.Subscriber("r_arm_controller/state", JointTrajectoryControllerState, callback)
  rospy.spin()

listener()
