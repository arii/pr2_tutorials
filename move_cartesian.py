#!/usr/bin/python
# Title: arm_controller.py 
# Author: Ariel Anders
# Date: August 8, 2012
# 
# Description:
# This program offers higher level functions for controlling arm movement based on
# the controllerManager program from the gripper_reactive_approach package.
# run this program with this launch files:
# roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch 

import roslib
roslib.load_manifest("pr2_gripper_reactive_approach")
import rospy
from pr2_gripper_reactive_approach import controller_manager
from object_manipulator.convert_functions import *
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion

class arm_controller():

  def __init__(self):
    self.GD = controller_manager.ControllerManager('l', using_slip_controller=False, using_slip_detection = False)

  def wait_for_action_server(self, client, name):
      while not rospy.is_shutdown():  
          rospy.loginfo("Waiting for %s to be there"%name)
          if client.wait_for_server(rospy.Duration(1.5)):
              break
      rospy.loginfo("%s found"%name)  

  def vertLiftDown(self, dist=0.025):
   #update current goal to current pose
   current_goal = self.return_current_pose_as_list()
   current_goal[2] -= dist
   self.move_cartesian_step( current_goal, blocking = 1)
   
  def moveRelCartesianStep(self, vector):
    current_goal = self.return_current_pose_as_list()
    current_goal[0] += vector[0]
    current_goal[1] += vector[1]
    current_goal[2] += vector[2]
    self.move_cartesian_step(current_goal, blocking=1)

  #return the current pos and rot of the wrist for the right arm def main():
  #as a 7-list (pos, quaternion rot)
  def return_current_pose_as_list(self):
      (pos, rot) = self.GD.return_cartesian_pose()
      return pos+rot 

  #move to a Cartesian pose goal
  def move_cartesian_step(self, pose, timeout = 1.0,
                          settling_time = 3.0, blocking = 0):
      if type(pose) == list:
          pose = create_pose_stamped(pose, 'base_link')
          self.GD.move_cartesian(pose, blocking = blocking, \
                     pos_thres = .0025, rot_thres = .05, \
                     timeout = rospy.Duration(timeout), \
                     settling_time = rospy.Duration(settling_time))



if __name__=="__main__":
  rospy.init_node('arm_controller')
  print "Hello welcome to arm controller"
  ctrl  = arm_controller()
  print "current pose is %s" % ctrl.return_current_pose_as_list()
  
  raw_input("move to the left?")
  ctrl.moveRelCartesianStep([0,1,0])
  raw_input("quit?")
