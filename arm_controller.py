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


    def moveArmToSide(self):
      print "moving arm to side"
      side_angles=[0]*7
      side_angles[0]=.7
      side_angles[1]=-.3
      self.GD.command_joint(side_angles)

    def moveArmToFront(self):
      print "moving arm to front"
      self.GD.command_joint([0]*7)

    def vertLiftUp(self, dist=0.025):
     #update current goal to current pose
     current_goal = self.return_current_pose_as_list()
     current_goal[2] += dist
     self.move_cartesian_step( current_goal, blocking = 1)

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

    def getArmAngles(self):
     print self.GD.get_current_arm_angles()
     return self.GD.get_current_arm_angles()
    
    def moveArmToAngle(self, angle):
      self.GD.command_joint(angle)
    
    def moveArmUpSide(self):
      angle = [1.4300966555069725, -0.33983147251202583, 1.5081292159236199, -1.4087675943089149,\
                -143.28328411625168, -1.6500563964512449, -28.27274062300144]
      self.GD.command_joint(angle)

    #return the current pos and rot of the wrist for the right arm 
    #as a 7-list (pos, quaternion rot)
    def return_current_pose_as_list(self):
        (pos, rot) = self.GD.return_cartesian_pose()
        return pos+rot 

    ##convert a relative vector in frame to a pose in the base_link frame
    #if start_pose is not specified, uses current pose of the wrist
    def return_rel_pose(self, vector, frame, start_pose = None):

        #if start_pose is not specified, use the current Cartesian pose of the wrist
        if start_pose == None:
            (start_trans, start_rot) = self.GD.return_cartesian_pose('base_link')
        else:
            start_pose.header.stamp = rospy.Time(0)
            (start_trans, start_rot) = pose_stamped_to_lists(self.GD.tf_listener, start_pose, 'base_link')

        #convert the vector in frame to base_link frame
        if frame != 'base_link':
            vector3_stamped = create_vector3_stamped(vector, frame)
            base_link_vector = vector3_stamped_to_list(self.GD.tf_listener, vector3_stamped, 'base_link')    
        else:
            base_link_vector = vector

        #add the desired vector to the position
        new_trans = [x+y for (x,y) in zip(start_trans, base_link_vector)]

        #create a new poseStamped
        pose_stamped = create_pose_stamped(new_trans+start_rot)

        return pose_stamped

    #move to a Cartesian pose goal
    def move_cartesian_step(self, pose, timeout = 1.0,
                            settling_time = 3.0, blocking = 0):
        if type(pose) == list:
            pose = create_pose_stamped(pose, 'base_link')
            self.GD.move_cartesian(pose, blocking = blocking, \
                       pos_thres = .0025, rot_thres = .05, \
                       timeout = rospy.Duration(timeout), \
                       settling_time = rospy.Duration(settling_time))



  def sampleFunctions(ctrl):
  # variables for open/close commands
    quit_ch = "q"
    up_ch ="u"
    down_ch = "d"
    side_ch = "s"
    front_ch = "f"
    small_step = .02
    home = ctrl.getArmAngles()
    text_in = "0"
    p1 = ctrl.getArmAngles()
    p2 = ctrl.getArmAngles()
    

    while text_in != quit_ch:
       print "Commands:\t(q)uit, teach new (H)ome position. \n", \
          "Move Arm:\t(s)ide 1 or (S)ide 2, (h)ome, or (f)ront \n",\
          "Move Gripper:\t(r)ight, (l)eft, (u)p, (d)own, (t)oward robot, (a)way from robot\n"
       text_in = raw_input()
       
       if text_in == up_ch:  
          print "moving up"
          ctrl.vertLiftUp()

       elif text_in == down_ch:
          print "moving down"
          ctrl.vertLiftDown()
       
       elif text_in == side_ch:
          ctrl.moveArmToSide()
       
       elif text_in == front_ch:
          ctrl.moveArmToFront()
       
       elif text_in == 'r':
          print "moving right"
          ctrl.moveRelCartesianStep([0,-1*small_step,0])
       
       elif text_in == 'l':
          print "moving left"
          ctrl.moveRelCartesianStep([0, small_step, 0])
       
       elif text_in == 'a':
          print "moving away from robot"
          ctrl.moveRelCartesianStep([small_step, 0, 0])
       
       elif text_in == 't':
          print "moving toward the robot"
          ctrl.moveRelCartesianStep([-1*small_step, 0, 0])
       
       elif text_in == 'H':
          print "new home position"
          home = ctrl.getArmAngles()
       
       elif text_in == 'h':
         print "moving to home position"
         ctrl.moveArmToAngle(home)
       elif text_in == "p1":
        print "learning p1"
        p1 = ctrl.getArmAngles()
       elif text_in == "p2":
         print "learning p2"
         p2 = ctrl.getArmAngles()
       elif text_in == "gp1":
         print "moving to p1"
         ctrl.moveArmToAngle(p1)
       elif text_in == "gp2":
         print "moving to p2"
         ctrl.moveArmToAngle(p2)
       elif text_in == 'S':
         print "moving to up side"
         ctrl.moveArmUpSide()
        


    print "exiting program"

  if __name__ == '__main__':
    rospy.init_node('arm_controller')
    print "Hello welecome to arm controller"
    ctrl  = arm_controller()
    print "welcome to Ari's gripper test program\n"
    # start gripper/open close keyboard interface control
    sampleFunctions(ctrl)


