#!/usr/bin/env python3

# Created by Rodrigo Gonzalez

# This node launches an action server with the same name as the one used in Robotnik's RB-1 to lift its elevator platform. 
# It is used in simulation in order to mimic the real robot operation. 
# The action server calls the service servers elevator fake pickup gazebo, provided by Robotnik, to attach two models together and simulate platform lift

import rospy
import actionlib
import robotnik_msgs.msg

# Import Pick service message from fake elevator server
from robotnik_base_hw_sim.srv import Pick, Place
from robotnik_base_hw_sim.msg import PickState
from geometry_msgs.msg import Pose


class FakeElevatorAction(object):
    # create messages that are used to publish feedback/result
    # _feedback = robotnik_msgs.msg.SetElevatorFakeFeedback()
    _result = None

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, robotnik_msgs.msg.SetElevatorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        # Create service client to be used in action server callback
        self._sc = rospy.wait_for_service('/elevator_fake_pickup_gazebo/pick')
        self._sc_place = rospy.wait_for_service('/elevator_fake_pickup_gazebo/place')

      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = None
        # rospy.loginfo("+++++++++++++++++"+str(goal.action.action)+"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, calling /elevator_fake_pickup_gazebo service server')
        # start executing the action
        # SERVICE CALL HERE
        if goal.action.action == 1:
            try: 
                # define the service request
                object_model = 'rb2_simple_cart'
                object_link = 'link_0'
                robot_model = 'robot'
                robot_link = 'robot_base_footprint'
                pose = Pose()
                pose.position.z = 0.2
                pose.orientation.w = 1.0
                self.lift_fake_elevator = rospy.ServiceProxy('/elevator_fake_pickup_gazebo/pick', Pick)
                self.resp1 = self.lift_fake_elevator(object_model, object_link, robot_model, robot_link, pose)
                success = True
            except rospy.ServiceException as e:
                print("Fake elevator lift service call failed: %s" %e)

        elif goal.action.action == -1:
            try:
                # define service request
                object_model = 'rb2_simple_cart'
                robot_model = 'robot'

                self.lower_fake_elevator = rospy.ServiceProxy('/elevator_fake_pickup_gazebo/place', Place)
                self.resp2 = self.lower_fake_elevator(object_model, robot_model)
                success = True
            except rospy.ServiceException as e:
                print("Fake elevator lower service call failed: %s" %e)

        else:
            success = False
            print("Action goal not accepted")
          
        if success:
            self._result = True
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('set_elevator')
    server = FakeElevatorAction(rospy.get_name())
    rospy.spin()