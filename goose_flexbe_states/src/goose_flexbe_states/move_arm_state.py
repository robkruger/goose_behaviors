#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

import rospy
import sys
import actionlib
from math import pi
from goose_arm.msg import MoveArmAction, MoveArmGoal
from mirte_msgs.srv import SetServoAngle


class MoveArmState(EventState):
    '''
    State that calls the arm service to move to a given position.

    -- joint_positions 	float64[] 	Four joint positions that the arm should move to.
    -- gripper_state    int         0: open, 1: close, 2: keep current state

    <= done 			    Arm is at position.
    <= failed 				Something went wrong.

    '''

    def feedback_cb(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")

    def result_cb(self, state, result):
        self.result = result

    def __init__(self, joint_positions, gripper_state):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MoveArmState, self).__init__(outcomes = ['done', 'failed'])
        
        self.client = actionlib.SimpleActionClient("/arm/move_arm_action", MoveArmAction)
        self.client.wait_for_server()

        # Subscribe to a ROS service
        rospy.wait_for_service('/mirte/set_servoGripper_servo_angle')
        self.move_arm_service = rospy.ServiceProxy('/mirte/set_servoGripper_servo_angle', SetServoAngle)
        self.gripper_state = gripper_state

        self.goal = MoveArmGoal()
        self.goal.joint_positions = joint_positions

        self.result = None
        

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        state = self.client.get_state()
        if state == 3:
            if self.result.success == True:
                self.move_gripper()
                return 'arrived'
            else: 
                return 'failed'

        rospy.sleep(0.2)

    
    def move_gripper(self):
        # Call the service to move the gripper
        try:
            if self.gripper_state == 0:
                self.move_arm_service(0.76)
            elif self.gripper_state == 1:
                self.move_arm_service(-0.64)
            elif self.gripper_state == 2:
                pass
            else:
                rospy.logerr("Invalid gripper state")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self.client.send_goal(self.goal, feedback_cb=self.feedback_cb, done_cb=self.result_cb)


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # self.client.cancel_goal()

        pass

        

