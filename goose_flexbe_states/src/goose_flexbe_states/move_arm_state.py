#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

import rospy
import sys
import moveit_commander
from math import pi


class MoveArmState(EventState):
    '''
    State that calls the arm service to move to a given position.

    -- joint_positions 	float64[] 	Four joint positions that the arm should move to.

    <= arrived 			    Arm is at position.
    <= failed 				Something went wrong.

    '''

    def __init__(self, joint_positions):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MoveArmState, self).__init__(outcomes = ['arrived', 'failed'])
        
        moveit_commander.roscpp_initialize(sys.argv)

        if rospy.has_param('/arm/robot_description'):
            Logger.logwarn("Parameter '/arm/robot_description' found!")
        else:
            Logger.logwarn("Parameter '/arm/robot_description' not found!")
            
        self.arm = moveit_commander.RobotCommander(robot_description="/arm/robot_description")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        
        self.move_group.set_max_velocity_scaling_factor(1)
        

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self.move_group.get_state() == moveit_commander.MoveItErrorCodes.SUCCESS:
            self.move_group.stop()
            return 'arrived'
        elif self.move_group.get_state() == moveit_commander.MoveItErrorCodes.MOTION_PLAN_INVALIDATED:
            Logger.logwarn("Motion plan invalidated during execution.")
            return 'failed'
        else:
            Logger.loginfo("Movement is still in progress.")
            
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self.move_group.go(userdata.joint_positions, wait=False)


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        self.move_group.stop()


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        self.move_group.stop()
        
