#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time
import numpy as np


class MoveForward(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- topic_name string 	Topic name to subscribe to for distance data.
    -- speed float 		Speed of the robot.
    -- stop_distance float 	Distance at which the robot should stop.
    -- outlier_magnitude float 	Outliers with (new_distance - old_distance) < outlier_magnitude are ignored.
    
    <= continue 			Moved forward
    <= failed 	            Costmap failed to be disables
    <= recovery             Too many outliers detected, stop the robot and go to recovery state.

    '''

    def __init__(self, topic_name, speed, stop_distance, outlier_magnitude):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MoveForward, self).__init__(outcomes = ['continue', 'failed', 'recovery'])

        self.distance_sub = rospy.Subscriber(f"{topic_name}", Float32, self.distance_callback)
        self.movement_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=2)

        self.distance = np.inf
        self.speed = speed
        self.stop_distance = stop_distance

        self.outliers = 0
        self.outlier_magnitude = outlier_magnitude

        self.outlier_panic = False


    def distance_callback(self, msg):
        Logger.loginfo(f"Old distance: {self.distance}, New distance: {msg.data}")
        if msg.data < self.distance:
            self.outliers = 0
            self.distance = msg.data  
        else:
            self.outliers += 1
            if self.outliers > 25 or msg.data - self.distance > self.outlier_magnitude:
                self.outlier_panic = True


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self.outlier_panic:
            Logger.logwarn(f"Outliers detected: {self.outliers}, stopping the robot.")
            return 'recovery'

        if self.distance > self.stop_distance:
            movement = Twist()
            movement.linear.x = self.speed
            movement.angular.z = 0.0
            self.movement_pub.publish(movement)
        else:
            Logger.loginfo(f"Distance is less than {self.stop_distance}, stopping the robot.")
            return 'continue'
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # The following code is just for illustrating how the behavior logger works.
        # Text logged by the behavior logger is sent to the operator and displayed in the GUI.

        pass


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # Unsubscribe from the topic to clean up resources
        self.distance_sub.unregister()


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        # In this example, we use this event to set the correct start time.
        pass


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        self.distance_sub.unregister()        
