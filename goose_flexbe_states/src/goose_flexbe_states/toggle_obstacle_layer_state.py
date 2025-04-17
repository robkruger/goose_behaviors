#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from dynamic_reconfigure.client import Client
import time


class ToggleObstacleLayerState(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- enabled 	bool 		True if the costmap should be enabled, False if it should be disabled.

    <= continue 			Moved forward
    <= reconfigure_failed 	Costmap failed to be disables

    '''

    def __init__(self, enabled):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ToggleObstacleLayerState, self).__init__(outcomes = ['continue', 'reconfigure_failed'])

        self.client = Client("/move_base/local_costmap/obstacles_layer", timeout=5)

        # Set the 'enabled' parameter to enabled
        self.enabled = enabled
        self.client.update_configuration({'enabled': enabled})


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self.client.get_configuration()['enabled'] == self.enabled:
            # The costmap is disabled, so we can move forward
            return 'continue'
        else:
            # The costmap is still enabled, so we cannot move forward
            return 'costmap_failed'
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # The following code is just for illustrating how the behavior logger works.
        # Text logged by the behavior logger is sent to the operator and displayed in the GUI.

        pass


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        # In this example, we use this event to set the correct start time.
        pass


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.
        
