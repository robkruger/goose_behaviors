#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from goose_detection.msg import DetectSheetsAction, DetectSheetsGoal

class DetectSheetsState(EventState):
	'''
	State for using the camera to detect sheets.

	-- max_attempts	int		max attempts to find a sheet

	#> distance     float   the distance to the closest sheet found.
	#> x_center     float   the center of the bounding box.
    
	<= found 	            A sheet is found.
	<= too_close            A sheet was found but the robot is now too close.
	<= not_found            A sheet was not found.
	<= failed               A there was another failure.

	'''

	def __init__(self, max_attempts):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(DetectSheetsState, self).__init__(outcomes=['found', 'too_close', 'not_found', 'failed'], 
												output_keys=['distance', 'x_center'])
		
		# setup the detect_sheet client
		self._action_topic = "/sheet_detector/detect_sheets"
		self._client = ProxyActionClient({self._action_topic: DetectSheetsAction})

		
		# set base status
		self._max_attempts = max_attempts
		self._attempts = 0
		self._goal_sent = False
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		Logger.loginfo("Entered detect_sheets_state")
		if not self._goal_sent:
			goal = DetectSheetsGoal()
			self._client.send_goal(self._action_topic, goal)
			self._goal_sent = True
			Logger.loginfo(f"sent goal to {self._action_topic}")

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		if self._client.has_result(self._action_topic):
			result =  self._client.get_result(self._action_topic)
			userdata.distance = result.distance
			userdata.x_center = result.bbox_x_center
			return 'found'
		else:
			if self._attempts >= self._max_attempts:
				Logger.logwarn(f"DetectSheetsState: No sheets found within {self._max_attempts} attempts.")
				return 'not_found'
			self._attempts += 1
		# Keep waiting until a result is received.
		return None



	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.
		self._goal_sent = False


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.