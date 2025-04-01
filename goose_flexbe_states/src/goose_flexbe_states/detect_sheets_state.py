#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from goose_detection.srv import DetectSheets, DetectSheetsRequest

class DetectSheetsState(EventState):
	'''
	State for using the camera to detect sheets.

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
		self._service = "/sheet_detector/detect_sheets"
		# rospy.wait_for_service(self._service)
		self._caller = ProxyServiceCaller({self._service: DetectSheets})
		
		# set base status
		self._max_attempts = max_attempts
		self._attempts = 0
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		Logger.loginfo("Entered detect_sheets_state")
	

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		if self._attempts < self._max_attempts:
			try:
				req = DetectSheetsRequest()
				distance, x_center = self._caller.call(self._service, req)
				if distance == x_center == 0.0:
					Logger.loginfo("not found")
					return 'not_found'
				else:
					Logger.loginfo(f"{distance}, {x_center}")
					return 'found'
			

			except Exception as e:
				Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
				return 'failed'



	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.