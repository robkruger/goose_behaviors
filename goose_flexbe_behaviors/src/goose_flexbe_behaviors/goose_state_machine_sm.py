#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from goose_flexbe_states.detect_sheets_state import DetectSheetsState
from goose_flexbe_states.go_to_home_state import GoToHomeState
from goose_flexbe_states.move_to_sheet import MoveToSheet
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Mar 21 2025
@author: Rob Kruger, Tom Rietjens
'''
class GoosestatemachineSM(Behavior):
	'''
	The state machine determining Goose's behavior when picking up cloths
	'''


	def __init__(self):
		super(GoosestatemachineSM, self).__init__()
		self.name = 'Goose state machine'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		max_detect_attempts = 10
		camera_width = 640
		camera_horizontal_FOV = 58.4
		# x:690 y:307, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.distance = 0.0
		_state_machine.userdata.x_center = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:128 y:50
			OperatableStateMachine.add('Start message',
										LogState(text="State machine started", severity=Logger.REPORT_HINT),
										transitions={'done': 'Go To Home State'},
										autonomy={'done': Autonomy.Off})

			# x:127 y:161
			OperatableStateMachine.add('Go To Home State',
										GoToHomeState(),
										transitions={'arrived': 'Detect Sheets', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:609 y:113
			OperatableStateMachine.add('Move to sheet',
										MoveToSheet(camera_width=camera_width, camera_horiz_FOV=camera_horizontal_FOV),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:363 y:98
			OperatableStateMachine.add('Detect Sheets',
										DetectSheetsState(max_attempts=max_detect_attempts),
										transitions={'found': 'Move to sheet', 'too_close': 'failed', 'not_found': 'failed', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'too_close': Autonomy.Off, 'not_found': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
