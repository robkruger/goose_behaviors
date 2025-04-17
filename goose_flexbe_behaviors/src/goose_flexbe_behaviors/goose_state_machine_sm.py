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
from goose_flexbe_states.RecoveryRotateState import RecoveryRotateState
from goose_flexbe_states.detect_sheets_state import DetectSheetsState
from goose_flexbe_states.go_to_home_state import GoToHomeState
from goose_flexbe_states.move_to_sheet_state import MoveToSheet
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
		max_detect_attempts = 100
		camera_width = 640
		camera_horizontal_FOV = 58.4
		distance_scaler = 0.8
		home_pos_x = -0.94
		home_pos_y = 2.44
		home_ori_z = 0.00
		home_ori_w = 1.00
		# x:731 y:572, x:193 y:628
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

			# x:292 y:70
			OperatableStateMachine.add('Go To Home State',
										GoToHomeState(home_px=home_pos_x, home_py=home_pos_y, home_oz=home_ori_z, home_ow=home_ori_w),
										transitions={'arrived': 'Detect Sheets', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:656 y:256
			OperatableStateMachine.add('Move to sheet',
										MoveToSheet(camera_width=camera_width, camera_horiz_FOV=camera_horizontal_FOV, distance_scaler=distance_scaler),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:495 y:35
			OperatableStateMachine.add('recovery sheet detection',
										RecoveryRotateState(rnd_mew=0.0, rnd_std=1.0),
										transitions={'arrived': 'Detect Sheets', 'failed': 'Go To Home State'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:453 y:177
			OperatableStateMachine.add('Detect Sheets',
										DetectSheetsState(max_attempts=max_detect_attempts),
										transitions={'found': 'Move to sheet', 'too_close': 'failed', 'not_found': 'recovery sheet detection', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'too_close': Autonomy.Off, 'not_found': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
