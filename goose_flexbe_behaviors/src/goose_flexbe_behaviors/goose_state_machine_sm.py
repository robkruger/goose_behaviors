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
from flexbe_states.wait_state import WaitState
from goose_flexbe_states.RecoveryRotateState import RecoveryRotateState
from goose_flexbe_states.detect_sheets_state import DetectSheetsState
from goose_flexbe_states.go_to_home_state import GoToHomeState
from goose_flexbe_states.move_arm_state import MoveArmState
from goose_flexbe_states.move_manually import MoveForward
from goose_flexbe_states.move_to_sheet_state import MoveToSheet
from goose_flexbe_states.reverse_out_state import ReverseOutState
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
		distance_scaler = 0.7
		home_pos_x = -0.94
		home_pos_y = 2.44
		home_ori_z = 0.00
		home_ori_w = 1.00
		lidar_front_topic = "/lidar_front_distance"
		arm_home = [0.0, 0.0, 0.0, 0.0]
		conv_pos_x = -2.17
		conv_pos_y = 2.35
		# x:1809 y:827, x:32 y:849
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.distance = 0.0
		_state_machine.userdata.x_center = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:69
			OperatableStateMachine.add('Start message',
										LogState(text="State machine started", severity=Logger.REPORT_HINT),
										transitions={'done': 'Move arm home'},
										autonomy={'done': Autonomy.Off})

			# x:1004 y:188
			OperatableStateMachine.add('Detect confirm',
										DetectSheetsState(max_attempts=max_detect_attempts),
										transitions={'found': 'Move to sheet 2', 'too_close': 'failed', 'not_found': 'recover confirm detection', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'too_close': Autonomy.Off, 'not_found': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:292 y:70
			OperatableStateMachine.add('Go To Home State',
										GoToHomeState(home_px=home_pos_x, home_py=home_pos_y, home_oz=home_ori_z, home_ow=home_ori_w),
										transitions={'arrived': 'Wait camera stable', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1542 y:322
			OperatableStateMachine.add('Move arm down',
										MoveArmState(joint_positions=[0.0, -1.05, -1.40, -0.70], gripper_state=1),
										transitions={'done': 'wait gripper', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:143 y:106
			OperatableStateMachine.add('Move arm home',
										MoveArmState(joint_positions=arm_home, gripper_state=0),
										transitions={'done': 'Go To Home State', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1175 y:330
			OperatableStateMachine.add('Move arm up',
										MoveArmState(joint_positions=[0.0, 0.4, 0.3, 0.0], gripper_state=1),
										transitions={'done': 'Reverse from sheets', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1531 y:187
			OperatableStateMachine.add('Move manually to sheet',
										MoveForward(topic_name=lidar_front_topic, speed=0.15, stop_distance=0.25, outlier_magnitude=0.5),
										transitions={'continue': 'Move arm down', 'failed': 'failed', 'recovery': 'recover manual forward movement'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'recovery': Autonomy.Off})

			# x:630 y:177
			OperatableStateMachine.add('Move to sheet',
										MoveToSheet(camera_width=camera_width, camera_horiz_FOV=camera_horizontal_FOV, distance_scaler=distance_scaler),
										transitions={'arrived': 'wait camera stable 2', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:1168 y:188
			OperatableStateMachine.add('Move to sheet 2',
										MoveToSheet(camera_width=camera_width, camera_horiz_FOV=camera_horizontal_FOV, distance_scaler=distance_scaler),
										transitions={'arrived': 'wait lidar stable', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:983 y:322
			OperatableStateMachine.add('Reverse from sheets',
										ReverseOutState(speed=0.5),
										transitions={'arrived': 'go to conveyor belt', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:294 y:166
			OperatableStateMachine.add('Wait camera stable',
										WaitState(wait_time=5),
										transitions={'done': 'Detect Sheets'},
										autonomy={'done': Autonomy.Off})

			# x:586 y:323
			OperatableStateMachine.add('drop sheet',
										MoveArmState(joint_positions=[0.0, -1.00, -1.35, -0.65], gripper_state=0),
										transitions={'done': 'wait gripper 2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:795 y:319
			OperatableStateMachine.add('go to conveyor belt',
										GoToHomeState(home_px=conv_pos_x, home_py=conv_pos_y, home_oz=home_ori_w, home_ow=home_ori_z),
										transitions={'arrived': 'drop sheet', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:240 y:312
			OperatableStateMachine.add('move arm up 2',
										MoveArmState(joint_positions=arm_home, gripper_state=0),
										transitions={'done': 'wait arm up', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:825 y:41
			OperatableStateMachine.add('recover confirm detection',
										RecoveryRotateState(rnd_mew=0.0, rnd_std=1.0),
										transitions={'arrived': 'wait camera stable 2', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:1401 y:64
			OperatableStateMachine.add('recover manual forward movement',
										RecoveryRotateState(rnd_mew=0.0, rnd_std=1.0),
										transitions={'arrived': 'wait lidar stable', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:495 y:35
			OperatableStateMachine.add('recovery sheet detection',
										RecoveryRotateState(rnd_mew=0.5, rnd_std=1.0),
										transitions={'arrived': 'Wait camera stable', 'failed': 'Go To Home State'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:22 y:200
			OperatableStateMachine.add('reverse from conveyor',
										ReverseOutState(speed=0.5),
										transitions={'arrived': 'Move arm home', 'failed': 'finished'},
										autonomy={'arrived': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'distance': 'distance', 'x_center': 'x_center'})

			# x:104 y:302
			OperatableStateMachine.add('wait arm up',
										WaitState(wait_time=1),
										transitions={'done': 'reverse from conveyor'},
										autonomy={'done': Autonomy.Off})

			# x:815 y:184
			OperatableStateMachine.add('wait camera stable 2',
										WaitState(wait_time=5),
										transitions={'done': 'Detect confirm'},
										autonomy={'done': Autonomy.Off})

			# x:1360 y:335
			OperatableStateMachine.add('wait gripper',
										WaitState(wait_time=2),
										transitions={'done': 'Move arm up'},
										autonomy={'done': Autonomy.Off})

			# x:422 y:311
			OperatableStateMachine.add('wait gripper 2',
										WaitState(wait_time=2),
										transitions={'done': 'move arm up 2'},
										autonomy={'done': Autonomy.Off})

			# x:1343 y:186
			OperatableStateMachine.add('wait lidar stable',
										WaitState(wait_time=2),
										transitions={'done': 'Move manually to sheet'},
										autonomy={'done': Autonomy.Off})

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
