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
from goose_flexbe_states.move_arm_state import MoveArmState
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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:128 y:50
			OperatableStateMachine.add('Start message',
										LogState(text="State machine started", severity=Logger.REPORT_HINT),
										transitions={'done': 'MoveArm'},
										autonomy={'done': Autonomy.Off})

			# x:293 y:46
			OperatableStateMachine.add('MoveArm',
										MoveArmState(joint_positions=[0, 0, 0, 0]),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
