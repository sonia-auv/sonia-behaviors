#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.coin_flip_sm import coin_flipSM
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.move_to_gate_no_trickshot_sm import move_to_gate_no_trickshotSM
from sonia_flexbe_behaviors.trickshot_yaw_sm import trickshotyawSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 21 2021
@author: FA
'''
class gate_trickshot_yaw_taskSM(Behavior):
	'''
	Init the sub, coin filp and gate with a yaw trickshot only.
	'''


	def __init__(self):
		super(gate_trickshot_yaw_taskSM, self).__init__()
		self.name = 'gate_trickshot_yaw_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(coin_flipSM, 'coin_flip')
		self.add_behavior(init_submarineSM, 'init_submarine')
		self.add_behavior(move_to_gate_no_trickshotSM, 'move_to_gate_no_trickshot')
		self.add_behavior(trickshotyawSM, 'trickshot yaw')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:796 y:384, x:377 y:410
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:165 y:95
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'coin_flip', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:727 y:127
			OperatableStateMachine.add('move_to_gate_no_trickshot',
										self.use_behavior(move_to_gate_no_trickshotSM, 'move_to_gate_no_trickshot'),
										transitions={'finished': 'trickshot yaw', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:612 y:311
			OperatableStateMachine.add('trickshot yaw',
										self.use_behavior(trickshotyawSM, 'trickshot yaw'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:446 y:90
			OperatableStateMachine.add('coin_flip',
										self.use_behavior(coin_flipSM, 'coin_flip'),
										transitions={'finished': 'move_to_gate_no_trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
