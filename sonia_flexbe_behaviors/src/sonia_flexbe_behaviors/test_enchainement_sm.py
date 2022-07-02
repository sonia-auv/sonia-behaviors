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
from sonia_flexbe_behaviors.move_to_gate_trickshot_v2_sm import move_to_gate_trickshot_v2SM
from sonia_flexbe_behaviors.straight_path_sm import straight_pathSM
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.set_initial_position import set_initial_position
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 21 2022
@author: CS
'''
class test_enchainementSM(Behavior):
	'''
	Test
	'''


	def __init__(self):
		super(test_enchainementSM, self).__init__()
		self.name = 'test_enchainement'

		# parameters of this behavior
		self.add_parameter('simulation', False)

		# references to used behaviors
		self.add_behavior(coin_flipSM, 'coin_flip')
		self.add_behavior(move_to_gate_trickshot_v2SM, 'move_to_gate_trickshot_v2')
		self.add_behavior(straight_pathSM, 'straight_path')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:908 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:81 y:73
			OperatableStateMachine.add('start_simulation',
										set_initial_position(simulation=self.simulation),
										transitions={'continue': 'Planner mode'},
										autonomy={'continue': Autonomy.Off})

			# x:99 y:251
			OperatableStateMachine.add('coin_flip',
										self.use_behavior(coin_flipSM, 'coin_flip'),
										transitions={'finished': 'move_to_gate_trickshot_v2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:292 y:367
			OperatableStateMachine.add('move_to_gate_trickshot_v2',
										self.use_behavior(move_to_gate_trickshot_v2SM, 'move_to_gate_trickshot_v2'),
										transitions={'finished': 'straight_path', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:269 y:507
			OperatableStateMachine.add('straight_path',
										self.use_behavior(straight_pathSM, 'straight_path'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:63 y:166
			OperatableStateMachine.add('Planner mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'coin_flip', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
