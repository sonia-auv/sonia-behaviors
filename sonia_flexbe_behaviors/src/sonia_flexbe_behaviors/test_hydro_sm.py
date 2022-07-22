#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.hydro import hydro
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 21 2022
@author: GS
'''
class test_hydroSM(Behavior):
	'''
	hydro
	'''


	def __init__(self):
		super(test_hydroSM, self).__init__()
		self.name = 'test_hydro'

		# parameters of this behavior
		self.add_parameter('frequency', '25000')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:890 y:50, x:629 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:74 y:61
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'hydro'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:411 y:48
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:599 y:45
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:287 y:126
			OperatableStateMachine.add('hydro',
										hydro(frequency=self.frequency, speed=0, precision=0, long_rotation=False, timeout=10),
										transitions={'continue': 'planner', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
