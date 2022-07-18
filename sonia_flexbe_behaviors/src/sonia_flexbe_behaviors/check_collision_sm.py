#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.touch_buoy_sm import touch_buoySM
from sonia_navigation_states.has_collided import has_collided
from sonia_navigation_states.stop_move import stop_move
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 15 2022
@author: GS
'''
class check_collisionSM(Behavior):
	'''
	Move forward and check collision
	'''


	def __init__(self):
		super(check_collisionSM, self).__init__()
		self.name = 'check_collision'

		# parameters of this behavior
		self.add_parameter('threshold', 0.5)

		# references to used behaviors
		self.add_behavior(touch_buoySM, 'Container/touch_buoy')
		self.add_behavior(init_submarineSM, 'init_submarine')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:117 y:202, x:720 y:54
		_state_machine = OperatableStateMachine(outcomes=['failed', 'target_reached'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:191 y:451, x:779 y:39, x:330 y:458, x:430 y:458
		_sm_container_0 = ConcurrencyContainer(outcomes=['failed', 'target_reached'], conditions=[
										('failed', [('touch_buoy', 'failed'), ('has_collided', 'error')]),
										('target_reached', [('has_collided', 'target_reached')]),
										('failed', [('touch_buoy', 'finished')])
										])

		with _sm_container_0:
			# x:143 y:27
			OperatableStateMachine.add('touch_buoy',
										self.use_behavior(touch_buoySM, 'Container/touch_buoy'),
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:494 y:114
			OperatableStateMachine.add('has_collided',
										has_collided(timeout=30, threshold=self.threshold),
										transitions={'target_reached': 'target_reached', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'error': Autonomy.Off})



		with _state_machine:
			# x:29 y:333
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'Container', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:344 y:45
			OperatableStateMachine.add('stop',
										stop_move(timeout=30),
										transitions={'target_reached': 'target_reached', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:93 y:36
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'failed': 'failed', 'target_reached': 'stop'},
										autonomy={'failed': Autonomy.Inherit, 'target_reached': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
