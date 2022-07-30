#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.swipe_sm import swipeSM
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: CS
'''
class search_torpedoesSM(Behavior):
	'''
	swipe around yaw axis to look for torpedoes
	'''


	def __init__(self):
		super(search_torpedoesSM, self).__init__()
		self.name = 'search_torpedoes'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(swipeSM, 'Container/swipe')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:784 y:166, x:161 y:190, x:282 y:123, x:315 y:185
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'], input_keys=['target', 'topic'])
		_state_machine.userdata.target = ''
		_state_machine.userdata.topic = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:481 y:233, x:474 y:46, x:476 y:109, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target', 'topic'], conditions=[
										('finished', [('find_target', 'continue')]),
										('lost_target', [('swipe', 'finished')]),
										('failed', [('swipe', 'failed')])
										])

		with _sm_container_0:
			# x:182 y:60
			OperatableStateMachine.add('swipe',
										self.use_behavior(swipeSM, 'Container/swipe'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:156 y:222
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:99 y:56
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:427 y:202
			OperatableStateMachine.add('check_stop',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'stop', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:429 y:73
			OperatableStateMachine.add('stop',
										stop_move(timeout=15),
										transitions={'target_reached': 'finished', 'target_not_reached': 'check_stop', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
