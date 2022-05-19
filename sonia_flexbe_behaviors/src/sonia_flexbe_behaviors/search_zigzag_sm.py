#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.test_zigzag_sm import test_zigzagSM
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 18 2022
@author: KY
'''
class search_zigzagSM(Behavior):
	'''
	Search mouvement in a zigzag patern.
	'''


	def __init__(self):
		super(search_zigzagSM, self).__init__()
		self.name = 'search_zigzag'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(test_zigzagSM, 'Container/test_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:966 y:299, x:614 y:439, x:407 y:424
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'])
		_state_machine.userdata.target = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:598 y:216, x:607 y:51, x:622 y:144, x:596 y:288, x:753 y:94, x:715 y:266, x:736 y:185
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'], conditions=[
										('lost_target', [('test_zigzag', 'finished')]),
										('failed', [('test_zigzag', 'failed')]),
										('finished', [('find_target', 'continue')]),
										('lost_target', [('find_target', 'failed')])
										])

		with _sm_container_0:
			# x:286 y:59
			OperatableStateMachine.add('test_zigzag',
										self.use_behavior(test_zigzagSM, 'Container/test_zigzag'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:277 y:203
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10, timeout=105),
										transitions={'continue': 'finished', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:136 y:150
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop_movement', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target'})

			# x:655 y:203
			OperatableStateMachine.add('stop_movement',
										stop_move(timeout=3),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
