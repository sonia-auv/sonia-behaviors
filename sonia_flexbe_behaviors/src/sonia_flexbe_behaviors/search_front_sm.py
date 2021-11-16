#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.snake_mouvement_sm import snake_mouvementSM
from sonia_flexbe_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 14 2021
@author: FA
'''
class search_frontSM(Behavior):
	'''
	Look for a vision target on the front camera
	'''


	def __init__(self):
		super(search_frontSM, self).__init__()
		self.name = 'search_front'

		# parameters of this behavior
		self.add_parameter('move_time', 15)

		# references to used behaviors
		self.add_behavior(snake_mouvementSM, 'Snake searching/snake_mouvement')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:418 y:40, x:449 y:123, x:390 y:216
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'])
		_state_machine.userdata.target = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:349 y:161, x:437 y:28, x:414 y:120, x:322 y:249, x:598 y:122, x:641 y:62, x:543 y:252
		_sm_snake_searching_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'], conditions=[
										('lost_target', [('snake_mouvement', 'finished')]),
										('failed', [('snake_mouvement', 'failed')]),
										('lost_target', [('find target', 'failed')]),
										('finished', [('find target', 'continue')])
										])

		with _sm_snake_searching_0:
			# x:126 y:63
			OperatableStateMachine.add('snake_mouvement',
										self.use_behavior(snake_mouvementSM, 'Snake searching/snake_mouvement',
											parameters={'timeout': self.move_time}),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:119 y:201
			OperatableStateMachine.add('find target',
										find_vision_target(number_samples=10, timeout=self.move_time*7),
										transitions={'continue': 'finished', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:111 y:54
			OperatableStateMachine.add('Snake searching',
										_sm_snake_searching_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
