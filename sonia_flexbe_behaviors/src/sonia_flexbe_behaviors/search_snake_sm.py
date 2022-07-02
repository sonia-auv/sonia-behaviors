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
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 14 2021
@author: FA
'''
class search_snakeSM(Behavior):
	'''
	Search in a snake pattern.
	'''


	def __init__(self):
		super(search_snakeSM, self).__init__()
		self.name = 'search_snake'

		# parameters of this behavior
		self.add_parameter('time_stop_search', 105)

		# references to used behaviors
		self.add_behavior(snake_mouvementSM, 'Snake searching/snake_mouvement')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:642 y:64, x:389 y:239, x:209 y:246
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'])
		_state_machine.userdata.target = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:349 y:161, x:437 y:28, x:414 y:120, x:322 y:249, x:598 y:122, x:641 y:62, x:543 y:252
		_sm_snake_searching_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'], conditions=[
										('lost_target', [('find target', 'failed')]),
										('finished', [('find target', 'continue')]),
										('lost_target', [('snake_mouvement', 'finished')]),
										('failed', [('snake_mouvement', 'failed')])
										])

		with _sm_snake_searching_0:
			# x:95 y:39
			OperatableStateMachine.add('snake_mouvement',
										self.use_behavior(snake_mouvementSM, 'Snake searching/snake_mouvement'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:119 y:201
			OperatableStateMachine.add('find target',
										find_vision_target(number_samples=10, timeout=self.time_stop_search),
										transitions={'continue': 'finished', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:90 y:76
			OperatableStateMachine.add('Snake searching',
										_sm_snake_searching_0,
										transitions={'finished': 'found target', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target'})

			# x:338 y:63
			OperatableStateMachine.add('found target',
										stop_move(timeout=3),
										transitions={'continue': 'finished', 'failed': 'failed', 'error': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
