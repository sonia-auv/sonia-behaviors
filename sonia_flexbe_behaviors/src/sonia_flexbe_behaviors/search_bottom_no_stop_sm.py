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
Created on Mon Nov 15 2021
@author: FA
'''
class search_bottom_no_stopSM(Behavior):
	'''
	Looking for a vision target on bottom camera. Waiting for the actual move to complete.
	'''


	def __init__(self):
		super(search_bottom_no_stopSM, self).__init__()
		self.name = 'search_bottom_no_stop'

		# parameters of this behavior
		self.add_parameter('time_stop_search', 105)

		# references to used behaviors
		self.add_behavior(snake_mouvementSM, 'move with search/snake_mouvement')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:391 y:39, x:446 y:218, x:179 y:231
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'])
		_state_machine.userdata.target = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:374 y:297, x:559 y:126, x:401 y:134, x:469 y:41, x:608 y:67, x:595 y:178, x:397 y:202
		_sm_move_with_search_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target'], conditions=[
										('finished', [('find_target', 'continue')]),
										('lost_target', [('find_target', 'failed')]),
										('failed', [('snake_mouvement', 'failed')]),
										('lost_target', [('snake_mouvement', 'finished')])
										])

		with _sm_move_with_search_0:
			# x:99 y:41
			OperatableStateMachine.add('snake_mouvement',
										self.use_behavior(snake_mouvementSM, 'move with search/snake_mouvement'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:78 y:163
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10, timeout=self.time_stop_search),
										transitions={'continue': 'finished', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:122 y:66
			OperatableStateMachine.add('move with search',
										_sm_move_with_search_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
