#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.square_mouvement_sm import square_mouvementSM
from sonia_flexbe_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class search_bottomSM(Behavior):
	'''
	Looking for a vision target on bottom camera
	'''


	def __init__(self):
		super(search_bottomSM, self).__init__()
		self.name = 'search_bottom'

		# parameters of this behavior
		self.add_parameter('search_timeout', 0)

		# references to used behaviors
		self.add_behavior(square_mouvementSM, 'move square/square_mouvement')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:168 y:248, x:382 y:82
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target'])
		_state_machine.userdata.target = '/proc_image_processing/simple_pipe45_result'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:374 y:297, x:378 y:208, x:383 y:59, x:386 y:152, x:430 y:365, x:530 y:365
		_sm_move_square_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['target'], conditions=[
										('finished', [('find_target', 'continue')]),
										('failed', [('find_target', 'failed')]),
										('finished', [('square_mouvement', 'finished')]),
										('failed', [('square_mouvement', 'failed')])
										])

		with _sm_move_square_0:
			# x:82 y:50
			OperatableStateMachine.add('square_mouvement',
										self.use_behavior(square_mouvementSM, 'move square/square_mouvement'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:84 y:179
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10, timeout=self.search_timeout),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:122 y:66
			OperatableStateMachine.add('move square',
										_sm_move_square_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
