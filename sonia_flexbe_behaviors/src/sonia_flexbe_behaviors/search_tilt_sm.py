#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_tiltdown_turn_tiltup_sm import move_tiltdown_turn_tiltupSM
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 30/08/2022
@author: GS
'''
class search_tiltSM(Behavior):
	'''
	Search mouvement in a tilt patern.
	'''


	def __init__(self):
		super(search_tiltSM, self).__init__()
		self.name = 'search_tilt'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(move_tiltdown_turn_tiltupSM, 'Container/move_tiltdown_turn_tiltup')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:892 y:251, x:370 y:336, x:479 y:203, x:450 y:260
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'], input_keys=['target', 'topic'])
		_state_machine.userdata.target = ''
		_state_machine.userdata.topic = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:598 y:216, x:607 y:51, x:622 y:144, x:596 y:288, x:753 y:94, x:715 y:266
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target', 'topic'], conditions=[
										('finished', [('find_target', 'continue')]),
										('lost_target', [('move_tiltdown_turn_tiltup', 'finished')]),
										('failed', [('move_tiltdown_turn_tiltup', 'failed')])
										])

		with _sm_container_0:
			# x:30 y:40
			OperatableStateMachine.add('move_tiltdown_turn_tiltup',
										self.use_behavior(move_tiltdown_turn_tiltupSM, 'Container/move_tiltdown_turn_tiltup'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:244 y:254
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:190 y:96
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:602 y:323
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'failed', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:613 y:158
			OperatableStateMachine.add('stop',
										stop_move(timeout=15),
										transitions={'target_reached': 'finished', 'target_not_reached': 'is_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
