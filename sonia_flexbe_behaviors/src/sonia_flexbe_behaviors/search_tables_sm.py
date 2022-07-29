#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.zigzag_sm import zigzagSM
from sonia_flexbe_behaviors.zigzag_tables_sm import zigzag_tablesSM
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 18 2022
@author: GS
'''
class search_tablesSM(Behavior):
	'''
	Search mouvement in a zigzag patern.
	'''


	def __init__(self):
		super(search_tablesSM, self).__init__()
		self.name = 'search_tables'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(zigzagSM, 'Container/zigzag')
		self.add_behavior(zigzag_tablesSM, 'Container_2/zigzag_tables')
		self.add_behavior(moveSM, 'move')

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

		# x:454 y:349, x:471 y:224, x:473 y:122, x:330 y:458, x:430 y:458, x:530 y:458
		_sm_container_2_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target', 'topic'], conditions=[
										('lost_target', [('zigzag_tables', 'finished')]),
										('failed', [('zigzag_tables', 'failed')]),
										('finished', [('find_target', 'continue')])
										])

		with _sm_container_2_0:
			# x:62 y:103
			OperatableStateMachine.add('zigzag_tables',
										self.use_behavior(zigzag_tablesSM, 'Container_2/zigzag_tables'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:71 y:288
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})


		# x:598 y:216, x:607 y:51, x:622 y:144, x:596 y:288, x:753 y:94, x:715 y:266
		_sm_container_1 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['target', 'topic'], conditions=[
										('finished', [('find_target', 'continue')]),
										('lost_target', [('zigzag', 'finished')]),
										('failed', [('zigzag', 'failed')])
										])

		with _sm_container_1:
			# x:104 y:69
			OperatableStateMachine.add('zigzag',
										self.use_behavior(zigzagSM, 'Container/zigzag'),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:244 y:254
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:75 y:177
			OperatableStateMachine.add('Container',
										_sm_container_1,
										transitions={'finished': 'stop', 'failed': 'failed', 'lost_target': 'move'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:432 y:17
			OperatableStateMachine.add('Container_2',
										_sm_container_2_0,
										transitions={'finished': 'stop', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:602 y:323
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'failed', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:175 y:53
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 90, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'Container_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:613 y:158
			OperatableStateMachine.add('stop',
										stop_move(timeout=15),
										transitions={'target_reached': 'finished', 'target_not_reached': 'is_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
