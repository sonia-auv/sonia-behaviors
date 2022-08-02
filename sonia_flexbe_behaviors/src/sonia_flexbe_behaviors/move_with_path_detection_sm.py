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
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.stop_move import stop_move
from sonia_vision_states.find_vision_target import find_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 02 2022
@author: CS
'''
class move_with_path_detectionSM(Behavior):
	'''
	Move until we detect the path
	'''


	def __init__(self):
		super(move_with_path_detectionSM, self).__init__()
		self.name = 'move_with_path_detection'

		# parameters of this behavior
		self.add_parameter('path_filterchain', 'simple_path_straight')
		self.add_parameter('path_target', 'pipe straight')
		self.add_parameter('path_camera', 2)

		# references to used behaviors
		self.add_behavior(moveSM, 'Container/move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1137 y:274, x:130 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:546 y:110, x:534 y:43, x:230 y:458, x:330 y:458
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['topic', 'target'], conditions=[
										('finished', [('move', 'finished'), ('find_path', 'continue')]),
										('failed', [('move', 'failed')])
										])

		with _sm_container_0:
			# x:159 y:106
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'Container/move',
											parameters={'positionX': 5, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:164 y:243
			OperatableStateMachine.add('find_path',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:55 y:165
			OperatableStateMachine.add('start_path',
										start_filter_chain(filterchain=self.path_filterchain, target=self.path_target, camera_no=self.path_camera),
										transitions={'continue': 'Container', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:602 y:382
			OperatableStateMachine.add('are_you_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_success', 'moving': 'stop', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:580 y:203
			OperatableStateMachine.add('stop',
										stop_move(timeout=30),
										transitions={'target_reached': 'stop_success', 'target_not_reached': 'are_you_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:228 y:268
			OperatableStateMachine.add('stop_failed',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:891 y:226
			OperatableStateMachine.add('stop_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:259 y:149
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop', 'failed': 'stop_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'topic': 'topic', 'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
