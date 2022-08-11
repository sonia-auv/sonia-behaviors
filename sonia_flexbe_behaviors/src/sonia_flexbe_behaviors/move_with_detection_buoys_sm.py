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
class move_with_detection_buoysSM(Behavior):
	'''
	Move while trying to detect the buoys
	'''


	def __init__(self):
		super(move_with_detection_buoysSM, self).__init__()
		self.name = 'move_with_detection_buoys'

		# parameters of this behavior
		self.add_parameter('move_for_buoys', 15)
		self.add_parameter('buoys_filterchain', 'deep_compe_front')
		self.add_parameter('buoys_target', 'Badge')
		self.add_parameter('buoys_camera', 1)

		# references to used behaviors
		self.add_behavior(moveSM, 'Container/move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1101 y:73, x:130 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:606 y:181, x:625 y:39, x:230 y:458, x:330 y:458, x:430 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['topic', 'target'], conditions=[
										('finished', [('move', 'finished')]),
										('finished', [('find_buoys', 'continue')]),
										('failed', [('move', 'failed')])
										])

		with _sm_container_0:
			# x:138 y:67
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'Container/move',
											parameters={'positionX': self.move_for_buoys, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:156 y:196
			OperatableStateMachine.add('find_buoys',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:86 y:131
			OperatableStateMachine.add('start_deep',
										start_filter_chain(filterchain=self.buoys_filterchain, target=self.buoys_target, camera_no=self.buoys_camera),
										transitions={'continue': 'Container', 'failed': 'stop_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:543 y:303
			OperatableStateMachine.add('are_you_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_success', 'moving': 'stop', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:565 y:118
			OperatableStateMachine.add('stop',
										stop_move(timeout=30),
										transitions={'target_reached': 'stop_success', 'target_not_reached': 'are_you_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:46 y:278
			OperatableStateMachine.add('stop_fail',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:921 y:209
			OperatableStateMachine.add('stop_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:332 y:63
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'topic': 'topic', 'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
