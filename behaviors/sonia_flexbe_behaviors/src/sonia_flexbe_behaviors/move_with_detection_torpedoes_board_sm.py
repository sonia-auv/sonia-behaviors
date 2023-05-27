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
@author: GS
'''
class move_with_detection_torpedoes_boardSM(Behavior):
	'''
	move_with_detection_torpedoes_board
	'''


	def __init__(self):
		super(move_with_detection_torpedoes_boardSM, self).__init__()
		self.name = 'move_with_detection_torpedoes_board'

		# parameters of this behavior
		self.add_parameter('move_for_tb', 15.0)
		self.add_parameter('move_with_detection_torpedoes_board_filterchain', 'deep_compe_front')
		self.add_parameter('move_with_detection_torpedoes_board_target', 'G-Man')
		self.add_parameter('turn_for_tb', 0)
		self.add_parameter('camera', 1)

		# references to used behaviors
		self.add_behavior(moveSM, 'Container/move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:968 y:87, x:460 y:393, x:112 y:374
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:453 y:289, x:460 y:122, x:230 y:458, x:330 y:458, x:469 y:52, x:530 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'lost_target'], input_keys=['topic', 'target'], conditions=[
										('lost_target', [('move', 'finished')]),
										('finished', [('find_torpedoes_board', 'continue')]),
										('failed', [('move', 'failed')])
										])

		with _sm_container_0:
			# x:84 y:50
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'Container/move',
											parameters={'positionX': self.move_for_tb, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': self.turn_for_tb, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'lost_target', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:63 y:259
			OperatableStateMachine.add('find_torpedoes_board',
										find_vision_target(number_samples=10),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})



		with _state_machine:
			# x:130 y:75
			OperatableStateMachine.add('start_AI',
										start_filter_chain(filterchain=self.move_with_detection_torpedoes_board_filterchain, target=self.move_with_detection_torpedoes_board_target, camera_no=self.camera),
										transitions={'continue': 'Container', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:822 y:382
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_filter', 'moving': 'stop', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:686 y:237
			OperatableStateMachine.add('stop',
										stop_move(timeout=30),
										transitions={'target_reached': 'stop_filter', 'target_not_reached': 'is_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:720 y:78
			OperatableStateMachine.add('stop_filter',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:444 y:203
			OperatableStateMachine.add('stop_filter2',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:197 y:428
			OperatableStateMachine.add('stop_filter3',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:442 y:81
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stop', 'failed': 'stop_filter2', 'lost_target': 'stop_filter3'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'topic': 'topic', 'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
