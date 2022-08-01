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
from sonia_flexbe_behaviors.search_zigzag_sm import search_zigzagSM
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.get_front_vision_target_rotation import get_front_vision_target_rotation
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 12 2022
@author: CS
'''
class vision_buoys_newSM(Behavior):
	'''
	Detect the buoy and align with a rotation
	'''


	def __init__(self):
		super(vision_buoys_newSM, self).__init__()
		self.name = 'vision_buoys_new'

		# parameters of this behavior
		self.add_parameter('vision_buoys_filterchain', 'deep_compe_front')
		self.add_parameter('camera_no', 1)
		self.add_parameter('vision_buoys_target', 'Badge')
		self.add_parameter('bounding_box_width', 200)
		self.add_parameter('bounding_box_height', 350)
		self.add_parameter('center_bounding_box_width', 100)
		self.add_parameter('center_bounding_box_height', 100)
		self.add_parameter('max_mouvement', 1.0)
		self.add_parameter('min_mouvement', 0.25)
		self.add_parameter('activate_vision_buoys', True)
		self.add_parameter('vision_buoys_distance_forward', 2.5)

		# references to used behaviors
		self.add_behavior(moveSM, 'move_2')
		self.add_behavior(moveSM, 'move_3')
		self.add_behavior(moveSM, 'move_4')
		self.add_behavior(moveSM, 'move_5')
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1316 y:50, x:914 y:716, x:456 y:420
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('activate_vision_buoys',
										activate_behavior(activate=True),
										transitions={'activate': 'filter_chain', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:801 y:583
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait_target_reached', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:43 y:367
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(filterchain=self.vision_buoys_filterchain, target=self.vision_buoys_target, camera_no=self.camera_no),
										transitions={'continue': 'init', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'front', 'target': 'target'})

			# x:773 y:108
			OperatableStateMachine.add('get_target',
										get_front_vision_target_rotation(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, max_rotation=25, min_rotation=5, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_filter_success', 'align': 'move', 'move': 'move', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'front', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:225 y:72
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:716 y:389
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1109 y:410
			OperatableStateMachine.add('move_2',
										self.use_behavior(moveSM, 'move_2',
											parameters={'positionX': self.vision_buoys_distance_forward, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'move_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1133 y:538
			OperatableStateMachine.add('move_3',
										self.use_behavior(moveSM, 'move_3',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 1, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 4, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'move_4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1136 y:646
			OperatableStateMachine.add('move_4',
										self.use_behavior(moveSM, 'move_4',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 15, 'frame': 2, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'move_5', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1218 y:755
			OperatableStateMachine.add('move_5',
										self.use_behavior(moveSM, 'move_5',
											parameters={'positionX': 1, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:483 y:157
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:524 y:586
			OperatableStateMachine.add('stop_filter_fail',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:290 y:303
			OperatableStateMachine.add('stop_filter_lost',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:1157 y:149
			OperatableStateMachine.add('stop_filter_success',
										stop_filter_chain(),
										transitions={'continue': 'move_2', 'failed': 'move_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:1040 y:269
			OperatableStateMachine.add('wait_target_reached',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'check_moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
