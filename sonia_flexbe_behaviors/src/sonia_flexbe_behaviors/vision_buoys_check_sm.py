#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_zigzag_sm import search_zigzagSM
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.get_vision_target import get_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 21 2022
@author: CS
'''
class vision_buoys_checkSM(Behavior):
	'''
	test check
	'''


	def __init__(self):
		super(vision_buoys_checkSM, self).__init__()
		self.name = 'vision_buoys_check'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:883 y:30, x:924 y:495
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('start_deep',
										start_filter_chain(filterchain='deep_compe_front', target='Badge', camera_no=3, param_cmd=1),
										transitions={'continue': 'start_sift', 'failed': 'stop_deep'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no', 'target': 'deep_target'})

			# x:611 y:268
			OperatableStateMachine.add('check',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'chaud', 'moving': 'wait', 'error': 'stop_deep'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:44 y:381
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'chaud'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:327 y:331
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'stop_deep'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:192 y:261
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'chaud', 'failed': 'stop_deep', 'lost_target': 'stop_deep', 'controller_error': 'stop_deep'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'deep_target', 'filterchain': 'deep_filterchain'})

			# x:30 y:132
			OperatableStateMachine.add('start_sift',
										start_filter_chain(filterchain='sift_front', target='makeGrade_badge', camera_no=3, param_cmd=1),
										transitions={'continue': 'start_simple', 'failed': 'stop_deep'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'sift_filterchain', 'camera_no': 'camera_no', 'target': 'sift_target'})

			# x:30 y:224
			OperatableStateMachine.add('start_simple',
										start_filter_chain(filterchain='simulation_badge', target='badge', camera_no=3, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'stop_deep'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'simple_filterchain', 'camera_no': 'camera_no', 'target': 'simple_target'})

			# x:691 y:50
			OperatableStateMachine.add('stop_deep',
										start_filter_chain(filterchain='deep_compe_front', target='Badge', camera_no=3, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no', 'target': 'deep_target'})

			# x:697 y:440
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'chaud', 'target_not_reached': 'check', 'error': 'stop_deep'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:261 y:450
			OperatableStateMachine.add('chaud',
										get_vision_target(center_bounding_box_pixel_height=100, center_bounding_box_pixel_width=100, deep_bounding_box_pixel_height=150, deep_bounding_box_pixel_width=350, sift_bounding_box_pixel_height=150, sift_bounding_box_pixel_width=150, simple_bounding_box_pixel_height=150, simple_bounding_box_pixel_width=150, image_height=400, image_width=600, deep_number_of_average=10, sift_number_of_average=10, simple_number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_deep', 'align': 'move', 'move': 'move', 'failed': 'failed', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'deep_filterchain': 'deep_filterchain', 'sift_filterchain': 'sift_filterchain', 'simple_filterchain': 'simple_filterchain', 'camera_no': 'camera_no', 'deep_target': 'deep_target', 'sift_target': 'sift_target', 'simple_target': 'simple_target', 'camera': 'camera', 'angle': 'angle', 'output_trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
