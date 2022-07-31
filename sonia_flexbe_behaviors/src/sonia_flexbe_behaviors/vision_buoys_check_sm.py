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
from sonia_vision_states.cross_check_vision_target import cross_check_vision_target
from sonia_vision_states.get_vision_target_list import get_vision_target_list
from sonia_vision_states.init_vision_target import init_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
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
		# x:883 y:30, x:856 y:457, x:500 y:685
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('start_deep',
										start_filter_chain(filterchain='deep_compe_front', target='Badge', camera_no=1),
										transitions={'continue': 'start_sift', 'failed': 'start_sift'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no', 'target': 'deep_target'})

			# x:199 y:367
			OperatableStateMachine.add('check_vision',
										cross_check_vision_target(center_bounding_box_pixel_height=100, center_bounding_box_pixel_width=100, bounding_box_pixel_height=350, bounding_box_pixel_width=200, image_height=400, image_width=600, max_mouvement=1.2, min_mouvement=0.1, long_rotation=False, speed_profile=0, timeout=10),
										transitions={'success': 'stop_deep_success', 'align': 'move', 'lost_target': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'lost_target': Autonomy.Off},
										remapping={'target_list_in': 'target_list', 'input_trajectory': 'trajectory', 'camera_no': 'camera_no', 'target_list_out': 'target_list', 'output_trajectory': 'trajectory'})

			# x:235 y:38
			OperatableStateMachine.add('get_deep',
										get_vision_target_list(number_of_average=10, timeout=15),
										transitions={'success': 'get_sift', 'lost_target': 'get_sift'},
										autonomy={'success': Autonomy.Off, 'lost_target': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no', 'target': 'deep_target', 'target_list_in': 'target_list', 'camera': 'camera', 'target_list_out': 'target_list'})

			# x:240 y:129
			OperatableStateMachine.add('get_sift',
										get_vision_target_list(number_of_average=10, timeout=3),
										transitions={'success': 'get_simple', 'lost_target': 'get_simple'},
										autonomy={'success': Autonomy.Off, 'lost_target': Autonomy.Off},
										remapping={'filterchain': 'sift_filterchain', 'camera_no': 'camera_no', 'target': 'sift_target', 'target_list_in': 'target_list', 'camera': 'camera', 'target_list_out': 'target_list'})

			# x:271 y:239
			OperatableStateMachine.add('get_simple',
										get_vision_target_list(number_of_average=10, timeout=3),
										transitions={'success': 'check_vision', 'lost_target': 'check_vision'},
										autonomy={'success': Autonomy.Off, 'lost_target': Autonomy.Off},
										remapping={'filterchain': 'simple_filterchain', 'camera_no': 'camera_no', 'target': 'simple_target', 'target_list_in': 'target_list', 'camera': 'camera', 'target_list_out': 'target_list'})

			# x:11 y:321
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'init_vision_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:42 y:415
			OperatableStateMachine.add('init_vision_target',
										init_vision_target(),
										transitions={'continue': 'get_deep'},
										autonomy={'continue': Autonomy.Off},
										remapping={'target_list': 'target_list'})

			# x:691 y:330
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'stop_deep_failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:32 y:537
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_deep', 'failed': 'stop_deep_failed', 'lost_target': 'stop_deep_failed', 'controller_error': 'stop_deep_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'deep_target', 'filterchain': 'deep_filterchain'})

			# x:30 y:132
			OperatableStateMachine.add('start_sift',
										start_filter_chain(filterchain='sift_front', target='makeGrade_badge', camera_no=1),
										transitions={'continue': 'start_simple', 'failed': 'start_simple'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'sift_filterchain', 'camera_no': 'camera_no', 'target': 'sift_target'})

			# x:30 y:224
			OperatableStateMachine.add('start_simple',
										start_filter_chain(filterchain='simple_buoy_badge', target='badge', camera_no=1),
										transitions={'continue': 'init_traj', 'failed': 'init_traj'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'simple_filterchain', 'camera_no': 'camera_no', 'target': 'simple_target'})

			# x:279 y:665
			OperatableStateMachine.add('stop_deep_error',
										stop_filter_chain(),
										transitions={'continue': 'controller_error', 'failed': 'controller_error'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no'})

			# x:608 y:557
			OperatableStateMachine.add('stop_deep_failed',
										stop_filter_chain(),
										transitions={'continue': 'stop_sift_failed', 'failed': 'stop_sift_failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no'})

			# x:524 y:29
			OperatableStateMachine.add('stop_deep_success',
										stop_filter_chain(),
										transitions={'continue': 'stop_sift_success', 'failed': 'stop_sift_success'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'deep_filterchain', 'camera_no': 'camera_no'})

			# x:594 y:660
			OperatableStateMachine.add('stop_sift_failed',
										stop_filter_chain(),
										transitions={'continue': 'stop_simple_failed', 'failed': 'stop_simple_failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'sift_filterchain', 'camera_no': 'camera_no'})

			# x:524 y:115
			OperatableStateMachine.add('stop_sift_success',
										stop_filter_chain(),
										transitions={'continue': 'stop_simple_success', 'failed': 'stop_simple_success'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'sift_filterchain', 'camera_no': 'camera_no'})

			# x:745 y:581
			OperatableStateMachine.add('stop_simple_failed',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'simple_filterchain', 'camera_no': 'camera_no'})

			# x:755 y:128
			OperatableStateMachine.add('stop_simple_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'simple_filterchain', 'camera_no': 'camera_no'})

			# x:340 y:510
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_deep', 'target_not_reached': 'check', 'error': 'stop_deep_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:551 y:227
			OperatableStateMachine.add('check',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_deep', 'moving': 'wait', 'error': 'stop_deep_failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
