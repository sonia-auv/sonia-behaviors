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
from sonia_vision_states.get_simple_vision_target import get_simple_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 06/07/2022
@author: CS
'''
class vision_binsSM(Behavior):
	'''
	Find the bins with the cover.
	'''


	def __init__(self):
		super(vision_binsSM, self).__init__()
		self.name = 'vision_bins'

		# parameters of this behavior
		self.add_parameter('filterchain', 'simulation_cover')
		self.add_parameter('target', 'bins_cover')
		self.add_parameter('camera_no', 2)
		self.add_parameter('bounding_box_width', 60)
		self.add_parameter('bounding_box_height', 60)
		self.add_parameter('center_bb_height', 50)
		self.add_parameter('center_bb_width', 50)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:455 y:272, x:130 y:365, x:590 y:631, x:771 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:163
			OperatableStateMachine.add('find_bins',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:516 y:225
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_bins', 'moving': 'wait_align', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:383 y:31
			OperatableStateMachine.add('get_bins',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bb_height, center_bounding_box_pixel_width=self.center_bb_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_success', 'align': 'align', 'move': 'align', 'failed': 'failed', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'input_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:175 y:40
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_bins'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:536 y:310
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_bins', 'failed': 'failed', 'lost_target': 'stop_lost_target', 'controller_error': 'controller_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'filterchain': 'filterchain'})

			# x:765 y:550
			OperatableStateMachine.add('stop_lost_target',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:292 y:277
			OperatableStateMachine.add('stop_success',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:603 y:101
			OperatableStateMachine.add('wait_align',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'get_bins', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:844 y:31
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait_align', 'failed': 'stop_lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'input_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
