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
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.get_simple_vision_target import get_simple_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
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
		self.add_parameter('vision_bins_filterchain', 'deep_compe_bottom')
		self.add_parameter('vision_bins_target', 'cover')
		self.add_parameter('camera_no', 2)
		self.add_parameter('bounding_box_width', 200)
		self.add_parameter('bounding_box_height', 200)
		self.add_parameter('center_bb_height', 50)
		self.add_parameter('center_bb_width', 50)
		self.add_parameter('max_mouvement', 1.0)
		self.add_parameter('min_mouvement', 0.1)
		self.add_parameter('activate_vision_bins', True)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:193 y:141, x:140 y:454, x:590 y:631, x:809 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('activation',
										activate_behavior(activate=self.activate_vision_bins),
										transitions={'activate': 'find_bins', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:844 y:31
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait_align', 'failed': 'stop_lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'input_trajectory'})

			# x:516 y:225
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_bins', 'moving': 'wait_align', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:20 y:330
			OperatableStateMachine.add('find_bins',
										start_filter_chain(filterchain=self.vision_bins_filterchain, target=self.vision_bins_target, camera_no=self.camera_no),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:399 y:46
			OperatableStateMachine.add('get_bins',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bb_height, center_bounding_box_pixel_width=self.center_bb_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_success', 'align': 'align', 'move': 'align', 'failed': 'failed', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'input_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:108 y:202
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_bins'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:490 y:354
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_bins', 'failed': 'failed', 'lost_target': 'stop_lost_target', 'controller_error': 'controller_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:874 y:432
			OperatableStateMachine.add('stop_lost_target',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:197 y:31
			OperatableStateMachine.add('stop_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:727 y:142
			OperatableStateMachine.add('wait_align',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'get_bins', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
