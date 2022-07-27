#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
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
Created on Wed Jul 13 2022
@author: CS
'''
class vision_torpedoesSM(Behavior):
	'''
	Behavior to detect the gman.
	'''


	def __init__(self):
		super(vision_torpedoesSM, self).__init__()
		self.name = 'vision_torpedoes'

		# parameters of this behavior
		self.add_parameter('torpedoes_filterchain', 'deep_compe_front')
		self.add_parameter('torpedoes_target', 'aDefinir')
		self.add_parameter('camera_no', 1)
		self.add_parameter('bounding_box_width', 0)
		self.add_parameter('bounding_box_height', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:997 y:12, x:130 y:400, x:474 y:436, x:978 y:155
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:59 y:81
			OperatableStateMachine.add('detect_gman',
										start_filter_chain(filterchain=self.torpedoes_filterchain, target=self.torpedoes_target, camera_no=self.camera_no),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:560 y:66
			OperatableStateMachine.add('get_gman',
										get_simple_vision_target(center_bounding_box_pixel_height=50, center_bounding_box_pixel_width=50, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_filter_success', 'align': 'move', 'move': 'move', 'failed': 'stop_filter_failed', 'search': 'stop_filter_lost_target'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:239 y:56
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_gman'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:316 y:267
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_reach', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:182 y:195
			OperatableStateMachine.add('stop_filter_failed',
										start_filter_chain(filterchain=self.torpedoes_filterchain, target=self.torpedoes_target, camera_no=self.camera_no),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:697 y:462
			OperatableStateMachine.add('stop_filter_lost_target',
										start_filter_chain(filterchain=self.torpedoes_filterchain, target=self.torpedoes_target, camera_no=self.camera_no),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:757 y:32
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(filterchain=self.torpedoes_filterchain, target=self.torpedoes_target, camera_no=self.camera_no),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:641 y:316
			OperatableStateMachine.add('wait_reach',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_gman', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:695 y:186
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_gman', 'moving': 'wait_reach', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
