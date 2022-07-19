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
from sonia_navigation_states.yaw_orbit_from_given_point_and_angle import yaw_orbit_from_given_point_and_angle
from sonia_vision_states.get_simple_vision_target import get_simple_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class vision_path_new_algoSM(Behavior):
	'''
	Behaviors for the task of the path and rotate to the right orientationLook for a vision target on the front camera
	'''


	def __init__(self):
		super(vision_path_new_algoSM, self).__init__()
		self.name = 'vision_path_new_algo'

		# parameters of this behavior
		self.add_parameter('filterchain', 'simple_pipe_straight')
		self.add_parameter('target', 'pipe straight')
		self.add_parameter('camera_no', 2)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:888 y:652, x:550 y:677, x:687 y:449
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:14 y:254
			OperatableStateMachine.add('start path filter',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:486 y:36
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(center_bounding_box_pixel_height=50, center_bounding_box_pixel_width=50, bounding_box_pixel_height=300, bounding_box_pixel_width=50, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=20, speed_profile=0),
										transitions={'success': 'rotate', 'align': 'align', 'move': 'planner', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'trajectory', 'output_trajectory': 'output_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:253 y:45
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:159 y:515
			OperatableStateMachine.add('is_mov',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait_move', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:1440 y:281
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:1305 y:620
			OperatableStateMachine.add('is_moving_rotation',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_filter_success', 'moving': 'wait_rotate', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:206 y:214
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_move', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})

			# x:1092 y:148
			OperatableStateMachine.add('rotate',
										yaw_orbit_from_given_point_and_angle(pointX=0, pointY=0),
										transitions={'continue': 'rotate_on_path'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory', 'camera': 'camera', 'angle': 'angle', 'trajectory': 'trajectory'})

			# x:1232 y:398
			OperatableStateMachine.add('rotate_on_path',
										send_to_planner(),
										transitions={'continue': 'wait_rotate', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:587 y:217
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'filterchain', 'filterchain': 'filterchain'})

			# x:504 y:552
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:767 y:362
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:997 y:598
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:1127 y:324
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'is_moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:376 y:226
			OperatableStateMachine.add('wait_move',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'is_mov', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1139 y:530
			OperatableStateMachine.add('wait_rotate',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'stop_filter_success', 'target_not_reached': 'is_moving_rotation', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:827 y:257
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'is_moving', 'failed': 'stop_filter_lost'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
