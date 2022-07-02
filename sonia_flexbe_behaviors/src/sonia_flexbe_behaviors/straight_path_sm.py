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
from sonia_vision_states.get_bottom_vision_target import get_bottom_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: CS
'''
class straight_pathSM(Behavior):
	'''
	Behaviors for the task of the path and rotate to the right orientationLook for a vision target on the front camera
	'''


	def __init__(self):
		super(straight_pathSM, self).__init__()
		self.name = 'straight_path'

		# parameters of this behavior
		self.add_parameter('path_filterchain', 'simple_pipe_straight')
		self.add_parameter('path_header', 'pipe straight')
		self.add_parameter('path_camera', 4)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:888 y:652, x:550 y:677, x:687 y:449, x:967 y:35
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'max_attempt'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:14 y:254
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.path_filterchain, header_name=self.path_header, camera_no=self.path_camera, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:1035 y:474
			OperatableStateMachine.add('check moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_filter_success', 'moving': 'wait_rotate', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:253 y:45
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'path'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:469 y:22
			OperatableStateMachine.add('path',
										get_bottom_vision_target(bounding_box_pixel_height=300, bounding_box_pixel_width=50, bounding_box_offset_x=-100, bounding_box_offset_y=0, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=15, speed_profile=0, max_tentative=10),
										transitions={'success': 'rotate', 'align': 'align', 'move': 'planner', 'failed': 'stop_filter_fail', 'search': 'search_zigzag', 'max_attempt': 'max_attempt'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off, 'max_attempt': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'input_trajectory': 'trajectory', 'output_trajectory': 'output_trajectory', 'camera': 'camera', 'angle': 'angle'})

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
										transitions={'finished': 'path', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:504 y:552
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.path_filterchain, header_name=self.path_header, camera_no=self.path_camera, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:767 y:362
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.path_filterchain, header_name=self.path_header, camera_no=self.path_camera, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:997 y:598
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.path_filterchain, header_name=self.path_header, camera_no=self.path_camera, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:30 y:40
			OperatableStateMachine.add('the',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'path', 'moving': 'wait_move', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:1145 y:288
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'path', 'target_not_reached': 'what', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:401 y:219
			OperatableStateMachine.add('wait_move',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'path', 'target_not_reached': 'the', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1225 y:567
			OperatableStateMachine.add('wait_rotate',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'stop_filter_success', 'target_not_reached': 'check moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:976 y:373
			OperatableStateMachine.add('what',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'path', 'moving': 'wait', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:827 y:257
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'stop_filter_lost'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
