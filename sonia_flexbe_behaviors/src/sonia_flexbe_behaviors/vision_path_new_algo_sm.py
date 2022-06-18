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
		self.add_parameter('filterchain_name', 'simple_pipe_straight')
		self.add_parameter('header_name', 'pipe straight')
		self.add_parameter('camera_no', 4)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1135 y:470, x:550 y:677, x:1237 y:280
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:243
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:486 y:36
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(bounding_box_pixel_height=300, bounding_box_pixel_width=50, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=20, speed_profile=0),
										transitions={'success': 'rotate', 'align': 'align', 'move': 'planner', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'input_trajectory': 'trajectory', 'output_trajectory': 'output_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:253 y:45
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:216 y:203
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_move', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})

			# x:30 y:40
			OperatableStateMachine.add('rotate',
										yaw_orbit_from_given_point_and_angle(pointX=0, pointY=0),
										transitions={'continue': 'rotate_on_path'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory', 'camera': 'camera', 'angle': 'angle', 'trajectory': 'trajectory'})

			# x:581 y:234
			OperatableStateMachine.add('rotate_on_path',
										send_to_planner(),
										transitions={'continue': 'wait_rotate', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1136 y:27
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:504 y:552
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:1177 y:169
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:989 y:565
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:783 y:144
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'stop_filter_fail', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:396 y:199
			OperatableStateMachine.add('wait_move',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'stop_filter_fail', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:793 y:486
			OperatableStateMachine.add('wait_rotate',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'stop_filter_success', 'target_not_reached': 'stop_filter_fail', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:963 y:71
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'stop_filter_lost'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
