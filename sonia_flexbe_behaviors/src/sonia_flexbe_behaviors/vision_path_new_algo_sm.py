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
		# x:1081 y:521, x:550 y:677, x:435 y:473
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
										get_simple_vision_target(bounding_box_pixel=100, image_height=400, image_width=600, ratio_victory=0.05, number_of_average=10, max_mouvement=1, alignement_distance=5, long_rotation=False, timeout=20, speed_profile=0),
										transitions={'success': 'rotate_on_path', 'align': 'align', 'move': 'planner', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'input_trajectory': 'trajectory', 'output_trajectory': 'output_trajectory', 'bounding_box': 'bounding_box'})

			# x:253 y:45
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:561 y:186
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'get_target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})

			# x:1093 y:226
			OperatableStateMachine.add('rotate_on_path',
										send_to_planner(),
										transitions={'continue': 'stop_filter_success', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})

			# x:319 y:187
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

			# x:349 y:339
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:1075 y:363
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:774 y:225
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'get_target', 'failed': 'stop_filter_lost'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
