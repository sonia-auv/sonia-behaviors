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
from sonia_hardware_states.activate_io import activate_io
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
@author: Camille Sauvain
'''
class vision_droppersSM(Behavior):
	'''
	Find the bin with the right filter chain, then drop the markers in the bin.
	'''


	def __init__(self):
		super(vision_droppersSM, self).__init__()
		self.name = 'vision_droppers'

		# parameters of this behavior
		self.add_parameter('filterchain', 'deep_wolf')
		self.add_parameter('header_name', 'wolf')
		self.add_parameter('camera_no', 2)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365, x:230 y:365, x:695 y:466
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:78 y:98
			OperatableStateMachine.add('find_bins',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:791 y:297
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_bins', 'moving': 'wait_align', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:156 y:164
			OperatableStateMachine.add('droppers',
										activate_io(element=2, side=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:490 y:51
			OperatableStateMachine.add('get_bins',
										get_simple_vision_target(bounding_box_pixel_height=450, bounding_box_pixel_width=250, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_success', 'align': 'align', 'move': 'align', 'failed': 'stop_lost_target', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'input_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:175 y:40
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_bins'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:350 y:222
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_bins', 'failed': 'failed', 'lost_target': 'stop_lost_target', 'controller_error': 'controller_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:335 y:411
			OperatableStateMachine.add('stop_lost_target',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:346 y:100
			OperatableStateMachine.add('stop_success',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'droppers', 'failed': 'droppers'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:768 y:117
			OperatableStateMachine.add('wait_align',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'get_bins', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:649 y:224
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait_align', 'failed': 'stop_lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'input_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
