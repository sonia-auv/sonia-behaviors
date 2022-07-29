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
Created on 27/08/2022
@author: GS
'''
class vision_torpedoes_boardsSM(Behavior):
	'''
	Detect the torpedoes and align
	'''


	def __init__(self):
		super(vision_torpedoes_boardsSM, self).__init__()
		self.name = 'vision_torpedoes_boards'

		# parameters of this behavior
		self.add_parameter('vision_torpedoes_boards_filterchain', 'deep_compe_front')
		self.add_parameter('camera_no', 1)
		self.add_parameter('vision_torpedoes_boards_target', 'G-Man')
		self.add_parameter('bounding_box_width', 200)
		self.add_parameter('bounding_box_height', 350)
		self.add_parameter('center_bounding_box_width', 100)
		self.add_parameter('center_bounding_box_height', 100)
		self.add_parameter('max_mouvement', 1)
		self.add_parameter('min_mouvement', 0.25)
		self.add_parameter('activate_vision_buoys', True)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1454 y:39, x:1039 y:794, x:456 y:420
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('activation',
										activate_behavior(activate=self.activate_vision_buoys),
										transitions={'activate': 'filter_chain', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:960 y:578
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait_target_reached', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:43 y:367
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(filterchain=self.vision_torpedoes_boards_filterchain, target=self.vision_torpedoes_boards_target, camera_no=self.camera_no),
										transitions={'continue': 'init', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'front', 'target': 'target'})

			# x:916 y:57
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_filter_success', 'align': 'move', 'move': 'move', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'front', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:225 y:72
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:596 y:260
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:483 y:157
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:524 y:586
			OperatableStateMachine.add('stop_filter_fail',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:270 y:230
			OperatableStateMachine.add('stop_filter_lost',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:1241 y:115
			OperatableStateMachine.add('stop_filter_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:838 y:393
			OperatableStateMachine.add('wait_target_reached',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'check_moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
