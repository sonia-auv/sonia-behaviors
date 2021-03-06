#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.check_collision_sm import check_collisionSM
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
Created on Tue Jul 12 2022
@author: CS
'''
class vision_buoysSM(Behavior):
	'''
	Detect the buoy and align
	'''


	def __init__(self):
		super(vision_buoysSM, self).__init__()
		self.name = 'vision_buoys'

		# parameters of this behavior
		self.add_parameter('filterchain', 'simple_buoy_badge')
		self.add_parameter('camera_no', 1)
		self.add_parameter('target', 'obstacle')
		self.add_parameter('bounding_box_width', 250)
		self.add_parameter('bounding_box_height', 300)
		self.add_parameter('center_bounding_box_width', 50)
		self.add_parameter('center_bounding_box_height', 50)
		self.add_parameter('max_mouvement', 1.2)
		self.add_parameter('min_mouvement', 0.25)

		# references to used behaviors
		self.add_behavior(check_collisionSM, 'check_collision')
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:915 y:223, x:110 y:631, x:265 y:395
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:157
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'init', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front', 'target': 'target'})

			# x:797 y:471
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait_target_reached', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:706 y:50
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_filter_success', 'align': 'move', 'move': 'move', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:217 y:45
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:531 y:215
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:214 y:138
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'filterchain': 'filterchain'})

			# x:109 y:475
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:258 y:239
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:973 y:74
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'check_collision', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:561 y:368
			OperatableStateMachine.add('wait_target_reached',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'check_moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1207 y:244
			OperatableStateMachine.add('check_collision',
										self.use_behavior(check_collisionSM, 'check_collision'),
										transitions={'failed': 'failed', 'target_reached': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'target_reached': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
