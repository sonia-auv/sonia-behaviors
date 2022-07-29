#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.search_zigzag_sm import search_zigzagSM
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_navigation_states.yaw_orbit_from_given_point_and_angle import yaw_orbit_from_given_point_and_angle
from sonia_vision_states.get_simple_vision_target import get_simple_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class vision_pathSM(Behavior):
	'''
	Behaviors for the task of the path and rotate to the right orientationLook for a vision target on the front camera
	'''


	def __init__(self):
		super(vision_pathSM, self).__init__()
		self.name = 'vision_path'

		# parameters of this behavior
		self.add_parameter('vision_path_filterchain', 'simple_pipe_straight')
		self.add_parameter('vision_path_target', 'pipe straight')
		self.add_parameter('camera_no', 2)
		self.add_parameter('min_mouvement', 0.1)
		self.add_parameter('max_mouvement', 0.5)
		self.add_parameter('bounding_box_height', 225)
		self.add_parameter('bounding_box_width', 50)
		self.add_parameter('center_bounding_box_height', 50)
		self.add_parameter('center_bounding_box_width', 50)
		self.add_parameter('activate_vision_path', True)

		# references to used behaviors
		self.add_behavior(moveSM, 'move')
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:741, x:171 y:605, x:326 y:355
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('activation',
										activate_behavior(activate=self.activate_vision_path),
										transitions={'activate': 'start path filter', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:602 y:271
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'is_moving', 'failed': 'stop_filter_lost'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory'})

			# x:474 y:20
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=5, speed_profile=0),
										transitions={'success': 'rotate', 'align': 'align', 'move': 'align', 'failed': 'stop_filter_fail', 'search': 'search_zigzag'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'trajectory', 'output_trajectory': 'output_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:196 y:19
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:475 y:190
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_target', 'moving': 'wait', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:746 y:595
			OperatableStateMachine.add('is_moving_rotation',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'stop_filter_success', 'moving': 'wait_rotate', 'error': 'stop_filter_fail'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:303 y:656
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1092 y:148
			OperatableStateMachine.add('rotate',
										yaw_orbit_from_given_point_and_angle(pointX=0, pointY=0),
										transitions={'continue': 'rotate_on_path'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'output_trajectory', 'camera': 'camera', 'angle': 'angle', 'trajectory': 'trajectory'})

			# x:1059 y:379
			OperatableStateMachine.add('rotate_on_path',
										send_to_planner(),
										transitions={'continue': 'wait_rotate', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:214 y:105
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost', 'controller_error': 'stop_filter_fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:59 y:210
			OperatableStateMachine.add('start path filter',
										start_filter_chain(filterchain=self.vision_path_filterchain, target=self.vision_path_target, camera_no=self.camera_no),
										transitions={'continue': 'init_traj', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:148 y:407
			OperatableStateMachine.add('stop_filter_fail',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:431 y:294
			OperatableStateMachine.add('stop_filter_lost',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:537 y:707
			OperatableStateMachine.add('stop_filter_success',
										stop_filter_chain(),
										transitions={'continue': 'move', 'failed': 'move'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:959 y:262
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'get_target', 'target_not_reached': 'is_moving', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1073 y:471
			OperatableStateMachine.add('wait_rotate',
										wait_target_reached(timeout=15),
										transitions={'target_reached': 'stop_filter_success', 'target_not_reached': 'is_moving_rotation', 'error': 'stop_filter_fail'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
