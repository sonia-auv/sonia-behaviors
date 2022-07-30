#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_square_sm import search_squareSM
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
Created on Sat Jul 30 2022
@author: CS
'''
class vision_one_tableSM(Behavior):
	'''
	Find one table under the octagon.
	'''


	def __init__(self):
		super(vision_one_tableSM, self).__init__()
		self.name = 'vision_one_table'

		# parameters of this behavior
		self.add_parameter('one_table_activate_vision', True)
		self.add_parameter('one_table_filterchain', 'simple_one_table')
		self.add_parameter('one_table_target', 'Table')
		self.add_parameter('one_table_camera', 2)
		self.add_parameter('one_table_bounding_box_height', 100)
		self.add_parameter('one_table_bounding_box_width', 100)
		self.add_parameter('one_table_center_bounding_box_height', 100)
		self.add_parameter('one_table_center_bounding_box_width', 100)
		self.add_parameter('one_table_max_mouv', 1.0)
		self.add_parameter('one_table_min_mouv', 0.1)

		# references to used behaviors
		self.add_behavior(search_squareSM, 'search_square')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:570 y:35, x:171 y:225, x:305 y:436
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:38
			OperatableStateMachine.add('activate_vision_one_table',
										activate_behavior(activate=self.one_table_activate_vision),
										transitions={'activate': 'start_one_table', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:1070 y:360
			OperatableStateMachine.add('align',
										yaw_orbit_from_given_point_and_angle(pointX=0.16818, pointY=0),
										transitions={'continue': 'move_success'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'camera': 'camera', 'angle': 'angle', 'trajectory': 'trajectory'})

			# x:699 y:128
			OperatableStateMachine.add('get_vision_one_table',
										get_simple_vision_target(center_bounding_box_pixel_height=self.one_table_center_bounding_box_height, center_bounding_box_pixel_width=self.one_table_center_bounding_box_width, bounding_box_pixel_height=self.one_table_bounding_box_height, bounding_box_pixel_width=self.one_table_bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.one_table_max_mouv, min_mouvement=self.one_table_min_mouv, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'align', 'align': 'move', 'move': 'move', 'failed': 'failed', 'search': 'search_square'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:387 y:474
			OperatableStateMachine.add('give_me_target',
										is_moving(timeout=10, tolerance=0.1),
										transitions={'stopped': 'get_vision_one_table', 'moving': 'wait_target_reached', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:468 y:91
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_vision_one_table'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:959 y:88
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'stop_filter_success', 'moving': 'wait_success', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:880 y:482
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1131 y:131
			OperatableStateMachine.add('move_success',
										send_to_planner(),
										transitions={'continue': 'wait_success', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:71 y:312
			OperatableStateMachine.add('search_square',
										self.use_behavior(search_squareSM, 'search_square'),
										transitions={'finished': 'get_vision_one_table', 'failed': 'failed', 'lost_target': 'lost_target', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:291 y:132
			OperatableStateMachine.add('start_one_table',
										start_filter_chain(filterchain=self.one_table_filterchain, target=self.one_table_target, camera_no=self.one_table_camera),
										transitions={'continue': 'init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:707 y:25
			OperatableStateMachine.add('stop_filter_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:1154 y:13
			OperatableStateMachine.add('wait_success',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'is_moving', 'target_not_reached': 'move_success', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:672 y:232
			OperatableStateMachine.add('wait_target_reached',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_vision_one_table', 'target_not_reached': 'give_me_target', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
