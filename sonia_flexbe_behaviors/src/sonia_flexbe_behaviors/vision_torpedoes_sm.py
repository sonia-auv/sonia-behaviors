#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_torpedoes_sm import search_torpedoesSM
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.get_front_vision_target_rotation import get_front_vision_target_rotation
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 13 2022
@author: CS
'''
class vision_torpedoesSM(Behavior):
	'''
	Behavior to detect the hole.
	'''


	def __init__(self):
		super(vision_torpedoesSM, self).__init__()
		self.name = 'vision_torpedoes'

		# parameters of this behavior
		self.add_parameter('torpedoes_filterchain', 'simple_torpedoes_star')
		self.add_parameter('torpedoes_target', 'torpedoes')
		self.add_parameter('camera_no', 1)

		# references to used behaviors
		self.add_behavior(search_torpedoesSM, 'search_torpedoes')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:997 y:12, x:130 y:400, x:1112 y:253, x:787 y:442
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
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:473 y:48
			OperatableStateMachine.add('get_target_rotate',
										get_front_vision_target_rotation(center_bounding_box_pixel_height=100, center_bounding_box_pixel_width=100, bounding_box_pixel_height=300, bounding_box_pixel_width=300, image_height=400, image_width=600, number_of_average=10, max_mouvement=0.1, min_mouvement=0.01, max_rotation=25, min_rotation=2, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_filter_success', 'align': 'move', 'move': 'move', 'failed': 'stop_filter_failed', 'search': 'search_torpedoes'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:239 y:56
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target_rotate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:307 y:240
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_reach', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:806 y:90
			OperatableStateMachine.add('search_torpedoes',
										self.use_behavior(search_torpedoesSM, 'search_torpedoes'),
										transitions={'finished': 'get_target_rotate', 'failed': 'failed', 'lost_target': 'stop_filter_lost', 'controller_error': 'controller_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:197 y:179
			OperatableStateMachine.add('stop_filter_failed',
										stop_filter_chain(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:857 y:237
			OperatableStateMachine.add('stop_filter_lost',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:784 y:7
			OperatableStateMachine.add('stop_filter_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:576 y:324
			OperatableStateMachine.add('wait_reach',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_target_rotate', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:618 y:171
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'get_target_rotate', 'moving': 'wait_reach', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
