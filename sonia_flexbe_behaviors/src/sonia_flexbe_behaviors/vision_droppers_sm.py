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
from sonia_vision_states.get_simple_vision_target import get_simple_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 06/07/2022
@author: CS
'''
class vision_droppersSM(Behavior):
	'''
	Find the bin with the right filter chain, then drop the markers in the bin.
	'''


	def __init__(self):
		super(vision_droppersSM, self).__init__()
		self.name = 'vision_droppers'

		# parameters of this behavior
		self.add_parameter('vision_droppers_filterchain', 'deep_compe_bottom')
		self.add_parameter('vision_droppers_target', 'Notepad')
		self.add_parameter('camera_no', 2)
		self.add_parameter('bounding_box_height', 90)
		self.add_parameter('bounding_box_width', 115)
		self.add_parameter('center_bounding_box_height', 50)
		self.add_parameter('center_bounding_box_width', 50)
		self.add_parameter('max_mouvement', 0.5)
		self.add_parameter('min_mouvement', 0.1)
		self.add_parameter('activate_vision_droppers', True)

		# references to used behaviors
		self.add_behavior(search_squareSM, 'search_square')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:274 y:263, x:66 y:423, x:99 y:587, x:406 y:469
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target', 'controller_error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:35 y:65
			OperatableStateMachine.add('activation',
										activate_behavior(activate=self.activate_vision_droppers),
										transitions={'activate': 'start_simple_rotate', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:424 y:220
			OperatableStateMachine.add('align',
										send_to_planner(),
										transitions={'continue': 'wait_align', 'failed': 'stop_lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'input_trajectory'})

			# x:621 y:375
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_bins', 'moving': 'wait_align', 'error': 'controller_error'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:33 y:236
			OperatableStateMachine.add('find_bins',
										start_filter_chain(filterchain=self.vision_droppers_filterchain, target=self.vision_droppers_target, camera_no=self.camera_no),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:646 y:37
			OperatableStateMachine.add('get_bins',
										get_simple_vision_target(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'stop_success', 'align': 'align', 'move': 'align', 'failed': 'failed', 'search': 'search_square'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'camera_no', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'input_trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:214 y:31
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_bins'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:789 y:467
			OperatableStateMachine.add('search_square',
										self.use_behavior(search_squareSM, 'search_square'),
										transitions={'finished': 'get_bins', 'failed': 'stop_lost_target', 'lost_target': 'stop_lost_target', 'controller_error': 'controller_error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:9 y:148
			OperatableStateMachine.add('start_simple_rotate',				
										start_filter_chain(filterchain='simple_rotate', target='', camera_no=2),
										transitions={'continue': 'find_bins', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:225 y:531
			OperatableStateMachine.add('stop_lost_target',
										stop_filter_chain(),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:253 y:109
			OperatableStateMachine.add('stop_success',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:412 y:311
			OperatableStateMachine.add('wait_align',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'get_bins', 'target_not_reached': 'check_moving', 'error': 'controller_error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
