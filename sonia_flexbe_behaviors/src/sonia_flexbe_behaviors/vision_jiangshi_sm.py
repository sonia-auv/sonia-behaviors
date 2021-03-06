#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.aligment_with_stopping_sm import AligmentwithstoppingSM
from sonia_flexbe_behaviors.search_snake_sm import search_snakeSM
from sonia_flexbe_states.get_simple_vision_target import get_simple_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 13 2021
@author: William Brouillard
'''
class vision_jiangshiSM(Behavior):
	'''
	Detect the jiangshi and ram into it.
	'''


	def __init__(self):
		super(vision_jiangshiSM, self).__init__()
		self.name = 'vision_jiangshi'

		# parameters of this behavior
		self.add_parameter('filtername', 'deep_jiangshi')
		self.add_parameter('cam_number', 1)
		self.add_parameter('header_name', 'jiangshi')

		# references to used behaviors
		self.add_behavior(AligmentwithstoppingSM, 'Aligment with stopping')
		self.add_behavior(AligmentwithstoppingSM, 'Aligment with stopping_2')
		self.add_behavior(search_snakeSM, 'search_snake')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1171 y:65, x:285 y:666, x:388 y:449
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:200
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(param_node_name=self.filtername, header_name=self.header_name, camera_no=self.cam_number, param_cmd=1),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:1142 y:332
			OperatableStateMachine.add('Aligment with stopping_2',
										self.use_behavior(AligmentwithstoppingSM, 'Aligment with stopping_2'),
										transitions={'lost_target': 'stop_filter_success', 'failed': 'stop_filter_fail', 'success': 'stop_filter_success'},
										autonomy={'lost_target': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'pose', 'filterchain': 'filterchain', 'header_name': 'header_name', 'bounding_box': 'bounding_box'})

			# x:489 y:32
			OperatableStateMachine.add('get target',
										get_simple_vision_target(bounding_box_pixel=75, image_height=400, image_width=600, ratio_victory=0.75, number_of_average=10, max_mouvement=1, alignement_distance=5, rotation=False, timeout=60),
										transitions={'success': 'stop_filter_success', 'align': 'Aligment with stopping', 'move': 'move', 'failed': 'stop_filter_fail', 'search': 'search_snake'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'pose': 'pose', 'bounding_box': 'bounding_box'})

			# x:1070 y:523
			OperatableStateMachine.add('get_vision_2',
										get_simple_vision_target(bounding_box_pixel=50, image_height=400, image_width=600, ratio_victory=0.75, number_of_average=10, max_mouvement=1, alignement_distance=5, rotation=False, timeout=30),
										transitions={'success': 'stop_filter_success', 'align': 'Aligment with stopping_2', 'move': 'move_target', 'failed': 'stop_filter_fail', 'search': 'stop_filter_success'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'pose': 'pose_2', 'bounding_box': 'bounding_box'})

			# x:576 y:307
			OperatableStateMachine.add('move',
										move_to_target(),
										transitions={'continue': 'get_vision_2', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:867 y:303
			OperatableStateMachine.add('move_target',
										move_to_target(),
										transitions={'continue': 'stop_filter_success', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose_2'})

			# x:325 y:171
			OperatableStateMachine.add('search_snake',
										self.use_behavior(search_snakeSM, 'search_snake'),
										transitions={'finished': 'get target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:477 y:607
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filtername, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:339 y:334
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filtername, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:835 y:47
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filtername, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:712 y:173
			OperatableStateMachine.add('Aligment with stopping',
										self.use_behavior(AligmentwithstoppingSM, 'Aligment with stopping'),
										transitions={'lost_target': 'stop_filter_lost', 'failed': 'stop_filter_fail', 'success': 'get target'},
										autonomy={'lost_target': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'pose', 'filterchain': 'filterchain', 'header_name': 'header_name', 'bounding_box': 'bounding_box'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
