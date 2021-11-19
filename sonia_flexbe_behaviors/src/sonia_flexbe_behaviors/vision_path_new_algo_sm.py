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
from sonia_flexbe_behaviors.search_bottom_sm import search_bottomSM
from sonia_flexbe_states.get_simple_vision_target import get_simple_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class vision_path_new_algoSM(Behavior):
	'''
	Behaviors for the task of the path and rotate to the right orientation
	'''


	def __init__(self):
		super(vision_path_new_algoSM, self).__init__()
		self.name = 'vision_path_new_algo'

		# parameters of this behavior
		self.add_parameter('filterchain_name', 'simple_pipe45')
		self.add_parameter('header_name', 'pipe')

		# references to used behaviors
		self.add_behavior(AligmentwithstoppingSM, 'Aligment with stopping')
		self.add_behavior(search_bottomSM, 'search_bottom')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1156 y:460, x:550 y:677, x:416 y:412
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:243
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=4, param_cmd=1),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:486 y:36
			OperatableStateMachine.add('get_target',
										get_simple_vision_target(bounding_box_pixel=150, image_height=400, image_width=600, ratio_victory=0.3, number_of_average=10, max_mouvement=1, alignement_distance=5, timeout=20),
										transitions={'success': 'rotate to path', 'align': 'Aligment with stopping', 'move': 'move to target', 'failed': 'failed', 'search': 'search_bottom'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'pose': 'target_pose', 'bounding_box': 'bounding_box'})

			# x:596 y:260
			OperatableStateMachine.add('move to target',
										move_to_target(),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:1100 y:240
			OperatableStateMachine.add('rotate to path',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:324 y:227
			OperatableStateMachine.add('search_bottom',
										self.use_behavior(search_bottomSM, 'search_bottom'),
										transitions={'finished': 'get_target', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:755 y:222
			OperatableStateMachine.add('Aligment with stopping',
										self.use_behavior(AligmentwithstoppingSM, 'Aligment with stopping'),
										transitions={'lost_target': 'lost_target', 'failed': 'failed', 'success': 'get_target'},
										autonomy={'lost_target': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'target_pose', 'filterchain': 'filterchain', 'header_name': 'header_name', 'bounding_box': 'bounding_box'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
