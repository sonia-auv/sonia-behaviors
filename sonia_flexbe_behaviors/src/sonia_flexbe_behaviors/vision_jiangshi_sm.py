#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_front_sm import search_frontSM
from sonia_flexbe_states.get_vision_target import get_vision_target
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
		self.add_parameter('filter_name', 'deep_jiangshi')
		self.add_parameter('cam_number', 1)

		# references to used behaviors
		self.add_behavior(search_frontSM, 'search_front')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:913 y:72, x:182 y:519, x:387 y:357
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:133 y:62
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(param_node_name=self.filter_name, camera_no=self.cam_number, param_cmd=1),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'jiangshi', 'camera_no': 'front'})

			# x:365 y:46
			OperatableStateMachine.add('get_target',
										get_vision_target(bounding_box_pixel=100, target_width_meter=0.1, target_height_meter=0.1, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=30),
										transitions={'success': 'finished', 'move': 'move', 'failed': 'failed', 'search': 'search_front'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'jiangshi', 'camera_no': 'front', 'pose': 'target'})

			# x:726 y:231
			OperatableStateMachine.add('move',
										move_to_target(),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target'})

			# x:379 y:202
			OperatableStateMachine.add('search_front',
										self.use_behavior(search_frontSM, 'search_front'),
										transitions={'finished': 'get_target', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
