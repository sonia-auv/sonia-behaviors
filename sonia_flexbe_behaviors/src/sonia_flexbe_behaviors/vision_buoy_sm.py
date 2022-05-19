#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_snake_sm import search_snakeSM
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_vision_states.get_vision_target import get_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 13 2021
@author: FA
'''
class vision_buoySM(Behavior):
	'''
	Detect the buoy to find the octogon
	'''


	def __init__(self):
		super(vision_buoySM, self).__init__()
		self.name = 'vision_buoy'

		# parameters of this behavior
		self.add_parameter('filter_name', 'simple_buoy')
		self.add_parameter('camera_no', 3)
		self.add_parameter('header_name', 'buoy')

		# references to used behaviors
		self.add_behavior(search_snakeSM, 'search_snake')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:913 y:72, x:136 y:617, x:354 y:399
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:53
			OperatableStateMachine.add('set control mode',
										set_control_mode(mode=11, timeout=2),
										transitions={'continue': 'filter_chain', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:376 y:46
			OperatableStateMachine.add('get_target',
										get_vision_target(bounding_box_pixel=100, target_width_meter=0.1, target_height_meter=0.1, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=30),
										transitions={'success': 'stop_filter_success', 'move': 'set mode single', 'failed': 'stop_filter_fail', 'search': 'search_snake'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front', 'pose': 'target'})

			# x:726 y:231
			OperatableStateMachine.add('move',
										move_to_target(),
										transitions={'continue': 'get_target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target'})

			# x:352 y:150
			OperatableStateMachine.add('search_snake',
										self.use_behavior(search_snakeSM, 'search_snake'),
										transitions={'finished': 'get_target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:575 y:133
			OperatableStateMachine.add('set mode single',
										set_control_mode(mode=11, timeout=2),
										transitions={'continue': 'move', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:120 y:477
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filter_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:368 y:294
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filter_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:701 y:40
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filter_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:207 y:39
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(param_node_name=self.filter_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'get_target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front', 'header_name': 'header_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
