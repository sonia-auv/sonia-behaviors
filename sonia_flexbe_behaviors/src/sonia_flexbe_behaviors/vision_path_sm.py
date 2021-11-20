#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_bottom_sm import search_bottomSM
from sonia_flexbe_states.get_vision_target import get_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class vision_pathSM(Behavior):
	'''
	Behaviors for the task of the path and rotate to the right orientation
	'''


	def __init__(self):
		super(vision_pathSM, self).__init__()
		self.name = 'vision_path'

		# parameters of this behavior
		self.add_parameter('filterchain_name', 'simple_pipe45')
		self.add_parameter('cam_number', 2)
		self.add_parameter('header_name', 'pipe')

		# references to used behaviors
		self.add_behavior(search_bottomSM, 'search_bottom')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1233 y:405, x:545 y:675, x:425 y:410
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:71 y:184
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.cam_number, param_cmd=1),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:725 y:219
			OperatableStateMachine.add('move to target',
										move_to_target(),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:921 y:228
			OperatableStateMachine.add('rotate to path',
										move_to_target(),
										transitions={'continue': 'stop_filter_success', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:325 y:192
			OperatableStateMachine.add('search_bottom',
										self.use_behavior(search_bottomSM, 'search_bottom'),
										transitions={'finished': 'get target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:499 y:547
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:352 y:297
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:1106 y:260
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.cam_number, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:521 y:52
			OperatableStateMachine.add('get target',
										get_vision_target(bounding_box_pixel=150, target_width_meter=0.6, target_height_meter=1.2, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=30),
										transitions={'success': 'rotate to path', 'move': 'move to target', 'failed': 'stop_filter_fail', 'search': 'search_bottom'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'pose': 'target_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
