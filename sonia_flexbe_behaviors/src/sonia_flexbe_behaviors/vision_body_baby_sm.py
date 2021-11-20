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
class vision_body_babySM(Behavior):
	'''
	Behaviors for the task to detect the baby.
	'''


	def __init__(self):
		super(vision_body_babySM, self).__init__()
		self.name = 'vision_body_baby'

		# parameters of this behavior
		self.add_parameter('filterchain_name', 'simple_body_baby')
		self.add_parameter('camera_no', 2)
		self.add_parameter('header_name', 'body_baby')

		# references to used behaviors
		self.add_behavior(search_bottomSM, 'search_bottom')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:982 y:102, x:544 y:683, x:418 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:71 y:184
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:724 y:249
			OperatableStateMachine.add('move to target',
										move_to_target(),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:312 y:200
			OperatableStateMachine.add('search_bottom',
										self.use_behavior(search_bottomSM, 'search_bottom'),
										transitions={'finished': 'get target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:509 y:560
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:349 y:313
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:771 y:60
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:521 y:52
			OperatableStateMachine.add('get target',
										get_vision_target(bounding_box_pixel=150, target_width_meter=0.6, target_height_meter=0.3, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=30),
										transitions={'success': 'stop_filter_success', 'move': 'move to target', 'failed': 'stop_filter_fail', 'search': 'search_bottom'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'pose': 'target_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
