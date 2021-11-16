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

		# references to used behaviors
		self.add_behavior(search_bottomSM, 'search_bottom')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1212 y:229, x:549 y:514, x:420 y:333
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:71 y:184
			OperatableStateMachine.add('start path filter',
										start_filter_chain(param_node_name=self.filterchain_name, camera_no=4, param_cmd=1),
										transitions={'continue': 'get target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:725 y:219
			OperatableStateMachine.add('move to target',
										move_to_target(),
										transitions={'continue': 'get target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:984 y:232
			OperatableStateMachine.add('rotate to path',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:312 y:200
			OperatableStateMachine.add('search_bottom',
										self.use_behavior(search_bottomSM, 'search_bottom'),
										transitions={'finished': 'get target', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:521 y:52
			OperatableStateMachine.add('get target',
										get_vision_target(bounding_box_pixel=150, target_width_meter=0.6, target_height_meter=1.2, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=30),
										transitions={'success': 'rotate to path', 'move': 'move to target', 'failed': 'failed', 'search': 'search_bottom'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'pose': 'target_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
