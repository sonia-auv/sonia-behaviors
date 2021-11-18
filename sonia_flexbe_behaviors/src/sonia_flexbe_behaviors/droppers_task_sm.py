#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.droppers_sm import droppersSM
from sonia_flexbe_behaviors.search_bottom_sm import search_bottomSM
from sonia_flexbe_states.get_vision_target import get_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William Brouillard
'''
class droppers_taskSM(Behavior):
	'''
	Find the bin with the right filter chain, then drop the markers in the bin.
	'''


	def __init__(self):
		super(droppers_taskSM, self).__init__()
		self.name = 'droppers_task'

		# parameters of this behavior
		self.add_parameter('filter_chain', 'deep_bat')

		# references to used behaviors
		self.add_behavior(droppersSM, 'droppers')
		self.add_behavior(search_bottomSM, 'search_bottom')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:756 y:373, x:429 y:499, x:113 y:345
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:178
			OperatableStateMachine.add('start_filter_chain',
										start_filter_chain(param_node_name=self.filter_chain, camera_no=2, param_cmd=1),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:428 y:64
			OperatableStateMachine.add('get_target',
										get_vision_target(bounding_box_pixel=75, target_width_meter=0.9, target_height_meter=0.6, ratio_victory=0.8, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=60),
										transitions={'success': 'droppers', 'move': 'move_to_target', 'failed': 'failed', 'search': 'search_bottom'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'pose': 'target_pose'})

			# x:546 y:186
			OperatableStateMachine.add('move_to_target',
										move_to_target(),
										transitions={'continue': 'get_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target_pose'})

			# x:265 y:192
			OperatableStateMachine.add('search_bottom',
										self.use_behavior(search_bottomSM, 'search_bottom'),
										transitions={'finished': 'get_target', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:757 y:194
			OperatableStateMachine.add('droppers',
										self.use_behavior(droppersSM, 'droppers'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
