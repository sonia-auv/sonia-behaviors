#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.search_zigzag_sm import search_zigzagSM
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 11 2022
@author: KY
'''
class search_pathSM(Behavior):
	'''
	Search for path and get movement from vision filterchain.
	'''


	def __init__(self):
		super(search_pathSM, self).__init__()
		self.name = 'search_path'

		# parameters of this behavior
		self.add_parameter('filterchain_name', 'simple_pipe_straight')
		self.add_parameter('header_name', 'pipe straight')
		self.add_parameter('camera_no', 4)

		# references to used behaviors
		self.add_behavior(search_zigzagSM, 'search_zigzag')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:916 y:266, x:402 y:638, x:357 y:475
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:17 y:105
			OperatableStateMachine.add('start_path_filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'search_zigzag', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:267 y:196
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
