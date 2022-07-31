#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.activate_io import activate_io
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri May 13 2022
@author: guilhem
'''
class test_filterchainSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(test_filterchainSM, self).__init__()
		self.name = 'test_filterchain'

		# parameters of this behavior
		self.add_parameter('cam_numero', 1)
		self.add_parameter('filterchain', 'simulation_cover')
		self.add_parameter('target', 'test')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:369 y:392, x:112 y:203
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:136 y:46
			OperatableStateMachine.add('test start buoy',
										start_filter_chain(filterchain=self.filterchain, target=self.target, camera_no=self.cam_numero),
										transitions={'continue': 'wait_', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'node': 'node', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:372 y:95
			OperatableStateMachine.add('wait_',
										activate_io(element=1, side=1, timeout=5),
										transitions={'continue': 'stop_deep', 'failed': 'stop_deep'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:356 y:271
			OperatableStateMachine.add('stop_deep',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'node': 'filterchain', 'filterchain': 'filterchain', 'camera_no': 'camera_no'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
