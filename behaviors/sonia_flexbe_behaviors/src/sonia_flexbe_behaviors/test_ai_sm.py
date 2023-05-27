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
from sonia_hardware_states.wait_mission import wait_mission
from sonia_vision_states.find_vision_target import find_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 31 2022
@author: CS
'''
class test_aiSM(Behavior):
	'''
	test ai durability
	'''


	def __init__(self):
		super(test_aiSM, self).__init__()
		self.name = 'test_ai'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:772 y:386, x:422 y:437
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:94 y:137
			OperatableStateMachine.add('mission',
										wait_mission(),
										transitions={'continue': 'start_deep'},
										autonomy={'continue': Autonomy.Off})

			# x:505 y:127
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10),
										transitions={'continue': 'drop'},
										autonomy={'continue': Autonomy.Off},
										remapping={'topic': 'topic', 'target': 'target'})

			# x:342 y:124
			OperatableStateMachine.add('start_deep',
										start_filter_chain(filterchain='deep_compe_front', target='G-Man', camera_no=1),
										transitions={'continue': 'find_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:729 y:131
			OperatableStateMachine.add('drop',
										activate_io(element=1, side=0, action=1, timeout=8),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
