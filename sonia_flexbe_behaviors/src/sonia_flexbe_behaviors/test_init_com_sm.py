#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.init_mission_list import init_mission_list
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 17 2022
@author: FA
'''
class test_init_comSM(Behavior):
	'''
	Testing the initialisation of the underwater com
	'''


	def __init__(self):
		super(test_init_comSM, self).__init__()
		self.name = 'test_init_com'

		# parameters of this behavior
		self.add_parameter('auv_list', '1,1,1,0,0,0,0,1,1,1,1')
		self.add_parameter('other_auv_list', '1,1,0,1,1,1,1,0,0,0,0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:358 y:275
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:200 y:78
			OperatableStateMachine.add('init_com',
										init_mission_list(auv_list=self.auv_list, other_auv_list=self.other_auv_list),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
