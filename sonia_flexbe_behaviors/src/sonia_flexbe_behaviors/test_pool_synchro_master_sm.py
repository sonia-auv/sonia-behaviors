#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.send_update import send_update
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.synchro_master_sm import SynchroMasterSM
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 20 2022
@author: FA
'''
class testpoolsynchromasterSM(Behavior):
	'''
	Testing the synchronisation between the 2 subs with a 180deg rotation
	'''


	def __init__(self):
		super(testpoolsynchromasterSM, self).__init__()
		self.name = 'test pool synchro master'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SynchroMasterSM, 'Synchro Master')
		self.add_behavior(moveSM, 'move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:996 y:535, x:495 y:454
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:94 y:106
			OperatableStateMachine.add('start_aray',
										send_update(mission=0, state=1),
										transitions={'continue': 'Synchro Master'},
										autonomy={'continue': Autonomy.Off})

			# x:758 y:96
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 180, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'target_reached', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1015 y:251
			OperatableStateMachine.add('target_reached',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:396 y:109
			OperatableStateMachine.add('Synchro Master',
										self.use_behavior(SynchroMasterSM, 'Synchro Master',
											parameters={'Change_depth': True, 'Max_distance_to_surface': 0.5, 'Max_distance_to_bottom': 2, 'Difference_between_sub': 0.75}),
										transitions={'finished': 'move', 'failed': 'failed', 'timeout': 'failed', 'failed_target_reached': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'failed_target_reached': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
