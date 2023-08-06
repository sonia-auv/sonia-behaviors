#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.single_pose_move_sm import SinglePoseMoveSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 06 2023
@author: Nimai
'''
class SnoresPokeSM(Behavior):
	'''
	ZzzzZzz poke ... Hehe
	'''


	def __init__(self):
		super(SnoresPokeSM, self).__init__()
		self.name = 'Snores Poke'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:475 y:259, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': 1}),
										transitions={'finished': 'Single Pose Move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:239 y:127
			OperatableStateMachine.add('Single Pose Move_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2',
											parameters={'positionX': -1}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
