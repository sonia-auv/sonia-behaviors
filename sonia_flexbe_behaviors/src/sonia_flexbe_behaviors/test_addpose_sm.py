#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_with_addpose_msg_sm import MovewithAddposemsgSM
from sonia_flexbe_states.create_pose import create_pose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 15 2022
@author: FA
'''
class testaddposeSM(Behavior):
	'''
	test the behavior to create a trajcetory with addpose
	'''


	def __init__(self):
		super(testaddposeSM, self).__init__()
		self.name = 'test addpose'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(MovewithAddposemsgSM, 'Move with Addpose msg')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:724 y:99, x:463 y:277
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:126 y:73
			OperatableStateMachine.add('create depth',
										create_pose(positionX=2, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=0, precision=0, rotation=False),
										transitions={'continue': 'Move with Addpose msg'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:389 y:91
			OperatableStateMachine.add('Move with Addpose msg',
										self.use_behavior(MovewithAddposemsgSM, 'Move with Addpose msg'),
										transitions={'finished': 'finished', 'failed': 'failed', 'failed_target_reached': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_target_reached': Autonomy.Inherit},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
