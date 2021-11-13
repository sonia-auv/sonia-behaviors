#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 12 2021
@author: FA
'''
class test_loop_moveSM(Behavior):
	'''
	Testing a multiple use of the same pose
	'''


	def __init__(self):
		super(test_loop_moveSM, self).__init__()
		self.name = 'test_loop_move'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1055 y:184, x:503 y:413
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:115 y:54
			OperatableStateMachine.add('forward',
										create_pose(positionX=5, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=15, precision=0, rotation=True),
										transitions={'continue': 'turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:423 y:176
			OperatableStateMachine.add('move_1',
										move_to_target(),
										transitions={'continue': 'move_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:714 y:170
			OperatableStateMachine.add('move_2',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:119 y:196
			OperatableStateMachine.add('turn',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=90, frame=1, time=2, precision=0, rotation=True),
										transitions={'continue': 'move_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'turn_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
