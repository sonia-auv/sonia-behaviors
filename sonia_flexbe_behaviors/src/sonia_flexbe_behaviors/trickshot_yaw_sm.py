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
Created on Sun Nov 21 2021
@author: FA
'''
class trickshotyawSM(Behavior):
	'''
	Yaw trickshot for the gate
	'''


	def __init__(self):
		super(trickshotyawSM, self).__init__()
		self.name = 'trickshot yaw'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:796 y:384, x:476 y:331
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:107 y:68
			OperatableStateMachine.add('rotation',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, time=8, precision=0, rotation=True),
										transitions={'continue': 'rotate 1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'rotation'})

			# x:698 y:146
			OperatableStateMachine.add('rotate 2',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation'})

			# x:398 y:97
			OperatableStateMachine.add('rotate 1',
										move_to_target(),
										transitions={'continue': 'rotate 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
