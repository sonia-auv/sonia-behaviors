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
@author: William Brouillard
'''
class coin_flipSM(Behavior):
	'''
	Orient to gate for coin flip task.
	'''


	def __init__(self):
		super(coin_flipSM, self).__init__()
		self.name = 'coin_flip'

		# parameters of this behavior
		self.add_parameter('orientation_to_gate', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:449 y:370, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:128 y:61
			OperatableStateMachine.add('pose_depth',
										create_pose(positionX=0, positionY=0, positionZ=1.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=15, precision=0, rotation=True),
										transitions={'continue': 'pose_gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:535 y:204
			OperatableStateMachine.add('Turn_gate',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'gate_pose'})

			# x:333 y:68
			OperatableStateMachine.add('pose_gate',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=self.orientation_to_gate, frame=2, time=5, precision=0, rotation=True),
										transitions={'continue': 'Depth'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'gate_pose'})

			# x:247 y:198
			OperatableStateMachine.add('Depth',
										move_to_target(),
										transitions={'continue': 'Turn_gate', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'depth_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
