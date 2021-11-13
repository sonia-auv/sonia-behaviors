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
from sonia_flexbe_states.move_single import move_single
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 12 2021
@author: William Brouillard
'''
class Move_Square_5mSM(Behavior):
	'''
	Moving in a square pattern of 5 m.
Turn 90 degree anticlockwise then move 5m.
	'''


	def __init__(self):
		super(Move_Square_5mSM, self).__init__()
		self.name = 'Move_Square_5m'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:695 y:564, x:748 y:464
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:135 y:99
			OperatableStateMachine.add('depth',
										move_single(positionX=0, positionY=0, positionZ=2, orientationX=0, orientationY=0, orientationZ=0, frame=0, time=5, precision=0, rotation=True),
										transitions={'continue': 'Pose_Forward', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:921 y:239
			OperatableStateMachine.add('Forward_2',
										move_to_target(),
										transitions={'continue': 'Turn_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:1105 y:341
			OperatableStateMachine.add('Forward_3',
										move_to_target(),
										transitions={'continue': 'Turn_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:1107 y:577
			OperatableStateMachine.add('Forward_4',
										move_to_target(),
										transitions={'continue': 'Turn_4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:343 y:104
			OperatableStateMachine.add('Pose_Forward',
										create_pose(positionX=5, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=15, precision=0, rotation=True),
										transitions={'continue': 'Pose_Turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'forward_pose'})

			# x:555 y:109
			OperatableStateMachine.add('Pose_Turn',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=90, frame=1, time=2, precision=0, rotation=True),
										transitions={'continue': 'Forward_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:724 y:239
			OperatableStateMachine.add('Turn_1',
										move_to_target(),
										transitions={'continue': 'Forward_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:1105 y:240
			OperatableStateMachine.add('Turn_2',
										move_to_target(),
										transitions={'continue': 'Forward_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:1107 y:458
			OperatableStateMachine.add('Turn_3',
										move_to_target(),
										transitions={'continue': 'Forward_4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:1110 y:680
			OperatableStateMachine.add('Turn_4',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'turn_pose'})

			# x:536 y:239
			OperatableStateMachine.add('Forward_1',
										move_to_target(),
										transitions={'continue': 'Turn_1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
