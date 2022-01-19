#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 20 2021
@author: William Brouillard
'''
class squarry_circleSM(Behavior):
	'''
	Move in a squarry circle pattern.
	'''


	def __init__(self):
		super(squarry_circleSM, self).__init__()
		self.name = 'squarry_circle'

		# parameters of this behavior
		self.add_parameter('distance', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:45 y:399, x:416 y:343
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:26 y:34
			OperatableStateMachine.add('log_start',
										LogState(text='Starting circle_search', severity=2),
										transitions={'done': 'pose_move_1'},
										autonomy={'done': Autonomy.Off})

			# x:135 y:77
			OperatableStateMachine.add('pose_move_1',
										create_pose(positionX=self.distance, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.distance*5, precision=0, rotation=True),
										transitions={'continue': 'pose_move_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'distance_1'})

			# x:373 y:33
			OperatableStateMachine.add('pose_move_2',
										create_pose(positionX=self.distance*2, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.distance*10, precision=0, rotation=True),
										transitions={'continue': 'pose_move_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'distance_2'})

			# x:599 y:21
			OperatableStateMachine.add('pose_move_3',
										create_pose(positionX=self.distance*3, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.distance*15, precision=0, rotation=True),
										transitions={'continue': 'pose_rotate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'distance_3'})

			# x:802 y:73
			OperatableStateMachine.add('pose_rotate',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=90, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'segment_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:320 y:168
			OperatableStateMachine.add('rotation_1',
										move_to_target(),
										transitions={'continue': 'segment_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:775 y:165
			OperatableStateMachine.add('rotation_2',
										move_to_target(),
										transitions={'continue': 'segment_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:973 y:264
			OperatableStateMachine.add('rotation_3',
										move_to_target(),
										transitions={'continue': 'segment_4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:910 y:511
			OperatableStateMachine.add('rotation_4',
										move_to_target(),
										transitions={'continue': 'segment_5', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:672 y:651
			OperatableStateMachine.add('rotation_5',
										move_to_target(),
										transitions={'continue': 'segment_6', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:309 y:648
			OperatableStateMachine.add('rotation_6',
										move_to_target(),
										transitions={'continue': 'segment_7', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'rotation_pose'})

			# x:100 y:153
			OperatableStateMachine.add('segment_1',
										move_to_target(),
										transitions={'continue': 'rotation_1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_1'})

			# x:546 y:168
			OperatableStateMachine.add('segment_2',
										move_to_target(),
										transitions={'continue': 'rotation_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_1'})

			# x:995 y:171
			OperatableStateMachine.add('segment_3',
										move_to_target(),
										transitions={'continue': 'rotation_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_1'})

			# x:952 y:396
			OperatableStateMachine.add('segment_4',
										move_to_target(),
										transitions={'continue': 'rotation_4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_2'})

			# x:902 y:665
			OperatableStateMachine.add('segment_5',
										move_to_target(),
										transitions={'continue': 'rotation_5', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_2'})

			# x:483 y:655
			OperatableStateMachine.add('segment_6',
										move_to_target(),
										transitions={'continue': 'rotation_6', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_3'})

			# x:118 y:669
			OperatableStateMachine.add('segment_7',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'distance_3'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
