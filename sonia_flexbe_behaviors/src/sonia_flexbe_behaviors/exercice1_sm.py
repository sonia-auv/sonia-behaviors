#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 09 2022
@author: Ewan
'''
class Exercice1SM(Behavior):
	'''
	Exercice 1 du control du auv8
	'''


	def __init__(self):
		super(Exercice1SM, self).__init__()
		self.name = 'Exercice1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:225 y:47
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'dess1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:398 y:327
			OperatableStateMachine.add('av2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=3, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'tour2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory4', 'trajectory': 'trajectory5'})

			# x:411 y:40
			OperatableStateMachine.add('dess1',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=1, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'tour1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory2'})

			# x:423 y:531
			OperatableStateMachine.add('ret1',
										manual_add_pose_to_trajectory(positionX=2, positionY=3, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'trick1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory6', 'trajectory': 'trajectory7'})

			# x:201 y:515
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory9'})

			# x:408 y:135
			OperatableStateMachine.add('tour1',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'av1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory2', 'trajectory': 'trajectory3'})

			# x:390 y:438
			OperatableStateMachine.add('tour2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=180, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'ret1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory5', 'trajectory': 'trajectory6'})

			# x:655 y:532
			OperatableStateMachine.add('trick1',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=180, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'trick2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory7', 'trajectory': 'trajectory8'})

			# x:517 y:661
			OperatableStateMachine.add('trick2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory8', 'trajectory': 'trajectory9'})

			# x:34 y:511
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:400 y:229
			OperatableStateMachine.add('av1',
										manual_add_pose_to_trajectory(positionX=2, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'av2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory3', 'trajectory': 'trajectory4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
