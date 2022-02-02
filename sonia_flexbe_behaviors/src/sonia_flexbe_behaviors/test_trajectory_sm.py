#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.add_pose_to_trajectory import add_pose_to_trajectory
from sonia_flexbe_states.init_trajectory import init_trajectory
from sonia_flexbe_states.send_to_planner import send_to_planner
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 02 2022
@author: FA
'''
class test_trajectorySM(Behavior):
	'''
	Test the proc planner with this engine
	'''


	def __init__(self):
		super(test_trajectorySM, self).__init__()
		self.name = 'test_trajectory'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1138 y:113, x:1124 y:224
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:94 y:78
			OperatableStateMachine.add('init',
										init_trajectory(),
										transitions={'continue': 'add pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:542 y:180
			OperatableStateMachine.add('add_pose_2',
										add_pose_to_trajectory(positionX=5, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=1, precision=0, rotation=True),
										transitions={'continue': 'send to planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory_1', 'trajectory': 'trajectory_2'})

			# x:795 y:81
			OperatableStateMachine.add('send to planner',
										send_to_planner(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory_2'})

			# x:288 y:153
			OperatableStateMachine.add('add pose',
										add_pose_to_trajectory(positionX=0, positionY=0, positionZ=2, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=1, precision=0, rotation=True),
										transitions={'continue': 'add_pose_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'trajectory_1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
