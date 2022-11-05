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
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 05 2022
@author: Alexandre Desgagné
'''
class LoopsquareSM(Behavior):
	'''
	Mission pour portes ouvertes.
	'''


	def __init__(self):
		super(LoopsquareSM, self).__init__()
		self.name = 'Loop square'

		# parameters of this behavior
		self.add_parameter('width', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:615 y:644, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:85 y:56
			OperatableStateMachine.add('control_mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:273 y:64
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'segment1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:564 y:468
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory8'})

			# x:679 y:62
			OperatableStateMachine.add('rotation1',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'segment2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory1', 'trajectory': 'trajectory2'})

			# x:679 y:170
			OperatableStateMachine.add('rotation2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'segment3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory3', 'trajectory': 'trajectory4'})

			# x:678 y:258
			OperatableStateMachine.add('rotation3',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'segment4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory5', 'trajectory': 'trajectory6'})

			# x:676 y:353
			OperatableStateMachine.add('rotation4',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory7', 'trajectory': 'trajectory8'})

			# x:468 y:62
			OperatableStateMachine.add('segment1',
										manual_add_pose_to_trajectory(positionX=self.width, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotation1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory1'})

			# x:467 y:167
			OperatableStateMachine.add('segment2',
										manual_add_pose_to_trajectory(positionX=self.width, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotation2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory2', 'trajectory': 'trajectory3'})

			# x:464 y:260
			OperatableStateMachine.add('segment3',
										manual_add_pose_to_trajectory(positionX=self.width, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotation3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory4', 'trajectory': 'trajectory5'})

			# x:467 y:356
			OperatableStateMachine.add('segment4',
										manual_add_pose_to_trajectory(positionX=self.width, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotation4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory6', 'trajectory': 'trajectory7'})

			# x:748 y:463
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'finished', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
