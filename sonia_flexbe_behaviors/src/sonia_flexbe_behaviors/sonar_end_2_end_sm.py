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
from sonia_sonar_states.start_bundle import start_bundle
from sonia_sonar_states.start_stop_sonar import start_stop_sonar
from sonia_sonar_states.stop_bundle import stop_bundle
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 09 2022
@author: team sonar
'''
class sonar_end_2_endSM(Behavior):
	'''
	sonar end 2 end
	'''


	def __init__(self):
		super(sonar_end_2_endSM, self).__init__()
		self.name = 'sonar_end_2_end'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1097 y:522, x:991 y:279
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:109 y:84
			OperatableStateMachine.add('start sonar',
										start_stop_sonar(startStop=True),
										transitions={'continue': 'start bundle'},
										autonomy={'continue': Autonomy.Off})

			# x:108 y:427
			OperatableStateMachine.add('allign',
										manual_add_pose_to_trajectory(positionX=-1.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=11, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'RAM'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:728 y:91
			OperatableStateMachine.add('down',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=2, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'up'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:514 y:81
			OperatableStateMachine.add('init1',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:361 y:421
			OperatableStateMachine.add('init2',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'allign'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:452 y:545
			OperatableStateMachine.add('retract',
										manual_add_pose_to_trajectory(positionX=-1.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=11, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'sned2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:731 y:241
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'tg', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:694 y:540
			OperatableStateMachine.add('sned2',
										send_to_planner(),
										transitions={'continue': 'tg2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:322 y:81
			OperatableStateMachine.add('start bundle',
										start_bundle(target='Buoys', resetBundle=True),
										transitions={'continue': 'init1'},
										autonomy={'continue': Autonomy.Off})

			# x:734 y:430
			OperatableStateMachine.add('stop bundle',
										stop_bundle(ObstacleID=1, resetBundle=False),
										transitions={'found': 'stop sonar', 'not_found': 'failed', 'time_out': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off, 'time_out': Autonomy.Off})

			# x:569 y:421
			OperatableStateMachine.add('stop sonar',
										start_stop_sonar(startStop=True),
										transitions={'continue': 'init2'},
										autonomy={'continue': Autonomy.Off})

			# x:731 y:326
			OperatableStateMachine.add('tg',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'stop bundle', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:876 y:531
			OperatableStateMachine.add('tg2',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:730 y:166
			OperatableStateMachine.add('up',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=-2.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:171 y:549
			OperatableStateMachine.add('RAM',
										manual_add_pose_to_trajectory(positionX=0.20, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=11, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'retract'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
