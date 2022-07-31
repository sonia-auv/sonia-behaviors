#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_mapping_states.sonar_config import sonar_config
from sonia_mapping_states.start_bundle import start_bundle
from sonia_mapping_states.start_stop_sonar import start_stop_sonar
from sonia_mapping_states.stop_sonar_bundle import stop_sonar_bundle
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
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
		self.add_parameter('simulation', False)
		self.add_parameter('dist_to_torpedoes_board', 1)
		self.add_parameter('y_offset', 0.4)

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
			# x:8 y:70
			OperatableStateMachine.add('start sonar',
										start_stop_sonar(startStop=True),
										transitions={'continue': 'simulation'},
										autonomy={'continue': Autonomy.Off})

			# x:728 y:91
			OperatableStateMachine.add('down',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=2, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'up'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:514 y:81
			OperatableStateMachine.add('init1',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'set_initial_depth'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:361 y:421
			OperatableStateMachine.add('init2',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'allign'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:1073 y:397
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'tg2', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:731 y:241
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'tg', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:184 y:163
			OperatableStateMachine.add('set_config',
										sonar_config(gain=35, range=4),
										transitions={'continue': 'start bundle'},
										autonomy={'continue': Autonomy.Off})

			# x:730 y:12
			OperatableStateMachine.add('set_initial_depth',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=1, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=4, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:187 y:38
			OperatableStateMachine.add('simulation',
										activate_behavior(activate=self.simulation),
										transitions={'activate': 'start bundle', 'desactivate': 'set_config'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:694 y:540
			OperatableStateMachine.add('sned2',
										send_to_planner(),
										transitions={'continue': 'tg2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:336 y:81
			OperatableStateMachine.add('start bundle',
										start_bundle(sonarBundle=True, hydroBundle=False, sonarTarget='Buoys', hydroTarget=20, resetSonarBundle=True, resetHydroBundle=False),
										transitions={'continue': 'init1'},
										autonomy={'continue': Autonomy.Off})

			# x:569 y:421
			OperatableStateMachine.add('stop sonar',
										start_stop_sonar(startStop=False),
										transitions={'continue': 'init2'},
										autonomy={'continue': Autonomy.Off})

			# x:734 y:430
			OperatableStateMachine.add('stop sonar bundle',
										stop_sonar_bundle(sonarObstacleID=1, resetSonarBundle=False),
										transitions={'found': 'stop sonar', 'not_found': 'failed', 'time_out': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off, 'time_out': Autonomy.Off})

			# x:731 y:326
			OperatableStateMachine.add('tg',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'stop sonar bundle', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:876 y:531
			OperatableStateMachine.add('tg2',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'is_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:730 y:166
			OperatableStateMachine.add('up',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=-2.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:108 y:427
			OperatableStateMachine.add('allign',
										manual_add_pose_to_trajectory(positionX=-self.dist_to_torpedoes_board, positionY=self.y_offset, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=12, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'sned2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
