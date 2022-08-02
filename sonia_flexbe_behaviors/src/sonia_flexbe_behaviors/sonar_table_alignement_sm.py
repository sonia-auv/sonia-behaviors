#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from sonia_mapping_states.sonar_config import sonar_config
from sonia_mapping_states.start_bundle import start_bundle
from sonia_mapping_states.start_stop_sonar import start_stop_sonar
from sonia_mapping_states.stop_sonar_bundle import stop_sonar_bundle
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 31 2022
@author: Alexandre Desgagné
'''
class sonar_table_alignementSM(Behavior):
	'''
	Alignement with the tables under the octogon.
	'''


	def __init__(self):
		super(sonar_table_alignementSM, self).__init__()
		self.name = 'sonar_table_alignement'

		# parameters of this behavior
		self.add_parameter('tables_depth', 2.6)
		self.add_parameter('move_y_for_table', 3.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:903 y:143, x:535 y:127
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:23
			OperatableStateMachine.add('torpido_depth_init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'back_up'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:37 y:333
			OperatableStateMachine.add('depth',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=self.tables_depth, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=4, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send_trajectory'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory3', 'trajectory': 'trajectory4'})

			# x:38 y:252
			OperatableStateMachine.add('go_left',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=self.move_y_for_table, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'depth'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory2', 'trajectory': 'trajectory3'})

			# x:483 y:251
			OperatableStateMachine.add('go_octogon_init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move_to_octogon'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory5'})

			# x:467 y:316
			OperatableStateMachine.add('move_to_octogon',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=17, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send_traj'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory5', 'trajectory': 'trajectory6'})

			# x:510 y:28
			OperatableStateMachine.add('moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'start_sonar', 'moving': 'wait_target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:699 y:57
			OperatableStateMachine.add('moving_octogon',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'soft_kill', 'moving': 'wait_target_octogon', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:36 y:170
			OperatableStateMachine.add('rotate',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=30, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=2, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_left'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory1', 'trajectory': 'trajectory2'})

			# x:477 y:402
			OperatableStateMachine.add('send_traj',
										send_to_planner(),
										transitions={'continue': 'wait_target_octogon', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory6'})

			# x:48 y:407
			OperatableStateMachine.add('send_trajectory',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory4'})

			# x:299 y:253
			OperatableStateMachine.add('set_config',
										sonar_config(gain=26, range=7),
										transitions={'continue': 'start_bundle'},
										autonomy={'continue': Autonomy.Off})

			# x:701 y:375
			OperatableStateMachine.add('set_mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'stop_sonar', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:708 y:203
			OperatableStateMachine.add('soft_kill',
										set_control_mode(mode=0, timeout=5),
										transitions={'continue': 'wait_surface', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:299 y:312
			OperatableStateMachine.add('start_bundle',
										start_bundle(sonarBundle=True, hydroBundle=False, sonarTarget='Tables', hydroTarget=20, resetSonarBundle=False, resetHydroBundle=False),
										transitions={'continue': 'wait'},
										autonomy={'continue': Autonomy.Off})

			# x:301 y:194
			OperatableStateMachine.add('start_sonar',
										start_stop_sonar(startStop=True),
										transitions={'continue': 'set_config'},
										autonomy={'continue': Autonomy.Off})

			# x:487 y:188
			OperatableStateMachine.add('stop_bundle',
										stop_sonar_bundle(sonarObstacleID=6, resetSonarBundle=False),
										transitions={'found': 'go_octogon_init', 'not_found': 'finished', 'time_out': 'finished'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off, 'time_out': Autonomy.Off})

			# x:706 y:467
			OperatableStateMachine.add('stop_sonar',
										start_stop_sonar(startStop=False),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:313 y:382
			OperatableStateMachine.add('wait',
										WaitState(wait_time=3),
										transitions={'done': 'stop_bundle'},
										autonomy={'done': Autonomy.Off})

			# x:726 y:291
			OperatableStateMachine.add('wait_surface',
										WaitState(wait_time=5),
										transitions={'done': 'set_mode'},
										autonomy={'done': Autonomy.Off})

			# x:305 y:25
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'start_sonar', 'target_not_reached': 'moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:478 y:482
			OperatableStateMachine.add('wait_target_octogon',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'soft_kill', 'target_not_reached': 'moving_octogon', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:38 y:96
			OperatableStateMachine.add('back_up',
										manual_add_pose_to_trajectory(positionX=-5, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
