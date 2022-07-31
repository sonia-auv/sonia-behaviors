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
from sonia_mapping_states.start_bundle import start_bundle
from sonia_mapping_states.stop_hydro_bundle import stop_hydro_bundle
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 28 2022
@author: Alexandre Desgagné
'''
class hydro_end2endSM(Behavior):
	'''
	A small behavior to test the hydrophone states.
	'''


	def __init__(self):
		super(hydro_end2endSM, self).__init__()
		self.name = 'hydro_end2end'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:931 y:187, x:598 y:38
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('start_hydro',
										start_bundle(sonarBundle=False, hydroBundle=True, sonarTarget="", hydroTarget=0, resetSonarBundle=False, resetHydroBundle=False),
										transitions={'continue': 'wait'},
										autonomy={'continue': Autonomy.Off})

			# x:437 y:227
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'moving'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory1'})

			# x:697 y:252
			OperatableStateMachine.add('moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait_target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:210 y:229
			OperatableStateMachine.add('set_target',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=-2, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=16, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory1'})

			# x:380 y:40
			OperatableStateMachine.add('stop_hydro',
										stop_hydro_bundle(hydroObstacleID=5, resetHydroBundle=False),
										transitions={'found': 'init_traj', 'not_found': 'failed', 'time_out': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off, 'time_out': Autonomy.Off})

			# x:231 y:41
			OperatableStateMachine.add('wait',
										WaitState(wait_time=30),
										transitions={'done': 'stop_hydro'},
										autonomy={'done': Autonomy.Off})

			# x:689 y:32
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:23 y:231
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'set_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
