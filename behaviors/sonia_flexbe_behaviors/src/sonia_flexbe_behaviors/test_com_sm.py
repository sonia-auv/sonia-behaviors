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
from sonia_com_states.get_update import get_update
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jun 25 2023
@author: Ewan F
'''
class test_comSM(Behavior):
	'''
	test de communication intersub
	'''


	def __init__(self):
		super(test_comSM, self).__init__()
		self.name = 'test_com'

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

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('intersub', 'continue')]),
										('failed', [('w5', 'done')])
										])

		with _sm_container_0:
			# x:151 y:99
			OperatableStateMachine.add('intersub',
										get_update(has_com=True),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'mission_array': 'mission_array'})

			# x:378 y:199
			OperatableStateMachine.add('w5',
										WaitState(wait_time=30),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:192 y:84
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'Container', 'failed': 'inti'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:310 y:164
			OperatableStateMachine.add('inti',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:327 y:313
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:297 y:229
			OperatableStateMachine.add('pose',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.2, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
