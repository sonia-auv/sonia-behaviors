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
from sonia_navigation_states.save_pose import save_pose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 24 2023
@author: Nimai
'''
class SavePoseSM(Behavior):
	'''
	Save Pose
	'''


	def __init__(self):
		super(SavePoseSM, self).__init__()
		self.name = 'Save Pose'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['save_traj'])
		_state_machine.userdata.save_traj = save_traj

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:158 y:57
			OperatableStateMachine.add('Init save traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Save Pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'save_traj'})

			# x:186 y:176
			OperatableStateMachine.add('Save Pose',
										save_pose(),
										transitions={'success': 'finished', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'input_traj': 'save_traj', 'trajectory': 'save_traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
