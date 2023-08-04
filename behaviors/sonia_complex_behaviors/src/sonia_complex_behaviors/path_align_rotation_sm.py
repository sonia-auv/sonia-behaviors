#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.single_pose_move_sm import SinglePoseMoveSM
from sonia_vision_states.get_pipe_angle import get_pipe_angle
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 03 2023
@author: Nimai
'''
class PathAlignRotationSM(Behavior):
	'''
	Path Align Rotation
	'''


	def __init__(self):
		super(PathAlignRotationSM, self).__init__()
		self.name = 'Path Align Rotation'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:329 y:384, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:145 y:46
			OperatableStateMachine.add('Get pipe angle',
										get_pipe_angle(num_imgs=10),
										transitions={'Success': 'Single Pose Move', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'angle_average': 'angle_average'})

			# x:395 y:66
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
