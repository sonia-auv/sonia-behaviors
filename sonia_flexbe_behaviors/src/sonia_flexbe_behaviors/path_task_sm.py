#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.vision_path_new_algo_sm import vision_path_new_algoSM
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 16 2021
@author: William Brouillard
'''
class path_taskSM(Behavior):
	'''
	Execute the detection of path and move to the next obstacle.
	'''


	def __init__(self):
		super(path_taskSM, self).__init__()
		self.name = 'path_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(vision_path_new_algoSM, 'vision_path_new_algo')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:735 y:71, x:312 y:287, x:165 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:256 y:54
			OperatableStateMachine.add('vision_path_new_algo',
										self.use_behavior(vision_path_new_algoSM, 'vision_path_new_algo'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:776 y:265
			OperatableStateMachine.add('pose_buffer',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'buffer_pose'})

			# x:515 y:209
			OperatableStateMachine.add('move_buffer',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'buffer_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
