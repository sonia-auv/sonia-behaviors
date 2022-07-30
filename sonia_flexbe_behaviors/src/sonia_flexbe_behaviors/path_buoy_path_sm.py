#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.path2_sm import path2SM
from sonia_flexbe_behaviors.vision_buoys_sm import vision_buoysSM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: CS
'''
class path_buoy_pathSM(Behavior):
	'''
	test for path gestion
	'''


	def __init__(self):
		super(path_buoy_pathSM, self).__init__()
		self.name = 'path_buoy_path'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(path2SM, 'path2')
		self.add_behavior(vision_buoysSM, 'vision_buoys')
		self.add_behavior(vision_pathSM, 'vision_path')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:753 y:122, x:343 y:360
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose_x = 0
		_state_machine.userdata.pose_y = 0
		_state_machine.userdata.pose_z = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path'),
										transitions={'finished': 'vision_buoys', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:247 y:31
			OperatableStateMachine.add('vision_buoys',
										self.use_behavior(vision_buoysSM, 'vision_buoys'),
										transitions={'finished': 'path2', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:531 y:138
			OperatableStateMachine.add('path2',
										self.use_behavior(path2SM, 'path2'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'angle': 'angle', 'pose_x': 'pose_x', 'pose_y': 'pose_y', 'pose_z': 'pose_z', 'camera': 'camera'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]