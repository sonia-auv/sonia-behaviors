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
from sonia_vision_states.get_target_angle import get_target_angle
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 28 2023
@author: Nimai
'''
class AlignGateAngleSM(Behavior):
	'''
	Align the gate perpendiculary
	'''


	def __init__(self):
		super(AlignGateAngleSM, self).__init__()
		self.name = 'Align Gate Angle'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:702 y:399, x:160 y:502
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Init calc block',
										init_blob_calc_block(),
										transitions={'success': 'Get target angle'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:338 y:235
			OperatableStateMachine.add('Get target angle_2',
										get_target_angle(filterchain_obj_topic="proc_image_processing/gate_angle", obj_ratio=10, nb_img=10),
										transitions={'success': 'Single Pose Move_2', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:317 y:116
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 10}),
										transitions={'finished': 'Get target angle_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:621 y:227
			OperatableStateMachine.add('Single Pose Move_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': -10}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:119 y:169
			OperatableStateMachine.add('Get target angle',
										get_target_angle(filterchain_obj_topic="proc_image_processing/gate_angle", obj_ratio=10, nb_img=10),
										transitions={'success': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
