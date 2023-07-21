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
from sonia_vision_states.calc_pixel_meter_ratio import calc_pixel_meter_ratio
from sonia_vision_states.get_blob_size import get_blob_size
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 21 2023
@author: Nimai
'''
class CalculatepixelmeterfunctioninxSM(Behavior):
	'''
	Calculate the function that represents a curve matching the scaling of pixels to missions
	'''


	def __init__(self):
		super(CalculatepixelmeterfunctioninxSM, self).__init__()
		self.name = 'Calculate pixel meter function in x'

		# parameters of this behavior
		self.add_parameter('gate_obj_topic', '')

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Move Backwards')
		self.add_behavior(SinglePoseMoveSM, 'return to origin')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:157 y:251, x:397 y:228
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['pixel_meter_func'])
		_state_machine.userdata.pixel_meter_func = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:79 y:30
			OperatableStateMachine.add('init the calculaiton block',
										init_blob_calc_block(),
										transitions={'success': 'origin'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:517 y:16
			OperatableStateMachine.add('Move Backwards',
										self.use_behavior(SinglePoseMoveSM, 'Move Backwards',
											parameters={'positionX': -0.1}),
										transitions={'finished': 'move1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:561 y:152
			OperatableStateMachine.add('move1',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.1, nb_img=10),
										transitions={'success': 'return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:303 y:27
			OperatableStateMachine.add('origin',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10),
										transitions={'success': 'Move Backwards', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:402 y:457
			OperatableStateMachine.add('return to origin',
										self.use_behavior(SinglePoseMoveSM, 'return to origin',
											parameters={'positionX': 0.1}),
										transitions={'finished': 'Calculate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:143 y:393
			OperatableStateMachine.add('Calculate',
										calc_pixel_meter_ratio(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block', 'pixel_meter_function': 'pixel_meter_func'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
