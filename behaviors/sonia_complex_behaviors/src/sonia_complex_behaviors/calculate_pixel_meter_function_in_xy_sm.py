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
from sonia_vision_states.calculate_function_constants import calculate_function_constants
from sonia_vision_states.get_blob_size import get_blob_size
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 21 2023
@author: Nimai
'''
class CalculatepixelmeterfunctioninxySM(Behavior):
	'''
	Calculate the function that represents a curve matching the scaling of pixels to missions
	'''


	def __init__(self):
		super(CalculatepixelmeterfunctioninxySM, self).__init__()
		self.name = 'Calculate pixel meter function in xy'

		# parameters of this behavior
		self.add_parameter('gate_obj_topic', '')

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Move Right')
		self.add_behavior(SinglePoseMoveSM, 'return to origin')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:22 y:566, x:56 y:349
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['func_block'])
		_state_machine.userdata.func_block = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:48 y:48
			OperatableStateMachine.add('init the calculaiton block',
										init_blob_calc_block(),
										transitions={'success': 'origin'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:282 y:158
			OperatableStateMachine.add('Move Right',
										self.use_behavior(SinglePoseMoveSM, 'Move Right',
											parameters={'positionX': 0.0, 'positionY': 0.3, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'move1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:288 y:299
			OperatableStateMachine.add('move1',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.3, nb_img=10, direction=1),
										transitions={'success': 'return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:271 y:47
			OperatableStateMachine.add('origin',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10, direction=1),
										transitions={'success': 'Move Right', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:431 y:432
			OperatableStateMachine.add('return to origin',
										self.use_behavior(SinglePoseMoveSM, 'return to origin',
											parameters={'positionX': 0.0, 'positionY': -0.3, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'Calculate 1st order function', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:177 y:503
			OperatableStateMachine.add('Calculate 1st order function',
										calculate_function_constants(order=1, precision=3),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block', 'func_block': 'func_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
