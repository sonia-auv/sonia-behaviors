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
		self.add_behavior(SinglePoseMoveSM, 'Move forward')
		self.add_behavior(SinglePoseMoveSM, 'Move to origin (1)')
		self.add_behavior(SinglePoseMoveSM, 'return to origin')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:144 y:806, x:164 y:641
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['func_block'])
		_state_machine.userdata.func_block = []
		_state_machine.userdata.calc_block = calc_block

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:54
			OperatableStateMachine.add('init the calculaiton block',
										init_blob_calc_block(),
										transitions={'success': 'origin'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:519 y:367
			OperatableStateMachine.add('Get blob size origin (2)',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10, direction=0),
										transitions={'success': 'Move forward', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:287 y:239
			OperatableStateMachine.add('Get blob size pt 1',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.1, nb_img=10, direction=0),
										transitions={'success': 'Move to origin (1)', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:740 y:515
			OperatableStateMachine.add('Get blob size pt 2',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=-0.1, nb_img=10, direction=0),
										transitions={'success': 'return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:258 y:118
			OperatableStateMachine.add('Move Backwards',
										self.use_behavior(SinglePoseMoveSM, 'Move Backwards',
											parameters={'positionX': -0.3, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'Get blob size pt 1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:763 y:360
			OperatableStateMachine.add('Move forward',
										self.use_behavior(SinglePoseMoveSM, 'Move forward',
											parameters={'positionX': 0.3, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'Get blob size pt 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:512 y:248
			OperatableStateMachine.add('Move to origin (1)',
										self.use_behavior(SinglePoseMoveSM, 'Move to origin (1)',
											parameters={'positionX': 0.3, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'Get blob size origin (2)', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:72 y:122
			OperatableStateMachine.add('origin',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10, direction=0),
										transitions={'success': 'Move Backwards', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:679 y:697
			OperatableStateMachine.add('return to origin',
										self.use_behavior(SinglePoseMoveSM, 'return to origin',
											parameters={'positionX': 0.3}),
										transitions={'finished': 'Calculate 2nd order', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:269 y:763
			OperatableStateMachine.add('Calculate 2nd order',
										calculate_function_constants(order=2, precision=3),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block', 'func_block': 'func_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
