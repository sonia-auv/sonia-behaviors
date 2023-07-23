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
class CalculatepixelmeterfunctioninyzSM(Behavior):
	'''
	Calculate the function that represents a curve matching the scaling of pixels to missions
	'''


	def __init__(self):
		super(CalculatepixelmeterfunctioninyzSM, self).__init__()
		self.name = 'Calculate pixel meter function in yz'

		# parameters of this behavior
		self.add_parameter('gate_obj_topic', '')

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Calc_block/Move Right')
		self.add_behavior(SinglePoseMoveSM, 'Calc_block/return to origin')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:454 y:260, x:335 y:182
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['func_block'])
		_state_machine.userdata.func_block = func_block

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:152 y:590
		_sm_calc_block_0 = OperatableStateMachine(outcomes=['failed', 'finished'], output_keys=['calc_block'])

		with _sm_calc_block_0:
			# x:30 y:55
			OperatableStateMachine.add('init the calculaiton block',
										init_blob_calc_block(),
										transitions={'success': 'origin'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:288 y:299
			OperatableStateMachine.add('move1',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=-0.1, nb_img=10, direction=1),
										transitions={'success': 'return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:220 y:40
			OperatableStateMachine.add('origin',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10, direction=1),
										transitions={'success': 'Move Right', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:93 y:470
			OperatableStateMachine.add('return to origin',
										self.use_behavior(SinglePoseMoveSM, 'Calc_block/return to origin',
											parameters={'positionY': -0.3}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:229 y:142
			OperatableStateMachine.add('Move Right',
										self.use_behavior(SinglePoseMoveSM, 'Calc_block/Move Right',
											parameters={'positionY': 0.3}),
										transitions={'finished': 'move1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:83 y:60
			OperatableStateMachine.add('Calc_block',
										_sm_calc_block_0,
										transitions={'failed': 'Calculate', 'finished': 'Calculate'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'calc_block': 'calc_block'})

			# x:229 y:380
			OperatableStateMachine.add('Calculate',
										calc_pixel_meter_ratio(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block', 'func_block': 'func_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
