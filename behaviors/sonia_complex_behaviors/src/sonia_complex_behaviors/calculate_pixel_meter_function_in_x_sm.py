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
		self.add_behavior(SinglePoseMoveSM, 'calc_group/Move Backwards')
		self.add_behavior(SinglePoseMoveSM, 'calc_group/return to origin')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:557 y:382, x:449 y:174
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['func_block'])
		_state_machine.userdata.func_block = func_block

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_calc_group_0 = OperatableStateMachine(outcomes=['failed', 'finished'], output_keys=['calc_block'])

		with _sm_calc_group_0:
			# x:30 y:40
			OperatableStateMachine.add('init the calculaiton block',
										init_blob_calc_block(),
										transitions={'success': 'origin'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:286 y:248
			OperatableStateMachine.add('move1',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.1, nb_img=10, direction=0),
										transitions={'success': 'return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:311 y:43
			OperatableStateMachine.add('origin',
										get_blob_size(filterchain_obj_topic=self.gate_obj_topic, dist_from_origin=0.0, nb_img=10, direction=0),
										transitions={'success': 'Move Backwards', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:291 y:372
			OperatableStateMachine.add('return to origin',
										self.use_behavior(SinglePoseMoveSM, 'calc_group/return to origin',
											parameters={'positionX': 0.3}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:422 y:134
			OperatableStateMachine.add('Move Backwards',
										self.use_behavior(SinglePoseMoveSM, 'calc_group/Move Backwards',
											parameters={'positionX': -0.3}),
										transitions={'finished': 'move1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:71 y:45
			OperatableStateMachine.add('calc_group',
										_sm_calc_group_0,
										transitions={'failed': 'Calculate', 'finished': 'Calculate'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'calc_block': 'calc_block'})

			# x:289 y:409
			OperatableStateMachine.add('Calculate',
										calc_pixel_meter_ratio(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block', 'func_block': 'func_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
