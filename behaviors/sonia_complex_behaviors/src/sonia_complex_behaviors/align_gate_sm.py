#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_complex_behaviors.calculate_pixel_meter_function_in_x_sm import CalculatepixelmeterfunctioninxSM
from sonia_complex_behaviors.calculate_pixel_meter_function_in_yz_sm import CalculatepixelmeterfunctioninyzSM
from sonia_vision_states.vision_alignment_check import vision_alignemnt_check
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 21 2023
@author: Nimai
'''
class AlignGateSM(Behavior):
	'''
	align to gate
	'''


	def __init__(self):
		super(AlignGateSM, self).__init__()
		self.name = 'Align Gate'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CalculatepixelmeterfunctioninxSM, 'Calculate pixel meter function in x')
		self.add_behavior(CalculatepixelmeterfunctioninyzSM, 'Calculate pixel meter function in yz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:273 y:379, x:281 y:220
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:69 y:43
			OperatableStateMachine.add('Calculate pixel meter function in x',
										self.use_behavior(CalculatepixelmeterfunctioninxSM, 'Calculate pixel meter function in x',
											parameters={'gate_obj_topic': "/proc_image_processing/gate_left_target"}),
										transitions={'finished': 'Calculate pixel meter function in yz', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pixel_meter_func': 'x_func'})

			# x:380 y:35
			OperatableStateMachine.add('Calculate pixel meter function in yz',
										self.use_behavior(CalculatepixelmeterfunctioninyzSM, 'Calculate pixel meter function in yz',
											parameters={'gate_obj_topic': "/proc_image_processing/gate_left_target"}),
										transitions={'finished': 'Align to gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pixel_meter_func': 'yz_function'})

			# x:438 y:258
			OperatableStateMachine.add('Align to gate',
										vision_alignemnt_check(filterchain_obj_topic="/proc_image_processing/gate_left_target", filterchain_box_topic="/proc_image_processing/gate_box", blob_size=130, nb_imgs=10, timeout_sec=5, max_adjusts=10, tolerance=0.05),
										transitions={'timeout': 'failed', 'success': 'finished', 'failed': 'failed'},
										autonomy={'timeout': Autonomy.Off, 'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x_func': 'x_func', 'yz_function': 'yz_function'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
