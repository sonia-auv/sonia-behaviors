#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_vision_states.vision_alignment_check import vision_alignemnt_check
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 09 2023
@author: Nimai
'''
class TestalignSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(TestalignSM, self).__init__()
		self.name = 'Test align'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:130 y:45
			OperatableStateMachine.add('Align',
										vision_alignemnt_check(filterchain_obj_topic=/proc_image_processing/gate_target, filterchain_box_topic=/proc_image_processing/gate_box, blob_size=130, nb_imgs=10, timeout_sec=5, max_adjusts=10, tolerance=0.05),
										transitions={'timeout': 'failed', 'success': 'finished', 'failed': 'failed'},
										autonomy={'timeout': Autonomy.Off, 'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
