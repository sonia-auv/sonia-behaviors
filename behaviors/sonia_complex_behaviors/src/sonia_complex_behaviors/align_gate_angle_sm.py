#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.rotate_angle_align_sm import RotateAngleAlignSM
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
		self.add_behavior(RotateAngleAlignSM, 'Rotate Angle Align')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:414 y:217, x:60 y:206
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:151 y:72
			OperatableStateMachine.add('Rotate Angle Align',
										self.use_behavior(RotateAngleAlignSM, 'Rotate Angle Align'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
