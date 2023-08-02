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
from sonia_vision_states.detect_gate import detect_gate
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 01 2023
@author: Nimai
'''
class CoinfliprotateandfindgateSM(Behavior):
	'''
	Rotate in steps of 30 and see if we can find the gate.
	'''


	def __init__(self):
		super(CoinfliprotateandfindgateSM, self).__init__()
		self.name = 'Coinflip rotate and find gate'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Move Left')
		self.add_behavior(SinglePoseMoveSM, 'Move Right')
		self.add_behavior(SinglePoseMoveSM, 'rotate 30')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:850 y:56, x:130 y:365, x:924 y:189, x:626 y:285
		_state_machine = OperatableStateMachine(outcomes=['found_both', 'failed', 'found_left', 'found_right'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:109
			OperatableStateMachine.add('calc_block',
										init_blob_calc_block(),
										transitions={'success': 'rotate 30'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:500 y:185
			OperatableStateMachine.add('Move Left',
										self.use_behavior(SinglePoseMoveSM, 'Move Left',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': -15}),
										transitions={'finished': 'found_right', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:792 y:331
			OperatableStateMachine.add('Move Right',
										self.use_behavior(SinglePoseMoveSM, 'Move Right',
											parameters={'positionX': 0.0, 'positionY': 0, 'positionZ': 0.0, 'orientationZ': 15}),
										transitions={'finished': 'found_left', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:229 y:60
			OperatableStateMachine.add('rotate 30',
										self.use_behavior(SinglePoseMoveSM, 'rotate 30',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 30}),
										transitions={'finished': 'Detect Gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:530 y:43
			OperatableStateMachine.add('Detect Gate',
										detect_gate(),
										transitions={'left_found': 'Move Right', 'right_found': 'Move Left', 'both_found': 'found_both', 'none_found': 'rotate 30', 'failed': 'failed'},
										autonomy={'left_found': Autonomy.Off, 'right_found': Autonomy.Off, 'both_found': Autonomy.Off, 'none_found': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
