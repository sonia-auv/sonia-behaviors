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
from sonia_base_behaviors.trick_shot_pitch_roll_sm import trick_shot_pitch_rollSM
from sonia_comp_behaviors.gate_full_left_2023_sm import GateFullLeft2023SM
from sonia_comp_behaviors.gate_full_right_2023_sm import GateFullRight2023SM
from sonia_complex_behaviors.coinflip_rotate_and_find_gate_sm import CoinfliprotateandfindgateSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 01 2023
@author: Nimai
'''
class FullRun2023SM(Behavior):
	'''
	Full Path Run 2023
	'''


	def __init__(self):
		super(FullRun2023SM, self).__init__()
		self.name = 'Full Run 2023'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CoinfliprotateandfindgateSM, 'Conflip and align/Coinflip rotate and find gate')
		self.add_behavior(GateFullLeft2023SM, 'Conflip and align/Gate Full Left 2023')
		self.add_behavior(GateFullRight2023SM, 'Conflip and align/Gate Full Right 2023')
		self.add_behavior(SinglePoseMoveSM, 'Dive')
		self.add_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:840 y:398, x:326 y:264
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:413 y:219, x:654 y:160
		_sm_conflip_and_align_0 = OperatableStateMachine(outcomes=['failed', 'finished'])

		with _sm_conflip_and_align_0:
			# x:30 y:47
			OperatableStateMachine.add('Coinflip rotate and find gate',
										self.use_behavior(CoinfliprotateandfindgateSM, 'Conflip and align/Coinflip rotate and find gate'),
										transitions={'found_both': 'Gate Full Right 2023', 'failed': 'failed', 'found_left': 'Gate Full Left 2023', 'found_right': 'Gate Full Right 2023'},
										autonomy={'found_both': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'found_left': Autonomy.Inherit, 'found_right': Autonomy.Inherit})

			# x:366 y:52
			OperatableStateMachine.add('Gate Full Left 2023',
										self.use_behavior(GateFullLeft2023SM, 'Conflip and align/Gate Full Left 2023'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:320 y:358
			OperatableStateMachine.add('Gate Full Right 2023',
										self.use_behavior(GateFullRight2023SM, 'Conflip and align/Gate Full Right 2023'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Dive',
										self.use_behavior(SinglePoseMoveSM, 'Dive',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 1.5, 'orientationZ': 0.0}),
										transitions={'finished': 'Conflip and align', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:520 y:210
			OperatableStateMachine.add('trick_shot_pitch_roll',
										self.use_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:267 y:32
			OperatableStateMachine.add('Conflip and align',
										_sm_conflip_and_align_0,
										transitions={'failed': 'failed', 'finished': 'trick_shot_pitch_roll'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
