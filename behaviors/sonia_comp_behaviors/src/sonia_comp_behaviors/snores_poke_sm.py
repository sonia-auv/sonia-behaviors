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
from sonia_comp_behaviors.snores_test_sm import SnorestestSM
from sonia_vision_states.choose_target import choose_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 06 2023
@author: Nimai
'''
class SnorespokeSM(Behavior):
	'''
	pokie pokie
	'''


	def __init__(self):
		super(SnorespokeSM, self).__init__()
		self.name = 'Snores poke'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2')
		self.add_behavior(SnorestestSM, 'Snores test')
		self.add_behavior(SnorestestSM, 'Snores test_2')
		self.add_behavior(SnorestestSM, 'Snores test_2_2')
		self.add_behavior(SnorestestSM, 'Snores test_2_2_2')
		self.add_behavior(SnorestestSM, 'Snores test_3')
		self.add_behavior(SnorestestSM, 'Snores test_3_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:629 y:390, x:349 y:198
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['gate_side'])
		_state_machine.userdata.gate_side = "Earth"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:50 y:27
			OperatableStateMachine.add('choose',
										choose_target(nb_img=10),
										transitions={'earth': 'Snores test', 'abydos': 'Snores test_3', 'fail': 'Snores test_3_2'},
										autonomy={'earth': Autonomy.Off, 'abydos': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'gate_side': 'gate_side'})

			# x:85 y:294
			OperatableStateMachine.add('Single Pose Move_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2'),
										transitions={'finished': 'Snores test_2_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:278 y:502
			OperatableStateMachine.add('Single Pose Move_2_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2'),
										transitions={'finished': 'Snores test_2_2_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:277 y:50
			OperatableStateMachine.add('Snores test',
										self.use_behavior(SnorestestSM, 'Snores test',
											parameters={'target': "Glyph_Earth_1"}),
										transitions={'finished': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:509 y:224
			OperatableStateMachine.add('Snores test_2',
										self.use_behavior(SnorestestSM, 'Snores test_2',
											parameters={'target': "Glyph_Earth_2"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:97 y:413
			OperatableStateMachine.add('Snores test_2_2',
										self.use_behavior(SnorestestSM, 'Snores test_2_2',
											parameters={'target': "Glyph_Abydos_2"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:560 y:476
			OperatableStateMachine.add('Snores test_2_2_2',
										self.use_behavior(SnorestestSM, 'Snores test_2_2_2',
											parameters={'target': "Glyph_Earth_2"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:85 y:170
			OperatableStateMachine.add('Snores test_3',
										self.use_behavior(SnorestestSM, 'Snores test_3',
											parameters={'target': "Glyph_Abydos_1"}),
										transitions={'finished': 'Single Pose Move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:0 y:517
			OperatableStateMachine.add('Snores test_3_2',
										self.use_behavior(SnorestestSM, 'Snores test_3_2',
											parameters={'target': "Glyph_Abydos_1"}),
										transitions={'finished': 'Single Pose Move_2_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:554 y:54
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move'),
										transitions={'finished': 'Snores test_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
