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
from sonia_hardware_states.wait_mission import wait_mission
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
		self.add_behavior(SnorestestSM, 'Snores test')
		self.add_behavior(SnorestestSM, 'Snores test_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:629 y:345, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:128
			OperatableStateMachine.add('Mission',
										wait_mission(),
										transitions={'continue': 'Snores test'},
										autonomy={'continue': Autonomy.Off})

			# x:554 y:54
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': -2.5, 'positionZ': 0.3}),
										transitions={'finished': 'Snores test_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:277 y:50
			OperatableStateMachine.add('Snores test',
										self.use_behavior(SnorestestSM, 'Snores test',
											parameters={'target': "Glyph_Earth_2"}),
										transitions={'finished': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:509 y:224
			OperatableStateMachine.add('Snores test_2',
										self.use_behavior(SnorestestSM, 'Snores test_2',
											parameters={'target': "Glyph_Abydos_2"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
