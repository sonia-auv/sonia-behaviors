#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.save_pose_sm import SavePoseSM
from sonia_complex_behaviors.align_gate_angle_sm import AlignGateAngleSM
from sonia_complex_behaviors.align_gate_left_sm import AlignGateLeftSM
from sonia_complex_behaviors.align_gate_right_sm import AlignGateRightSM
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 01 2023
@author: Nimai and Ewan
'''
class GateFull2023SM(Behavior):
	'''
	Full gate track including translation align and rotation align
	'''


	def __init__(self):
		super(GateFull2023SM, self).__init__()
		self.name = 'Gate Full 2023'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(AlignGateAngleSM, 'Align Gate Angle')
		self.add_behavior(AlignGateLeftSM, 'Align Gate Left')
		self.add_behavior(AlignGateRightSM, 'Align Gate Right')
		self.add_behavior(SavePoseSM, 'Save Pose')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:957 y:324, x:159 y:581
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:80
			OperatableStateMachine.add('Save Pose',
										self.use_behavior(SavePoseSM, 'Save Pose'),
										transitions={'finished': 'Align Gate Right', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'save_traj': 'save_traj'})

			# x:382 y:394
			OperatableStateMachine.add('Align Gate Left',
										self.use_behavior(AlignGateLeftSM, 'Align Gate Left'),
										transitions={'finished': 'Align Gate Angle', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:291 y:53
			OperatableStateMachine.add('Align Gate Right',
										self.use_behavior(AlignGateRightSM, 'Align Gate Right'),
										transitions={'finished': 'Align Gate Angle', 'failed': 'Reset pos'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:283 y:170
			OperatableStateMachine.add('Reset pos',
										send_to_planner(),
										transitions={'continue': 'wait reset', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'save_traj'})

			# x:286 y:278
			OperatableStateMachine.add('wait reset',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Align Gate Left', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:747 y:386
			OperatableStateMachine.add('Align Gate Angle',
										self.use_behavior(AlignGateAngleSM, 'Align Gate Angle'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
