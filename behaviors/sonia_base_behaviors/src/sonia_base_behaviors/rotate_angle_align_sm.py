#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.rotate_around_target import rotate_around_target
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.decision_angle import decision_angle
from sonia_vision_states.get_target_angle import get_target_angle
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 31 2023
@author: Ewan F
'''
class RotateAngleAlignSM(Behavior):
	'''
	Rotate around target to align perpendicular.
	'''


	def __init__(self):
		super(RotateAngleAlignSM, self).__init__()
		self.name = 'Rotate Angle Align'

		# parameters of this behavior
		self.add_parameter('distance', 2.5)
		self.add_parameter('calc_angle', 25)
		self.add_parameter('target_ratio', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:802 y:565, x:330 y:380
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:58 y:45
			OperatableStateMachine.add('init',
										init_blob_calc_block(),
										transitions={'success': 'get_angle'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:233 y:95
			OperatableStateMachine.add('get_angle',
										get_target_angle(filterchain_obj_topic="/proc_image_processing/gate_angle", obj_ratio=self.target_ratio, nb_img=10),
										transitions={'success': 'rotate1', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:856 y:70
			OperatableStateMachine.add('get_angle(2)',
										get_target_angle(filterchain_obj_topic="/proc_image_processing/gate_angle", obj_ratio=self.target_ratio, nb_img=10),
										transitions={'success': 'decision', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:371 y:45
			OperatableStateMachine.add('rotate1',
										rotate_around_target(angle=self.calc_angle, distance=self.distance),
										transitions={'continue': 'wait_target1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:713 y:365
			OperatableStateMachine.add('rotate2',
										rotate_around_target(angle=-2, distance=self.distance),
										transitions={'continue': 'wait2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})

			# x:511 y:470
			OperatableStateMachine.add('wait2',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:595 y:56
			OperatableStateMachine.add('wait_target1',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_angle(2)', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:888 y:242
			OperatableStateMachine.add('decision',
										decision_angle(),
										transitions={'success': 'rotate2'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
