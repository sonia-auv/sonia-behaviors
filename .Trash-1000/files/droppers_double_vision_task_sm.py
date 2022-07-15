#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.droppers_sm import droppersSM
from sonia_flexbe_behaviors.vision_double_droppers_sm import vision_double_droppersSM
from sonia_flexbe_states.create_absolute_depth import create_absolute_depth
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: FA
'''
class droppers_double_vision_taskSM(Behavior):
	'''
	Find the bin with the ai and conventionnal vision, then drop the markers in the bin.
	'''


	def __init__(self):
		super(droppers_double_vision_taskSM, self).__init__()
		self.name = 'droppers_double_vision_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(droppersSM, 'droppers')
		self.add_behavior(vision_double_droppersSM, 'vision_double_droppers')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:857 y:386, x:296 y:284, x:831 y:129
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:96 y:45
			OperatableStateMachine.add('pose depth',
										create_absolute_depth(positionZ=0.5),
										transitions={'continue': 'move depth'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:364 y:35
			OperatableStateMachine.add('move depth',
										move_to_target(),
										transitions={'continue': 'vision_double_droppers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:578 y:67
			OperatableStateMachine.add('vision_double_droppers',
										self.use_behavior(vision_double_droppersSM, 'vision_double_droppers'),
										transitions={'finished': 'droppers', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:519 y:224
			OperatableStateMachine.add('droppers',
										self.use_behavior(droppersSM, 'droppers'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
