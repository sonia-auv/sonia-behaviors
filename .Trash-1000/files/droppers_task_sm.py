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
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_states.create_absolute_depth import create_absolute_depth
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William Brouillard
'''
class droppers_taskSM(Behavior):
	'''
	Find the bin with the right filter chain, then drop the markers in the bin.
	'''


	def __init__(self):
		super(droppers_taskSM, self).__init__()
		self.name = 'droppers_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(droppersSM, 'droppers')
		self.add_behavior(vision_droppersSM, 'vision_droppers')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:970 y:386, x:429 y:499, x:623 y:286
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:120 y:52
			OperatableStateMachine.add('depth',
										create_absolute_depth(positionZ=0.5),
										transitions={'continue': 'depth_move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:310 y:77
			OperatableStateMachine.add('depth_move',
										move_to_target(),
										transitions={'continue': 'vision_droppers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:948 y:199
			OperatableStateMachine.add('droppers',
										self.use_behavior(droppersSM, 'droppers'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:580 y:133
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers'),
										transitions={'finished': 'droppers', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
