#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.launch_auv8_sm import launch_AUV8SM
from sonia_flexbe_behaviors.vision_torpedoes_sm import vision_torpedoesSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: GS
'''
class torpedoesSM(Behavior):
	'''
	align and shoot
	'''


	def __init__(self):
		super(torpedoesSM, self).__init__()
		self.name = 'torpedoes'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(launch_AUV8SM, 'launch_AUV8')
		self.add_behavior(vision_torpedoesSM, 'vision_torpedoes')

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
			# x:30 y:40
			OperatableStateMachine.add('vision_torpedoes',
										self.use_behavior(vision_torpedoesSM, 'vision_torpedoes'),
										transitions={'finished': 'launch_AUV8', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:200 y:208
			OperatableStateMachine.add('launch_AUV8',
										self.use_behavior(launch_AUV8SM, 'launch_AUV8'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
