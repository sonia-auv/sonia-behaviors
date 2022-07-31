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
from sonia_hardware_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 31 2022
@author: CS
'''
class align_launchSM(Behavior):
	'''
	shoot torpedoes
	'''


	def __init__(self):
		super(align_launchSM, self).__init__()
		self.name = 'align_launch'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(launch_AUV8SM, 'launch_AUV8')
		self.add_behavior(vision_torpedoesSM, 'vision_torpedoes')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:601 y:386, x:308 y:407
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:77
			OperatableStateMachine.add('mission',
										wait_mission(),
										transitions={'continue': 'vision_torpedoes'},
										autonomy={'continue': Autonomy.Off})

			# x:252 y:125
			OperatableStateMachine.add('vision_torpedoes',
										self.use_behavior(vision_torpedoesSM, 'vision_torpedoes'),
										transitions={'finished': 'launch_AUV8', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:506 y:170
			OperatableStateMachine.add('launch_AUV8',
										self.use_behavior(launch_AUV8SM, 'launch_AUV8'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
