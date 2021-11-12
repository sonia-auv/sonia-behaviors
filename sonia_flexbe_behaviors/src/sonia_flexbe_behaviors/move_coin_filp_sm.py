#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.move_single import move_single
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 11 2021
@author: FA
'''
class move_coin_filpSM(Behavior):
	'''
	Behavior to test the coin filp in simulation
	'''


	def __init__(self):
		super(move_coin_filpSM, self).__init__()
		self.name = 'move_coin_filp'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1124 y:460, x:327 y:403
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:437 y:160
			OperatableStateMachine.add('move to yaw',
										move_single(positionX=0, positionY=0, positionZ=2, orientationX=0, orientationY=0, orientationZ=180, frame=1, time=20, precision=0, rotation=True),
										transitions={'continue': 'rotate to gate', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:687 y:242
			OperatableStateMachine.add('rotate to gate',
										move_single(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=68, frame=2, time=5, precision=0, rotation=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
