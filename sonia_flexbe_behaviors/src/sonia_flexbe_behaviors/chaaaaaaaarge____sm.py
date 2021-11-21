#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from sonia_flexbe_states.create_absolute_depth import create_absolute_depth
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 16 2021
@author: William Brouillard
'''
class CHAAAAAAAARGESM(Behavior):
	'''
	LEEEEROOOOOOOOOOYYYYYYYYYYYYYYY JENNKINNSSSSSS!
	'''


	def __init__(self):
		super(CHAAAAAAAARGESM, self).__init__()
		self.name = 'CHAAAAAAAARGE!!!'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:972 y:514, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:49 y:54
			OperatableStateMachine.add('log',
										LogState(text='RAMMING', severity=2),
										transitions={'done': 'pose_charge'},
										autonomy={'done': Autonomy.Off})

			# x:381 y:156
			OperatableStateMachine.add('backoff',
										move_to_target(),
										transitions={'continue': 'absolute_dedpth_pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'backoff_pose'})

			# x:115 y:168
			OperatableStateMachine.add('charge_forward',
										move_to_target(),
										transitions={'continue': 'backoff', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'charge_pose'})

			# x:792 y:379
			OperatableStateMachine.add('log_end',
										LogState(text='Raming done', severity=2),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:602 y:286
			OperatableStateMachine.add('move_absolute_depth',
										move_to_target(),
										transitions={'continue': 'log_end', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:657 y:32
			OperatableStateMachine.add('pose_backoff',
										create_pose(positionX=-1, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=10, precision=0, rotation=True),
										transitions={'continue': 'charge_forward'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'backoff_pose'})

			# x:268 y:29
			OperatableStateMachine.add('pose_charge',
										create_pose(positionX=2.5, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=45, precision=0, rotation=True),
										transitions={'continue': 'pose_backoff'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'charge_pose'})

			# x:615 y:161
			OperatableStateMachine.add('absolute_dedpth_pose',
										create_absolute_depth(positionZ=1),
										transitions={'continue': 'move_absolute_depth'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'depth_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
