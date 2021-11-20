#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
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
		self.add_parameter('charge_distance', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:744 y:466, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:75 y:34
			OperatableStateMachine.add('pose_charge',
										create_pose(positionX=self.charge_distance, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.charge_distance*5, precision=0, rotation=True),
										transitions={'continue': 'pose_backoff'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'charge_pose'})

			# x:381 y:156
			OperatableStateMachine.add('backoff',
										move_to_target(),
										transitions={'continue': 'absolute_dedpth_pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'backoff_pose'})

			# x:137 y:157
			OperatableStateMachine.add('charge_forward',
										move_to_target(),
										transitions={'continue': 'backoff', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'charge_pose'})

			# x:602 y:286
			OperatableStateMachine.add('move_absolute_depth',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:275 y:35
			OperatableStateMachine.add('pose_backoff',
										create_pose(positionX=-1, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=10, precision=0, rotation=True),
										transitions={'continue': 'charge_forward'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'backoff_pose'})

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
