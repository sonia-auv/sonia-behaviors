#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class snake_mouvementSM(Behavior):
	'''
	Search mouvement in a snake partern. Includes 7 moves with a time entered in parameter
	'''


	def __init__(self):
		super(snake_mouvementSM, self).__init__()
		self.name = 'snake_mouvement'

		# parameters of this behavior
		self.add_parameter('distance_side', 4)
		self.add_parameter('timeout', 30)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:633 y:64, x:596 y:201
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:67 y:67
			OperatableStateMachine.add('slide left half pose',
										create_pose(positionX=self.distance_side/4, positionY=-self.distance_side/2, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.timeout/2, precision=0, rotation=True),
										transitions={'continue': 'slide right pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'slide_left_half'})

			# x:773 y:195
			OperatableStateMachine.add('half slide left 2',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_left_half'})

			# x:290 y:289
			OperatableStateMachine.add('slide left',
										move_to_target(),
										transitions={'continue': 'slide right 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_left'})

			# x:550 y:388
			OperatableStateMachine.add('slide left 2',
										move_to_target(),
										transitions={'continue': 'slide right 3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_left'})

			# x:69 y:283
			OperatableStateMachine.add('slide left pose',
										create_pose(positionX=self.distance_side/4, positionY=-self.distance_side, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.timeout, precision=0, rotation=True),
										transitions={'continue': 'half slide left'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'slide_left'})

			# x:287 y:180
			OperatableStateMachine.add('slide right',
										move_to_target(),
										transitions={'continue': 'slide left', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_right'})

			# x:290 y:399
			OperatableStateMachine.add('slide right 2',
										move_to_target(),
										transitions={'continue': 'slide left 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_right'})

			# x:781 y:379
			OperatableStateMachine.add('slide right 3',
										move_to_target(),
										transitions={'continue': 'half slide left 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_right'})

			# x:67 y:175
			OperatableStateMachine.add('slide right pose',
										create_pose(positionX=self.distance_side/4, positionY=self.distance_side, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.timeout, precision=0, rotation=True),
										transitions={'continue': 'slide left pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'slide_right'})

			# x:291 y:56
			OperatableStateMachine.add('half slide left',
										move_to_target(),
										transitions={'continue': 'slide right', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'slide_left_half'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
