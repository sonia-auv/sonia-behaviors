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
class square_mouvementSM(Behavior):
	'''
	Mouvement for searching on the bottom camera
	'''


	def __init__(self):
		super(square_mouvementSM, self).__init__()
		self.name = 'square_mouvement'

		# parameters of this behavior
		self.add_parameter('square_distance', 2)
		self.add_parameter('move_time', 5)
		self.add_parameter('down_distance', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:394 y:170, x:564 y:228
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:71
			OperatableStateMachine.add('left pose',
										create_pose(positionX=0, positionY=-self.square_distance/2, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.move_time, precision=0, rotation=True),
										transitions={'continue': 'right pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'left'})

			# x:244 y:172
			OperatableStateMachine.add('down pose',
										create_pose(positionX=0, positionY=0, positionZ=self.down_distance, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.move_time, precision=0, rotation=True),
										transitions={'continue': 'move left'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'down'})

			# x:61 y:267
			OperatableStateMachine.add('forward pose',
										create_pose(positionX=self.square_distance, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.move_time, precision=0, rotation=True),
										transitions={'continue': 'backward pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'forward'})

			# x:790 y:207
			OperatableStateMachine.add('move backward',
										move_to_target(),
										transitions={'continue': 'move left 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'backward'})

			# x:490 y:59
			OperatableStateMachine.add('move down',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'down'})

			# x:418 y:385
			OperatableStateMachine.add('move forward',
										move_to_target(),
										transitions={'continue': 'move right', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'forward'})

			# x:240 y:292
			OperatableStateMachine.add('move left',
										move_to_target(),
										transitions={'continue': 'move forward', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'left'})

			# x:736 y:63
			OperatableStateMachine.add('move left 2',
										move_to_target(),
										transitions={'continue': 'move down', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'left'})

			# x:694 y:352
			OperatableStateMachine.add('move right',
										move_to_target(),
										transitions={'continue': 'move backward', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'right'})

			# x:60 y:166
			OperatableStateMachine.add('right pose',
										create_pose(positionX=0, positionY=self.square_distance, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.move_time, precision=0, rotation=True),
										transitions={'continue': 'forward pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'right'})

			# x:218 y:72
			OperatableStateMachine.add('backward pose',
										create_pose(positionX=-self.square_distance, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.move_time, precision=0, rotation=True),
										transitions={'continue': 'down pose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'backward'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
