#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.activate_io import activate_io
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 16 2021
@author: FA
'''
class droppersSM(Behavior):
	'''
	Drop the markers in the target detected by the vision. It suppose that the sub is already aligned in the middle of the target.
	'''


	def __init__(self):
		super(droppersSM, self).__init__()
		self.name = 'droppers'

		# parameters of this behavior
		self.add_parameter('depth_to_target', 1)
		self.add_parameter('x_offset', -0.18)
		self.add_parameter('y_offset', 0.1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:359 y:453, x:629 y:176
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:99 y:44
			OperatableStateMachine.add('pose down',
										create_pose(positionX=0, positionY=0, positionZ=self.depth_to_target, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'pose offset'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'down'})

			# x:326 y:40
			OperatableStateMachine.add('move down',
										move_to_target(),
										transitions={'continue': 'move offset', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'down'})

			# x:323 y:181
			OperatableStateMachine.add('move offset',
										move_to_target(),
										transitions={'continue': 'activate dropper', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'offset'})

			# x:101 y:151
			OperatableStateMachine.add('pose offset',
										create_pose(positionX=self.x_offset, positionY=self.y_offset, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'move down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'offset'})

			# x:324 y:302
			OperatableStateMachine.add('activate dropper',
										activate_io(element=2, side=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
