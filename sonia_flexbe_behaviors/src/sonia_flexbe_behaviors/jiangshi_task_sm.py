#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.chaaaaaaaarge____sm import CHAAAAAAAARGESM
from sonia_flexbe_behaviors.vision_jiangshi_sm import vision_jiangshiSM
from sonia_flexbe_states.create_absolute_depth import create_absolute_depth
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William Brouillard
'''
class jiangshi_taskSM(Behavior):
	'''
	Find the Jiangshi and ram into it.
	'''


	def __init__(self):
		super(jiangshi_taskSM, self).__init__()
		self.name = 'jiangshi_task'

		# parameters of this behavior
		self.add_parameter('after_charge_distance', 6)

		# references to used behaviors
		self.add_behavior(CHAAAAAAAARGESM, 'CHAAAAAAAARGE!!!')
		self.add_behavior(vision_jiangshiSM, 'vision_jiangshi')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:762 y:123, x:250 y:380
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:181 y:20
			OperatableStateMachine.add('depth_jiangshi',
										create_absolute_depth(positionZ=2),
										transitions={'continue': 'pose_after_charge'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:730 y:232
			OperatableStateMachine.add('move_after_charge',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'after_charge_pose'})

			# x:33 y:112
			OperatableStateMachine.add('move_buffer',
										move_to_target(),
										transitions={'continue': 'vision_jiangshi', 'failed': 'vision_jiangshi'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'depth_pose'})

			# x:485 y:22
			OperatableStateMachine.add('pose_after_charge',
										create_pose(positionX=self.after_charge_distance, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=self.after_charge_distance*5, precision=0, rotation=True),
										transitions={'continue': 'move_buffer'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'after_charge_pose'})

			# x:193 y:183
			OperatableStateMachine.add('vision_jiangshi',
										self.use_behavior(vision_jiangshiSM, 'vision_jiangshi'),
										transitions={'finished': 'CHAAAAAAAARGE!!!', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:464 y:154
			OperatableStateMachine.add('CHAAAAAAAARGE!!!',
										self.use_behavior(CHAAAAAAAARGESM, 'CHAAAAAAAARGE!!!'),
										transitions={'finished': 'move_after_charge', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
