#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.save_pose_sm import SavePoseSM
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 24 2023
@author: Nimai
'''
class TestSavePoseSM(Behavior):
	'''
	Test Save Pose
	'''


	def __init__(self):
		super(TestSavePoseSM, self).__init__()
		self.name = 'Test Save Pose'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SavePoseSM, 'Save Pose')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:461, x:410 y:259
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Save Pose',
										self.use_behavior(SavePoseSM, 'Save Pose'),
										transitions={'finished': 'Init Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'save_traj': 'save_traj'})

			# x:441 y:35
			OperatableStateMachine.add('Move and turn',
										manual_add_pose_to_trajectory(positionX=1.5, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Send Move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'move_traj', 'trajectory': 'move_traj'})

			# x:707 y:67
			OperatableStateMachine.add('Send Move',
										send_to_planner(),
										transitions={'continue': 'Wait Move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'move_traj'})

			# x:523 y:425
			OperatableStateMachine.add('Send Save',
										send_to_planner(),
										transitions={'continue': 'Wait Save Move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'save_traj'})

			# x:787 y:253
			OperatableStateMachine.add('Wait Move',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Send Save', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:258 y:441
			OperatableStateMachine.add('Wait Save Move',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:253 y:38
			OperatableStateMachine.add('Init Move',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Move and turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'move_traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
