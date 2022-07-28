#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_hardware_states.activate_io import activate_io
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 12 2022
@author: CS
'''
class drop_AUV8SM(Behavior):
	'''
	Shift and drop for AUV8
	'''


	def __init__(self):
		super(drop_AUV8SM, self).__init__()
		self.name = 'drop_AUV8'

		# parameters of this behavior
		self.add_parameter('activate_drop_auv8', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:573 y:479, x:446 y:363
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:50
			OperatableStateMachine.add('activation',
										activate_behavior(activate=self.activate_drop_auv8),
										transitions={'activate': 'init', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:564 y:245
			OperatableStateMachine.add('check',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'drop', 'moving': 'wait', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:761 y:390
			OperatableStateMachine.add('drop',
										activate_io(element=1, side=0, action=1, timeout=10),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': '2nd_drop_attempt'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:124 y:140
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'shift'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:352 y:118
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:177 y:224
			OperatableStateMachine.add('shift',
										manual_add_pose_to_trajectory(positionX=-0.164, positionY=0.076, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:601 y:67
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'drop', 'target_not_reached': 'check', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:963 y:524
			OperatableStateMachine.add('2nd_drop_attempt',
										activate_io(element=1, side=0, action=1, timeout=8),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
