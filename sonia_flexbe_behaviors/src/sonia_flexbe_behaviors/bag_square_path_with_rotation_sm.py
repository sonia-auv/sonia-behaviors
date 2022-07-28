#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.square_search_with_rotation_sm import square_search_with_rotationSM
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: Willy Kao
'''
class bag_square_path_with_rotationSM(Behavior):
	'''
	This behavior allows the sub to do a defined incremental rotation until a full rotation is done in-between square movements for every depth level.
	'''


	def __init__(self):
		super(bag_square_path_with_rotationSM, self).__init__()
		self.name = 'bag_square_path_with_rotation'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(square_search_with_rotationSM, 'square_search_with_rotation')
		self.add_behavior(square_search_with_rotationSM, 'square_search_with_rotation_2')
		self.add_behavior(square_search_with_rotationSM, 'square_search_with_rotation_3')
		self.add_behavior(square_search_with_rotationSM, 'square_search_with_rotation_4')
		self.add_behavior(square_search_with_rotationSM, 'square_search_with_rotation_5')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1285 y:408, x:905 y:136
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:165 y:67
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'go_down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'traj1'})

			# x:356 y:82
			OperatableStateMachine.add('go_down',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=1, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'square_search_with_rotation'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj1', 'trajectory': 'traj2'})

			# x:358 y:181
			OperatableStateMachine.add('go_down_2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.5, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'square_search_with_rotation_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj3', 'trajectory': 'traj4'})

			# x:358 y:266
			OperatableStateMachine.add('go_down_3',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.5, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'square_search_with_rotation_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj5', 'trajectory': 'traj6'})

			# x:359 y:348
			OperatableStateMachine.add('go_down_4',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.5, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'square_search_with_rotation_4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj7', 'trajectory': 'traj8'})

			# x:357 y:435
			OperatableStateMachine.add('go_down_5',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.5, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'square_search_with_rotation_5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj9', 'trajectory': 'traj10'})

			# x:933 y:547
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj11'})

			# x:615 y:71
			OperatableStateMachine.add('square_search_with_rotation',
										self.use_behavior(square_search_with_rotationSM, 'square_search_with_rotation',
											parameters={'speed': 2}),
										transitions={'finished': 'go_down_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_traj': 'traj2', 'output_traj': 'traj3'})

			# x:619 y:174
			OperatableStateMachine.add('square_search_with_rotation_2',
										self.use_behavior(square_search_with_rotationSM, 'square_search_with_rotation_2',
											parameters={'speed': 2}),
										transitions={'finished': 'go_down_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_traj': 'traj4', 'output_traj': 'traj5'})

			# x:611 y:267
			OperatableStateMachine.add('square_search_with_rotation_3',
										self.use_behavior(square_search_with_rotationSM, 'square_search_with_rotation_3',
											parameters={'speed': 2}),
										transitions={'finished': 'go_down_4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_traj': 'traj6', 'output_traj': 'traj7'})

			# x:610 y:364
			OperatableStateMachine.add('square_search_with_rotation_4',
										self.use_behavior(square_search_with_rotationSM, 'square_search_with_rotation_4',
											parameters={'speed': 2}),
										transitions={'finished': 'go_down_5', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_traj': 'traj8', 'output_traj': 'traj9'})

			# x:604 y:454
			OperatableStateMachine.add('square_search_with_rotation_5',
										self.use_behavior(square_search_with_rotationSM, 'square_search_with_rotation_5',
											parameters={'speed': 2}),
										transitions={'finished': 'planner', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_traj': 'traj10', 'output_traj': 'traj11'})

			# x:1058 y:259
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'check_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1117 y:50
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=15, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait_target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
