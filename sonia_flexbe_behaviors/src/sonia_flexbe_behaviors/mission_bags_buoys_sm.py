#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.add_pose_to_trajectory import add_pose_to_trajectory
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_navigation_states.yaw_orbit_from_given_point import yaw_orbit_from_given_point
from sonia_vision_states.start_rosbag_record import start_rosbag_record
from sonia_vision_states.stop_rosbag_record import stop_rosbag_record
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun May 15 2022
@author: Willy Kao
'''
class mission_bags_buoysSM(Behavior):
	'''
	Mission to create bags for the buoys
	'''


	def __init__(self):
		super(mission_bags_buoysSM, self).__init__()
		self.name = 'mission_bags_buoys'

		# parameters of this behavior
		self.add_parameter('initial_depth', 0)
		self.add_parameter('initial_backward_length', 0)
		self.add_parameter('backward_length_in_between_arcs', 0)
		self.add_parameter('arc_angle', 0)
		self.add_parameter('orbit_speed', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:595 y:662, x:11 y:699
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:32
			OperatableStateMachine.add('wait_mission',
										wait_mission(),
										transitions={'continue': 'set_planner_mode', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:533 y:212
			OperatableStateMachine.add('go_backward_2',
										add_pose_to_trajectory(positionX=-self.backward_length_in_between_arcs, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_left_in_orbit_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_right_orbit_1', 'trajectory': 'traj_backward_2'})

			# x:528 y:400
			OperatableStateMachine.add('go_backward_3',
										add_pose_to_trajectory(positionX=-self.backward_length_in_between_arcs, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_right_in_orbit_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_left_orbit_2', 'trajectory': 'traj_backward_3'})

			# x:751 y:33
			OperatableStateMachine.add('go_backward_4',
										add_pose_to_trajectory(positionX=-self.backward_length_in_between_arcs, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_left_in_orbit_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_right_orbit_2', 'trajectory': 'traj_backward_4'})

			# x:754 y:226
			OperatableStateMachine.add('go_backward_5',
										add_pose_to_trajectory(positionX=-self.backward_length_in_between_arcs, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_right_in_orbit_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_left_orbit_3', 'trajectory': 'traj_backward_5'})

			# x:316 y:115
			OperatableStateMachine.add('go_down',
										add_pose_to_trajectory(positionX=0, positionY=0, positionZ=self.initial_depth, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_backward_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj_down'})

			# x:311 y:292
			OperatableStateMachine.add('go_left_in_orbit_1',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length, pointY=0, rotation=self.arc_angle/2, speed=self.orbit_speed),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_backward_1', 'trajectory': 'traj_left_orbit_1'})

			# x:521 y:308
			OperatableStateMachine.add('go_left_in_orbit_2',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length+self.backward_length_in_between_arcs, pointY=0, rotation=self.arc_angle, speed=self.orbit_speed),
										transitions={'continue': 'go_backward_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_backward_2', 'trajectory': 'traj_left_orbit_2'})

			# x:746 y:128
			OperatableStateMachine.add('go_left_in_orbit_3',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs, pointY=0, rotation=self.arc_angle, speed=self.orbit_speed),
										transitions={'continue': 'go_backward_5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_backward_4', 'trajectory': 'traj_left_orbit_3'})

			# x:522 y:122
			OperatableStateMachine.add('go_right_in_orbit_1',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length, pointY=0, rotation=-self.arc_angle, speed=self.orbit_speed),
										transitions={'continue': 'go_backward_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_2', 'trajectory': 'traj_right_orbit_1'})

			# x:519 y:492
			OperatableStateMachine.add('go_right_in_orbit_2',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs, pointY=0, rotation=-self.arc_angle, speed=self.orbit_speed),
										transitions={'continue': 'go_backward_4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_backward_3', 'trajectory': 'traj_right_orbit_2'})

			# x:738 y:333
			OperatableStateMachine.add('go_right_in_orbit_3',
										yaw_orbit_from_given_point(pointX=self.initial_backward_length+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs+self.backward_length_in_between_arcs, pointY=0, rotation=-self.arc_angle, speed=self.orbit_speed),
										transitions={'continue': 'send_to_planner_final'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_backward_5', 'trajectory': 'traj_right_orbit_3'})

			# x:310 y:33
			OperatableStateMachine.add('init_traj',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'go_down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'traj'})

			# x:530 y:30
			OperatableStateMachine.add('init_traj_2',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'go_right_in_orbit_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'traj_2'})

			# x:317 y:373
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj_left_orbit_1'})

			# x:745 y:431
			OperatableStateMachine.add('send_to_planner_final',
										send_to_planner(),
										transitions={'continue': 'wait_target_reached_final', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj_right_orbit_3'})

			# x:91 y:117
			OperatableStateMachine.add('set_planner_mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:278 y:517
			OperatableStateMachine.add('start_bag_record',
										start_rosbag_record(bag_name='bag_test', topic_name='/proc_simulation/front/compressed', timer_split=0, record_path='/home/willy'),
										transitions={'continue': 'init_traj_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rosbag_proc': 'rosbag_proc', 'command': 'command'})

			# x:781 y:596
			OperatableStateMachine.add('stop_rosbag_record',
										stop_rosbag_record(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rosbag_proc': 'rosbag_proc', 'command': 'command'})

			# x:310 y:455
			OperatableStateMachine.add('wait_target_reached',
										wait_target_reached(),
										transitions={'target_reached': 'start_bag_record', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:740 y:522
			OperatableStateMachine.add('wait_target_reached_final',
										wait_target_reached(),
										transitions={'target_reached': 'stop_rosbag_record', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:312 y:206
			OperatableStateMachine.add('go_backward_1',
										add_pose_to_trajectory(positionX=-self.initial_backward_length, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'go_left_in_orbit_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_down', 'trajectory': 'traj_backward_1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
