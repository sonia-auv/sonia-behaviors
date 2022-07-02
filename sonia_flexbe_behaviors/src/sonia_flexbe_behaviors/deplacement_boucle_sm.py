#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.add_pose_to_trajectory import add_pose_to_trajectory
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.search_zigzag import search_zigzag
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 01 2022
@author: Camille
'''
class Deplacement_boucleSM(Behavior):
	'''
	Déplacement en boucle
	'''


	def __init__(self):
		super(Deplacement_boucleSM, self).__init__()
		self.name = 'Deplacement_boucle'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:510 y:128
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:117
			OperatableStateMachine.add('set_mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:291 y:408
			OperatableStateMachine.add('go_back',
										add_pose_to_trajectory(positionX=-4, positionY=0, positionZ=1.5, orientationX=0, orientationY=0, orientationZ=135, frame=0, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory2', 'trajectory': 'trajectory2'})

			# x:229 y:64
			OperatableStateMachine.add('init',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'zigzag'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:516 y:426
			OperatableStateMachine.add('init2',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'go_back'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory2'})

			# x:112 y:249
			OperatableStateMachine.add('restart',
										wait_target_reached(),
										transitions={'target_reached': 'init', 'target_not_reached': 'init', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:711 y:135
			OperatableStateMachine.add('send1',
										send_to_planner(),
										transitions={'continue': 'end', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:100 y:441
			OperatableStateMachine.add('send2',
										send_to_planner(),
										transitions={'continue': 'restart', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory2'})

			# x:479 y:29
			OperatableStateMachine.add('zigzag',
										search_zigzag(boxX=5, boxY=5, stroke=1, radius=0.4, side=False),
										transitions={'continue': 'send1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:589 y:253
			OperatableStateMachine.add('end',
										wait_target_reached(),
										transitions={'target_reached': 'init2', 'target_not_reached': 'init2', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
