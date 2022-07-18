#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_sonar_states.start_bundle import start_bundle
from sonia_sonar_states.start_stop_sonar import start_stop_sonar
from sonia_sonar_states.stop_bundle import stop_bundle
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 22 2022
@author: .
'''
class test_align_sonarSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(test_align_sonarSM, self).__init__()
		self.name = 'test_align_sonar'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('start_sonar',
										start_stop_sonar(startStop=True),
										transitions={'continue': 'start_bundle'},
										autonomy={'continue': Autonomy.Off})

			# x:258 y:39
			OperatableStateMachine.add('start_bundle',
										start_bundle(resetBundle=True),
										transitions={'continue': 'stop_bundle'},
										autonomy={'continue': Autonomy.Off})

			# x:304 y:324
			OperatableStateMachine.add('stop-sonar',
										start_stop_sonar(startStop=False),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:446 y:107
			OperatableStateMachine.add('stop_bundle',
										stop_bundle(resetBundle=False, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send2planer', 'time_out': 'stop-sonar'},
										autonomy={'continue': Autonomy.Off, 'time_out': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:113 y:279
			OperatableStateMachine.add('stop_sonar',
										start_stop_sonar(startStop=False),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:188 y:161
			OperatableStateMachine.add('send2planer',
										send_to_planner(),
										transitions={'continue': 'stop_sonar', 'failed': 'stop-sonar'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
