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
from sonia_vision_states.start_rosbag_record import start_rosbag_record
from sonia_vision_states.stop_rosbag_record import stop_rosbag_record
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun May 15 2022
@author: Camille
'''
class test_bagSM(Behavior):
	'''
	test record
	'''


	def __init__(self):
		super(test_bagSM, self).__init__()
		self.name = 'test_bag'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:555 y:380, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:109 y:105
			OperatableStateMachine.add('start_bagging',
										start_rosbag_record(bag_name='test_record', topic_name='camera_array/front/image_raw/compressed', timer_split=10, record_path='/home/kamoss/Bags/test'),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rosbag_proc': 'rosbag_proc', 'command': 'command'})

			# x:391 y:220
			OperatableStateMachine.add('stop_bagging',
										stop_rosbag_record(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rosbag_proc': 'rosbag_proc', 'command': 'command'})

			# x:397 y:84
			OperatableStateMachine.add('wait',
										wait_mission(),
										transitions={'continue': 'stop_bagging', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
