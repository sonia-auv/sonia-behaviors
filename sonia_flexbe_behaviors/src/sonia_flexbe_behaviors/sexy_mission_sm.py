#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.activate_io import activate_io
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_vision_states.get_front_vision_target import get_front_vision_target
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 12 2022
@author: KY&LM
'''
class sexy_missionSM(Behavior):
	'''
	Mission for consenting adults
	'''


	def __init__(self):
		super(sexy_missionSM, self).__init__()
		self.name = 'sexy_mission'

		# parameters of this behavior
		self.add_parameter('vision_buoys_filterchain', 'deep_compe_front')
		self.add_parameter('camera_no', 1)
		self.add_parameter('vision_buoys_target', 'Gun')
		self.add_parameter('bounding_box_width', 200)
		self.add_parameter('bounding_box_height', 350)
		self.add_parameter('center_bounding_box_width', 100)
		self.add_parameter('center_bounding_box_height', 100)
		self.add_parameter('max_mouvement', 1)
		self.add_parameter('min_mouvement', 0.25)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1454 y:39, x:896 y:310
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:80
			OperatableStateMachine.add('mission_switch',
										wait_mission(),
										transitions={'continue': 'filter_chain'},
										autonomy={'continue': Autonomy.Off})

			# x:67 y:287
			OperatableStateMachine.add('filter_chain',
										start_filter_chain(filterchain=self.vision_buoys_filterchain, target=self.vision_buoys_target, camera_no=self.camera_no),
										transitions={'continue': 'init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'front', 'target': 'target'})

			# x:884 y:87
			OperatableStateMachine.add('get_target',
										get_front_vision_target(center_bounding_box_pixel_height=self.center_bounding_box_height, center_bounding_box_pixel_width=self.center_bounding_box_width, bounding_box_pixel_height=self.bounding_box_height, bounding_box_pixel_width=self.bounding_box_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=self.max_mouvement, min_mouvement=self.min_mouvement, long_rotation=False, timeout=10, speed_profile=0),
										transitions={'success': 'drop', 'align': 'get_target', 'move': 'get_target', 'failed': 'failed', 'search': 'get_target'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'topic': 'topic', 'camera_no': 'front', 'target': 'target', 'input_trajectory': 'input_trajectory', 'output_trajectory': 'trajectory', 'camera': 'camera', 'angle': 'angle'})

			# x:257 y:84
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_trajectory'})

			# x:1311 y:209
			OperatableStateMachine.add('stop_filterchain',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'front'})

			# x:1112 y:90
			OperatableStateMachine.add('drop',
										activate_io(element=1, side=0, action=1, timeout=8),
										transitions={'continue': 'stop_filterchain', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
