#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.aligment_with_stopping_sm import AligmentwithstoppingSM
from sonia_flexbe_behaviors.search_circle_sm import search_circleSM
from sonia_flexbe_states.get_simple_vision_target import get_simple_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William Brouillard
'''
class vision_droppersSM(Behavior):
	'''
	Find the bin with the right filter chain, then drop the markers in the bin.
	'''


	def __init__(self):
		super(vision_droppersSM, self).__init__()
		self.name = 'vision_droppers'

		# parameters of this behavior
		self.add_parameter('filterchain', 'deep_wolf')
		self.add_parameter('header_name', 'wolf')
		self.add_parameter('camera_no', 2)

		# references to used behaviors
		self.add_behavior(AligmentwithstoppingSM, 'Aligment with stopping')
		self.add_behavior(search_circleSM, 'search_circle')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1015 y:225, x:166 y:490, x:366 y:444
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:187
			OperatableStateMachine.add('start_filter_chain',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:438 y:27
			OperatableStateMachine.add('get target',
										get_simple_vision_target(bounding_box_pixel=100, image_height=400, image_width=600, ratio_victory=0.5, number_of_average=10, max_mouvement=1, alignement_distance=5, rotation=True, timeout=60),
										transitions={'success': 'stop_filter_success', 'align': 'Aligment with stopping', 'move': 'move_to_target', 'failed': 'stop_filter_fail', 'search': 'search_circle'},
										autonomy={'success': Autonomy.Off, 'align': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off, 'search': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name', 'pose': 'pose', 'bounding_box': 'bounding_box'})

			# x:530 y:165
			OperatableStateMachine.add('move_to_target',
										move_to_target(),
										transitions={'continue': 'get target', 'failed': 'stop_filter_fail'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:283 y:195
			OperatableStateMachine.add('search_circle',
										self.use_behavior(search_circleSM, 'search_circle'),
										transitions={'finished': 'get target', 'failed': 'stop_filter_fail', 'lost_target': 'stop_filter_lost'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'target': 'filterchain'})

			# x:397 y:509
			OperatableStateMachine.add('stop_filter_fail',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:275 y:299
			OperatableStateMachine.add('stop_filter_lost',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'lost_target', 'failed': 'lost_target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:946 y:96
			OperatableStateMachine.add('stop_filter_success',
										start_filter_chain(param_node_name=self.filterchain, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:721 y:198
			OperatableStateMachine.add('Aligment with stopping',
										self.use_behavior(AligmentwithstoppingSM, 'Aligment with stopping'),
										transitions={'lost_target': 'stop_filter_lost', 'failed': 'stop_filter_fail', 'success': 'get target'},
										autonomy={'lost_target': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'pose', 'filterchain': 'filterchain', 'header_name': 'header_name', 'bounding_box': 'bounding_box'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
