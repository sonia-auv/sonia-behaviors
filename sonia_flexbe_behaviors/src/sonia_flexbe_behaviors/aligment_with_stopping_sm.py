#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.stop_move import stop_move
from sonia_flexbe_states.verify_centroid import verify_centroid
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 19 2021
@author: FA
'''
class AligmentwithstoppingSM(Behavior):
	'''
	Alignement on the target by stopping the mouvement when the target is in the centroid.
	'''


	def __init__(self):
		super(AligmentwithstoppingSM, self).__init__()
		self.name = 'Aligment with stopping'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:187 y:296, x:419 y:249, x:848 y:133
		_state_machine = OperatableStateMachine(outcomes=['lost_target', 'failed', 'success'], input_keys=['target', 'filterchain', 'header_name', 'bounding_box'])
		_state_machine.userdata.target = ' '
		_state_machine.userdata.filterchain = ' '
		_state_machine.userdata.header_name = '  '
		_state_machine.userdata.bounding_box = 150

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:703 y:332, x:537 y:129, x:569 y:42, x:397 y:168, x:794 y:118, x:654 y:243, x:733 y:190
		_sm_alignement_with_stop_0 = ConcurrencyContainer(outcomes=['failed', 'lost_target', 'success'], input_keys=['target', 'filterchain', 'bounding_box', 'header_name'], conditions=[
										('lost_target', [('move', 'continue')]),
										('failed', [('move', 'failed')]),
										('success', [('centroid', 'align_complete')]),
										('lost_target', [('centroid', 'timeout_reached')])
										])

		with _sm_alignement_with_stop_0:
			# x:160 y:48
			OperatableStateMachine.add('centroid',
										verify_centroid(number_sample=10, timeout=30),
										transitions={'align_complete': 'success', 'timeout_reached': 'lost_target'},
										autonomy={'align_complete': Autonomy.Off, 'timeout_reached': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'bounding_box': 'bounding_box', 'header_name': 'header_name'})

			# x:106 y:156
			OperatableStateMachine.add('move',
										move_to_target(),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target'})



		with _state_machine:
			# x:138 y:92
			OperatableStateMachine.add('Alignement with stop',
										_sm_alignement_with_stop_0,
										transitions={'failed': 'failed', 'lost_target': 'lost_target', 'success': 'stop move'},
										autonomy={'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'target', 'filterchain': 'filterchain', 'bounding_box': 'bounding_box', 'header_name': 'header_name'})

			# x:568 y:102
			OperatableStateMachine.add('stop move',
										stop_move(timeout=10),
										transitions={'continue': 'success', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
