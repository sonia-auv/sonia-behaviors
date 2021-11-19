#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.test_new_alignement_sm import testnewalignementSM
from sonia_flexbe_states.move_to_target import move_to_target
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
		self.add_behavior(testnewalignementSM, 'Alignement with stop/test new alignement')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:127 y:293, x:361 y:264, x:422 y:93
		_state_machine = OperatableStateMachine(outcomes=['lost_target', 'failed', 'success'], input_keys=['target', 'filterchain', 'header_name', 'bounding_box'])
		_state_machine.userdata.target = ' '
		_state_machine.userdata.filterchain = ' '
		_state_machine.userdata.header_name = '  '
		_state_machine.userdata.bounding_box = 150

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:446 y:259, x:439 y:144, x:443 y:55, x:330 y:365, x:598 y:278, x:530 y:365, x:630 y:365, x:730 y:365
		_sm_alignement_with_stop_0 = ConcurrencyContainer(outcomes=['failed', 'lost_target', 'success'], input_keys=['target', 'filterchain', 'bounding_box', 'header_name'], conditions=[
										('failed', [('test new alignement', 'failed')]),
										('success', [('test new alignement', 'align_successed')]),
										('lost_target', [('test new alignement', 'timeout_reached')]),
										('lost_target', [('move', 'continue')]),
										('failed', [('move', 'failed')])
										])

		with _sm_alignement_with_stop_0:
			# x:145 y:53
			OperatableStateMachine.add('test new alignement',
										self.use_behavior(testnewalignementSM, 'Alignement with stop/test new alignement'),
										transitions={'align_successed': 'success', 'timeout_reached': 'lost_target', 'failed': 'failed'},
										autonomy={'align_successed': Autonomy.Inherit, 'timeout_reached': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'filterchain': 'filterchain', 'bounding_box': 'bounding_box', 'header_name': 'header_name'})

			# x:83 y:189
			OperatableStateMachine.add('move',
										move_to_target(),
										transitions={'continue': 'lost_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'target'})



		with _state_machine:
			# x:138 y:92
			OperatableStateMachine.add('Alignement with stop',
										_sm_alignement_with_stop_0,
										transitions={'failed': 'failed', 'lost_target': 'lost_target', 'success': 'success'},
										autonomy={'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'success': Autonomy.Inherit},
										remapping={'target': 'target', 'filterchain': 'filterchain', 'bounding_box': 'bounding_box', 'header_name': 'header_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
