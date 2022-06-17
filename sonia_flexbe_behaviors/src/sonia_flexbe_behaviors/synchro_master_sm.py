#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.synchro_master import synchro_master
from sonia_flexbe_behaviors.move_with_addpose_msg_sm import MovewithAddposemsgSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 08 2022
@author: FA
'''
class SynchroMasterSM(Behavior):
	'''
	Test du state pour la synchronisation du master
	'''


	def __init__(self):
		super(SynchroMasterSM, self).__init__()
		self.name = 'Synchro Master'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(MovewithAddposemsgSM, 'Move with Addpose msg')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:791 y:272, x:437 y:267, x:572 y:57
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'timeout'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:64
			OperatableStateMachine.add('sync',
										synchro_master(depth_change=True, max_depth_surface=0.5, max_depth_bottom=1.5, min_depth_offset=1, mission_id=2, timeout=180),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'timeout', 'pose_change_depth': 'Move with Addpose msg'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off, 'pose_change_depth': Autonomy.Off},
										remapping={'depth_change': 'depth_change'})

			# x:92 y:433
			OperatableStateMachine.add('Move with Addpose msg',
										self.use_behavior(MovewithAddposemsgSM, 'Move with Addpose msg'),
										transitions={'finished': 'sync', 'failed': 'failed', 'failed_target_reached': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_target_reached': Autonomy.Inherit},
										remapping={'pose': 'depth_change'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
