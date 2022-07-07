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
Created on Wed Jun 19 2022
@author: FA
'''
class SynchroMasterSM(Behavior):
	'''
	Synchronisation du master pour effectuer la tâche suivante ensemble.
	'''


	def __init__(self):
		super(SynchroMasterSM, self).__init__()
		self.name = 'Synchro Master'

		# parameters of this behavior
		self.add_parameter('Change_depth', True)
		self.add_parameter('Max_distance_to_surface', 0.5)
		self.add_parameter('Max_distance_to_bottom', 2)
		self.add_parameter('Difference_between_sub', 0.75)
		self.add_parameter('Mission_ID', 0)

		# references to used behaviors
		self.add_behavior(MovewithAddposemsgSM, 'Move with Addpose msg')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:791 y:272, x:488 y:312, x:572 y:57, x:545 y:556
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'timeout', 'failed_target_reached'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:64
			OperatableStateMachine.add('sync',
										synchro_master(depth_change=self.Change_depth, max_depth_surface=self.Max_distance_to_surface, max_depth_bottom=self.Max_distance_to_bottom, min_depth_offset=self.Difference_between_sub, mission_id=self.Mission_ID, timeout=180),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'timeout', 'pose_change_depth': 'Move with Addpose msg'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off, 'pose_change_depth': Autonomy.Off},
										remapping={'depth_change': 'depth_change'})

			# x:92 y:433
			OperatableStateMachine.add('Move with Addpose msg',
										self.use_behavior(MovewithAddposemsgSM, 'Move with Addpose msg'),
										transitions={'finished': 'sync', 'failed': 'failed', 'failed_target_reached': 'failed_target_reached'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_target_reached': Autonomy.Inherit},
										remapping={'pose': 'depth_change'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
