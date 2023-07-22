#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.single_pose_move_sm import SinglePoseMoveSM
from sonia_vision_states.calc_pixel_meter_ratio import calc_pixel_meter_ratio
from sonia_vision_states.get_blob_size import get_blob_size
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
from sonia_vision_states.vision_alignment_check import vision_alignemnt_check
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 21 2023
@author: Nimai
'''
class AlignGateSM(Behavior):
	'''
	align to gate
	'''


	def __init__(self):
		super(AlignGateSM, self).__init__()
		self.name = 'Align Gate'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Move Forward')
		self.add_behavior(SinglePoseMoveSM, 'Move Right')
		self.add_behavior(SinglePoseMoveSM, 'Return to origin')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:395 y:123, x:378 y:359
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Init X calc block',
										init_blob_calc_block(),
										transitions={'success': 'get origin X'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block_x'})

			# x:28 y:575
			OperatableStateMachine.add('Calc x',
										calc_pixel_meter_ratio(),
										transitions={'success': 'init YZ calc block', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_x', 'func_block': 'func_block_x'})

			# x:796 y:212
			OperatableStateMachine.add('Calc yz',
										calc_pixel_meter_ratio(),
										transitions={'success': 'Align to gate', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_yz', 'func_block': 'func_block_yz'})

			# x:472 y:548
			OperatableStateMachine.add('Get origin yz',
										get_blob_size(filterchain_obj_topic="/proc_image_processing/gate_left_target", dist_from_origin=0, nb_img=10),
										transitions={'success': 'Move Right', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_yz'})

			# x:25 y:242
			OperatableStateMachine.add('Move Forward',
										self.use_behavior(SinglePoseMoveSM, 'Move Forward',
											parameters={'positionX': -0.3}),
										transitions={'finished': 'get diff x', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:641 y:538
			OperatableStateMachine.add('Move Right',
										self.use_behavior(SinglePoseMoveSM, 'Move Right',
											parameters={'positionY': 0.3}),
										transitions={'finished': 'get yz diff', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:20 y:466
			OperatableStateMachine.add('Return to origin',
										self.use_behavior(SinglePoseMoveSM, 'Return to origin',
											parameters={'positionX': 0.3}),
										transitions={'finished': 'Calc x', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:809 y:330
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionY': -0.3}),
										transitions={'finished': 'Calc yz', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:32 y:347
			OperatableStateMachine.add('get diff x',
										get_blob_size(filterchain_obj_topic="/proc_image_processing/gate_left_target", dist_from_origin=0.1, nb_img=10),
										transitions={'success': 'Return to origin', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_x'})

			# x:32 y:149
			OperatableStateMachine.add('get origin X',
										get_blob_size(filterchain_obj_topic="/proc_image_processing/gate_left_target", dist_from_origin=0, nb_img=10),
										transitions={'success': 'Move Forward', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_x'})

			# x:843 y:461
			OperatableStateMachine.add('get yz diff',
										get_blob_size(filterchain_obj_topic="/proc_image_processing/gate_left_target", dist_from_origin=-0.1, nb_img=10),
										transitions={'success': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'calc_block': 'calc_block_yz'})

			# x:282 y:580
			OperatableStateMachine.add('init YZ calc block',
										init_blob_calc_block(),
										transitions={'success': 'Get origin yz'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'calc_block_yz'})

			# x:587 y:57
			OperatableStateMachine.add('Align to gate',
										vision_alignemnt_check(filterchain_obj_topic="/proc_image_processing/gate_left_target", filterchain_box_topic="/proc_image_processing/gate_box", blob_size=130, nb_imgs=10, timeout_sec=5, max_adjusts=10, tolerance=0.05),
										transitions={'timeout': 'failed', 'success': 'finished', 'failed': 'failed'},
										autonomy={'timeout': Autonomy.Off, 'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x_func': 'func_block_x', 'yz_function': 'func_block_yz'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
