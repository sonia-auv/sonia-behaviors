#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_complex_behaviors.calculate_pixel_meter_function_in_xy_sm import CalculatepixelmeterfunctioninxySM
from sonia_complex_behaviors.calculate_pixel_meter_function_in_z_sm import CalculatepixelmeterfunctioninzSM
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_vision_states.get_pipe_angle_pos import get_pipe_angle_pos
from sonia_vision_states.init_blob_calc_block import init_blob_calc_block
from sonia_vision_states.pipe_align_angle import pipe_align_angle
from sonia_vision_states.pipe_align_translation import pipe_align_translation
from sonia_vision_states.pipe_align_zoom import pipe_align_zoom
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 04 2023
@author: Nimai
'''
class AlignpathSM(Behavior):
	'''
	Align to path
	'''


	def __init__(self):
		super(AlignpathSM, self).__init__()
		self.name = 'Align path'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CalculatepixelmeterfunctioninxySM, 'Calculate pixel meter function in xy')
		self.add_behavior(CalculatepixelmeterfunctioninzSM, 'Calculate pixel meter function in z')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1028 y:400, x:489 y:455
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:5 y:164
			OperatableStateMachine.add('init move translation list',
										init_blob_calc_block(),
										transitions={'success': 'init move zoom list'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'translation_counter'})

			# x:0 y:492
			OperatableStateMachine.add('Calculate pixel meter function in xy',
										self.use_behavior(CalculatepixelmeterfunctioninxySM, 'Calculate pixel meter function in xy'),
										transitions={'finished': 'Calculate pixel meter function in z', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'func_block': 'function_constants_xy'})

			# x:159 y:376
			OperatableStateMachine.add('Calculate pixel meter function in z',
										self.use_behavior(CalculatepixelmeterfunctioninzSM, 'Calculate pixel meter function in z'),
										transitions={'finished': 'Get pipe data', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'func_block': 'function_constants_z'})

			# x:187 y:43
			OperatableStateMachine.add('Get pipe data',
										get_pipe_angle_pos(num_imgs=10),
										transitions={'Success': 'Move in translation', 'Failed': 'failed'},
										autonomy={'Success': Autonomy.Off, 'Failed': Autonomy.Off},
										remapping={'angle_average': 'angle_average', 'average_ctr': 'average_ctr', 'average_size': 'average_size'})

			# x:766 y:59
			OperatableStateMachine.add('Move in Zoom',
										pipe_align_zoom(target_size=130, tolerance=1, max_attempts=10),
										transitions={'sizeok': 'Align Angle', 'moved': 'Wait zoom', 'failed': 'failed'},
										autonomy={'sizeok': Autonomy.Off, 'moved': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'function_constants_z': 'function_constants_z', 'avg_size': 'average_size', 'counter': 'move_zoom_counter'})

			# x:479 y:72
			OperatableStateMachine.add('Move in translation',
										pipe_align_translation(tolerance=1, max_attempts=10),
										transitions={'centered': 'Move in Zoom', 'moved': 'Wait translation', 'failed': 'failed'},
										autonomy={'centered': Autonomy.Off, 'moved': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'function_constants_xy': 'function_constants_xy', 'average_ctr': 'average_ctr', 'counter': 'translation_counter'})

			# x:253 y:296
			OperatableStateMachine.add('Wait translation',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Get pipe data', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:668 y:215
			OperatableStateMachine.add('Wait zoom',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Get pipe data', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:771 y:352
			OperatableStateMachine.add('Wait zoom_2',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Get pipe data', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:100 y:295
			OperatableStateMachine.add('init get data list_3',
										init_blob_calc_block(),
										transitions={'success': 'Calculate pixel meter function in xy'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'move_angle_counter'})

			# x:71 y:222
			OperatableStateMachine.add('init move zoom list',
										init_blob_calc_block(),
										transitions={'success': 'init get data list_3'},
										autonomy={'success': Autonomy.Off},
										remapping={'calc_block': 'move_zoom_counter'})

			# x:888 y:204
			OperatableStateMachine.add('Align Angle',
										pipe_align_angle(max_attempts=10, tolerance=1.0),
										transitions={'aligned': 'finished', 'moved': 'Wait zoom_2', 'failed': 'failed'},
										autonomy={'aligned': Autonomy.Off, 'moved': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'avg_angle': 'average_size', 'counter': 'move_angle_counter'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
