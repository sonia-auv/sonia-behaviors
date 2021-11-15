#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.find_vision_target import find_vision_target
from sonia_flexbe_states.move_single import move_single
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 14 2021
@author: FA
'''
class search_frontSM(Behavior):
	'''
	Look for a vision target on the front camera
	'''


	def __init__(self):
		super(search_frontSM, self).__init__()
		self.name = 'search_front'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:676 y:96, x:857 y:406
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target'])
		_state_machine.userdata.target = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:335 y:23, x:333 y:115, x:332 y:189, x:328 y:275, x:430 y:365, x:530 y:365
		_sm_move_right_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['target'], conditions=[
										('finished', [('move right', 'continue')]),
										('failed', [('move right', 'failed')]),
										('failed', [('find front target', 'failed')]),
										('finished', [('find front target', 'continue')])
										])

		with _sm_move_right_0:
			# x:108 y:43
			OperatableStateMachine.add('move right',
										move_single(positionX=0, positionY=2, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=25, precision=0, rotation=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:107 y:186
			OperatableStateMachine.add('find front target',
										find_vision_target(number_samples=10, timeout=20),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})


		# x:302 y:304, x:304 y:216, x:295 y:129, x:286 y:44, x:430 y:365, x:530 y:365
		_sm_move_left_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['target'], conditions=[
										('failed', [('find_target', 'failed')]),
										('finished', [('find_target', 'continue')]),
										('failed', [('left', 'failed')]),
										('finished', [('left', 'continue')])
										])

		with _sm_move_left_1:
			# x:77 y:104
			OperatableStateMachine.add('left',
										move_single(positionX=0, positionY=-1, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=25, precision=0, rotation=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:77 y:253
			OperatableStateMachine.add('find_target',
										find_vision_target(number_samples=10, timeout=20),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})


		# x:413 y:34, x:417 y:135, x:414 y:320, x:411 y:209, x:430 y:365, x:530 y:365
		_sm_move_forward_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['target'], conditions=[
										('failed', [('move foward', 'failed')]),
										('finished', [('move foward', 'continue')]),
										('finished', [('find target', 'continue')]),
										('failed', [('find target', 'failed')])
										])

		with _sm_move_forward_2:
			# x:166 y:50
			OperatableStateMachine.add('move foward',
										move_single(positionX=1, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=25, precision=0, rotation=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:155 y:225
			OperatableStateMachine.add('find target',
										find_vision_target(number_samples=10, timeout=20),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'target'})



		with _state_machine:
			# x:104 y:66
			OperatableStateMachine.add('pose left',
										create_pose(positionX=0, positionY=-1, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'Move left'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'left_pose'})

			# x:333 y:40
			OperatableStateMachine.add('Move left',
										_sm_move_left_1,
										transitions={'finished': 'finished', 'failed': 'Move right'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target': 'target'})

			# x:336 y:190
			OperatableStateMachine.add('Move right',
										_sm_move_right_0,
										transitions={'finished': 'finished', 'failed': 'back to middle'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target': 'target'})

			# x:620 y:243
			OperatableStateMachine.add('back to middle',
										move_to_target(),
										transitions={'continue': 'Move forward', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'left_pose'})

			# x:857 y:186
			OperatableStateMachine.add('Move forward',
										_sm_move_forward_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target': 'target'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
