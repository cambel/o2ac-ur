#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from o2ac_flexbe_states.fasten import FastenActionState
from o2ac_flexbe_states.pick import PickActionState
from o2ac_flexbe_states.place import PlaceActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 1635312683072
@author: Cristian Beltran
'''
class TaskSkeletontoFlexbeSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(TaskSkeletontoFlexbeSM, self).__init__()
		self.name = 'Task Skeleton to Flexbe'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		a_bot = "a_bot"
		b_bot = "b_bot"
		task_name = "assembly"
		shaft = "shaft"
		end_cap = "end_cap"
		# x:30 y:365, x:480 y:552
		_state_machine = OperatableStateMachine(outcomes=['failed', 'finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('0_pick a_bot panel_bearing',
										PickActionState(robot_name=a_bot, object_name=panel_bearing),
										transitions={'success': '1_place a_bot panel_bearing base', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:80 y:130
			OperatableStateMachine.add('1_place a_bot panel_bearing base',
										PlaceActionState(robot_name=a_bot, object_name=panel_bearing, target=base),
										transitions={'success': '2_fasten-with-screw-dual-arm b_bot panel_bearing task_name a_bot', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:130 y:220
			OperatableStateMachine.add('2_fasten-with-screw-dual-arm b_bot panel_bearing task_name a_bot',
										FastenActionState(robot_name=b_bot, object_name=panel_bearing, task_name=task_name, helper_robot_name=a_bot),
										transitions={'success': '3_pick a_bot panel_motor', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:180 y:310
			OperatableStateMachine.add('3_pick a_bot panel_motor',
										PickActionState(robot_name=a_bot, object_name=panel_motor),
										transitions={'success': '4_place a_bot panel_motor sub-assembly-base-panel_bearing', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:230 y:400
			OperatableStateMachine.add('4_place a_bot panel_motor sub-assembly-base-panel_bearing',
										PlaceActionState(robot_name=a_bot, object_name=panel_motor, target=sub-assembly-base-panel_bearing),
										transitions={'success': '5_fasten-with-screw-dual-arm b_bot panel_motor task_name a_bot', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:280 y:490
			OperatableStateMachine.add('5_fasten-with-screw-dual-arm b_bot panel_motor task_name a_bot',
										FastenActionState(robot_name=b_bot, object_name=panel_motor, task_name=task_name, helper_robot_name=a_bot),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
