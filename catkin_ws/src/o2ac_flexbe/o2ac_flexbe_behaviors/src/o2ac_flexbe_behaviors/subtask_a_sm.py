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
from o2ac_flexbe_states.insertion import InsertActionState
from o2ac_flexbe_states.orient import OrientActionState
from o2ac_flexbe_states.pick import PickActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 1635318513067
@author: AUTOGENERATED
'''
class subtask_aSM(Behavior):
	'''
	Behavior extracted from PDDL trace: subtask_a
	'''


	def __init__(self):
		super(subtask_aSM, self).__init__()
		self.name = 'subtask_a'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:380 y:360
		_state_machine = OperatableStateMachine(outcomes=['failed', 'finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('1: pick b_bot motor ',
										PickActionState(robot_name="b_bot", object_name="motor", helper_robot_name=""),
										transitions={'success': '2: orient b_bot motor', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:80 y:130
			OperatableStateMachine.add('2: orient b_bot motor',
										OrientActionState(robot_name="b_bot", task_name="motor", object_name=undefined),
										transitions={'success': '3: insert b_bot motor panel_motor  ', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:130 y:220
			OperatableStateMachine.add('3: insert b_bot motor panel_motor  ',
										InsertActionState(robot_name="b_bot", object_name="motor", target="panel_motor", task_name="", helper_robot_name=""),
										transitions={'success': '4: fasten-dual-arm a_bot motor  b_bot', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:180 y:310
			OperatableStateMachine.add('4: fasten-dual-arm a_bot motor  b_bot',
										FastenActionState(robot_name="a_bot", object_name="motor", task_name="", helper_robot_name="b_bot"),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
