#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# import of required action
from o2ac_msgs.msg import ReleaseAction, ReleaseGoal


class ReleaseActionState(EventState):
    '''
    Actionlib for releasing an object hold by a robot

    -- robot_name         string  Name of robot performing the operation

    <= success              Release sequence completed successfully.
    <= error                Release sequence failed to execute.

    '''

    def __init__(self, robot_name):
        super(ReleaseActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/Release_object'
        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient({self._topic: ReleaseAction})
        self._robot_name = robot_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result.success:
                Logger.logwarn('Fail to complete Release sequence')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed Release sequence')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = ReleaseGoal()
        goal.robot_name = self._robot_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Release command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
