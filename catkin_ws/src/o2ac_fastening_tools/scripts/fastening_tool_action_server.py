#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
import os.path
import yaml
from std_msgs.msg import String
from o2ac_fastening_tools.srv import *
from o2ac_msgs.msg import *
from util import *

import dynamixel_workbench_msgs.msg
import dynamixel_workbench_msgs.srv

class FasteningToolController(object):
    _feedback = FastenerGripperControlFeedback()
    _result = FastenerGripperControlResult()

    def __init__(self):
        # get data from .yaml
        controller_namespace = rospy.get_param("~controller_ns")
        self._service_name = '/'.join(['', controller_namespace, 'dynamixel_command'])
        self._dynamixel_state_topic = '/'.join(['', controller_namespace, 'dynamixel_state'])
        self._motor_status_topic_is_list = True
        self._dynamixel_current_state = []
        conf_gripper_filename = rospy.get_param("~tools_info")
        fastening_tools = read_object_yaml_config(conf_gripper_filename)

        # initialize motor id table
        self.fastening_tools = dict()
        for tool_name, properties in fastening_tools.iteritems():
            rospy.loginfo("Loaded " + tool_name + " on motor id " + str(properties['ID']))
            self.fastening_tools.update({tool_name : properties['ID']})
        
        self._test_listener = rospy.Subscriber(self._dynamixel_state_topic, rospy.AnyMsg, self._test_listener_callback)

        rospy.wait_for_service(self._service_name)
        self.dynamixel_command_write = rospy.ServiceProxy(self._service_name, dynamixel_workbench_msgs.srv.DynamixelCommand)

        self._action_name = 'screw_tool_control'
        self._as = actionlib.SimpleActionServer(self._action_name, FastenerGripperControlAction, execute_cb=self.execute_control, auto_start = False)
        self._as.start()

    def _test_listener_callback(self, data):
        '''
        This function is executed when a messgae is received by _test_listener.
        It determines the actual ROS message type that is used in the topic and creates a subscriber with the
        given type, to read the status of the motors.
        '''
        message_type = data._connection_header['type']
        self._test_listener.unregister()
        if message_type == 'dynamixel_workbench_msgs/DynamixelState':
            self._motor_status_topic_is_list = False
            self._dynamixel_current_state = dynamixel_workbench_msgs.msg.DynamixelState()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic, dynamixel_workbench_msgs.msg.DynamixelState, self._listener_callback)
        elif message_type == 'dynamixel_workbench_msgs/DynamixelStateList':
            self._motor_status_topic_is_list = True
            self._dynamixel_current_state = dynamixel_workbench_msgs.msg.DynamixelStateList()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic, dynamixel_workbench_msgs.msg.DynamixelStateList, self._listener_callback)
        else:
            rospy.logerr('Unexpected message type: ' + message_type)

    def _listener_callback(self, data):
        '''
        This function is executed when a message is received by _listener.
        The data received is the status of the motors connected to the controller. It is stored in the _dynamixel_current_state variable
        '''
        self._dynamixel_current_state = data

    def set_torque_enable(self, motor_id, value):
        try:
            res = self.dynamixel_command_write('', motor_id, "Torque_Enable", value)
        except rospy.ServiceException as exc:
            rospy.logwarn('An exception occurred in the Torque_Enable set, but processing continues.')
        else:
            if not res.comm_result:
                rospy.logerr('Can not set torque_enable to XL-320. (ID=%i)' %motor_id)
            return res.comm_result
        return True

    def set_moving_speed(self, motor_id, value):
        try:
            res = self.dynamixel_command_write('', motor_id, "Moving_Speed", value)
            print(res, motor_id, "Moving_Speed", value)
        except rospy.ServiceException as exc:
            rospy.logwarn('An exception occurred in the Moving_Speed set. Processing retry.')
        else:
            if not res.comm_result:
                rospy.logerr('Can not set speed to XL-320. (ID=%i)' %motor_id)
            return res.comm_result
        return True

    def get_present_speed(self, motor_id):
        #Wait for the current state top be available
        while '_listener' not in dir(self):
                rospy.sleep(0.1)
        if self._motor_status_topic_is_list == True:
            while not self._dynamixel_current_state.dynamixel_state:
                rospy.sleep(0.1)
        elif self._motor_status_topic_is_list == False:
            while self._dynamixel_current_state.name == '':
                rospy.sleep(0.1)
        
        if self._motor_status_topic_is_list == True:
            return next((motor for motor in self._dynamixel_current_state.dynamixel_state if motor.id == motor_id)).present_velocity
        elif self._motor_status_topic_is_list == False:
            return self._dynamixel_current_state.present_velocity

    def execute_control(self, goal):
        t_duration = 1
        motor_id = self.fastening_tools[goal.fastening_tool_name]
        self._result.control_result = True
        if not goal.speed:
            goal.speed = 1023
        self._feedback.motor_speed = goal.speed

        #  For tighten the maximum speed is at 1023, while for loosen it is 2047, at 1024 the motor is stopped
        if goal.direction == "loosen" :
            goal.speed = 1024 + goal.speed
            if goal.speed > 2047 :
                goal.speed = 2047
        elif goal.direction == "tighten" :
            if goal.speed > 1023 :
                goal.speed = 1023

        if (goal.fastening_tool_name in self.fastening_tools) == False :
            rospy.logerr("'%s' does not exist in %s." % (goal.fastening_tool_name, self.conf_gripper_filename))
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return

        if not self.set_moving_speed(motor_id, goal.speed) :
            self.set_moving_speed(motor_id, 1024)
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = False
            self._as.set_aborted(self._result)
            return
        
        if goal.direction == "loosen" and not goal.duration:
            rospy.logwarn("Loosen command was sent, but without a duration. Setting to 2 seconds.")
            goal.duration = 2
        elif goal.direction == "tighten" and not goal.duration:
            rospy.logwarn("Tighten command was sent, but without a maximum duration. Setting to 10 seconds.")
            goal.duration = 10

        # Turn the motor for the specified number of seconds.
        # Rotate the motor until goal.duration is reached is loaded and stops.
        success_flag = True
        start_time = rospy.get_rostime()
        rospy.sleep(1)   # Wait for motor to start up (and avoid reading wrong speed values)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success_flag = False
                break

            if (rospy.get_rostime().secs - start_time.secs) > goal.duration:
                rospy.loginfo("Stopping motor due to timeout")
                if goal.direction == "loosen":
                    success_flag = True
                    self.set_moving_speed(motor_id, 1024)
                elif goal.direction == "tighten": # If motor does not stall before timeout, tightening was not successful
                    success_flag = False
                    self.set_moving_speed(motor_id, 1024)
                break
            
            if self._feedback.motor_speed == 0 and goal.direction == 'tighten':
                rospy.loginfo("Stopping motor because it has stalled (the screw is tightened)")
                success_flag = True
                break
            
            # Read motor speed. To avoid erroneous readouts, it is read twice. This is not an ideal workaround.
            first_speed = self.get_present_speed(motor_id)
            rospy.sleep(0.1)
            second_speed = self.get_present_speed(motor_id)
            if first_speed == -1 or second_speed == -1:
                success_flag = False
                rospy.logwarn("Error in motor readout. Stopping.")
                break
                
            # If both readings are below a threshold, the motor has stalled
            if first_speed <= 10 and second_speed <= 10:
                self._feedback.motor_speed = 0
            else:
                self._feedback.motor_speed = max(first_speed, second_speed)

            rospy.logdebug("first_speed, second_speed = " + str(first_speed) + ", " + str(second_speed))
            self._as.publish_feedback(self._feedback)
        
        motor_stopped = self.set_moving_speed(motor_id, 1024)
        if motor_stopped and success_flag:
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = True
            self._as.set_succeeded(self._result)
        else:
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = False
            self._as.set_aborted(self._result)

        
if __name__ == '__main__':
    rospy.init_node('fastening_tool_controller')
    server = FasteningToolController()
    rospy.spin()
