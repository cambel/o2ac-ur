#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_msgs
import tf_conversions
import tf
from math import pi, radians, sin, cos, pi
tau = 2.0*pi  # Part of math from Python 3.6
import math
import time
import numpy as np

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg
import moveit_msgs.msg
import moveit_task_constructor_msgs.msg


from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg

import o2ac_msgs
import o2ac_msgs.srv

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.common import O2ACCommon
import o2ac_routines.helpers as helpers
from o2ac_routines.helpers import wait_for_UR_program, get_target_force

from ur_control import transformations as ur_transformations
from ur_control import conversions
from ur_control.constants import TERMINATION_CRITERIA


class O2ACTaskboard(O2ACCommon):
  """
  This class contains the taskboard routines.
  """
  def __init__(self):
    super(O2ACTaskboard, self).__init__()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # Initialize debug monitor
    self.start_task_timer()
    
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/4))

    self.at_set_screw_hole = geometry_msgs.msg.PoseStamped()
    self.at_set_screw_hole.header.frame_id = "taskboard_set_screw_link"
    self.at_set_screw_hole.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    self.at_set_screw_hole.pose.position.x = 0.001   # MAGIC NUMBER
    self.at_set_screw_hole.pose.position.y = -0.0005   # MAGIC NUMBER
    self.at_set_screw_hole.pose.position.z = 0.001   # MAGIC NUMBER (points downward)
    if not self.assembly_database.db_name == "taskboard":
      self.assembly_database.change_assembly("taskboard")

  def spawn_example_objects(self):
    # This function spawns the objects into the tray as if they had been recognized by the vision node
    names = ["taskboard_idler_pulley_small", "bearing", "shaft", "motor_pulley"]
    offsets = {"bearing": [-.04, -.02, .001],
    "taskboard_idler_pulley_small": [.07, .06, .03],
    "shaft": [.03, -.06, .005], 
    "motor_pulley": [-.01, .12, .005]}

    # We publish each object to its own frame.
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(.5)
    counter = 0
    for name in names:
      collision_object = self.assembly_database.get_collision_object(name)
      if collision_object:
        counter += 1
        collision_object.header.frame_id = "collision_object_spawn_helper_frame" + str(counter)
        if name == "bearing":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        if name == "taskboard_idler_pulley_small" or name == "motor_pulley":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        elif name == "shaft":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
          
        offset = offsets[name]
        broadcaster.sendTransform((offset[0], offset[1], offset[2]), q_rotate, rospy.Time.now(), 
            "collision_object_spawn_helper_frame" + str(counter), "tray_center")
        rospy.sleep(1.0) # Wait for the transform to have propagated through the system

        self.planning_scene_interface.add_object(collision_object)
        # print("======== collision object: " + name)
        # print(collision_object.mesh_poses)
        # print(collision_object.subframe_names)
        # print(collision_object.subframe_poses)
      else:
        rospy.logerr("Could not retrieve collision object:" + name)

  #####
  def look_in_tray(self):
    #TODO (felixvd)
    # Look at tray
    # loop through all items
    # check if they were recognized and are graspable
    # otherwise check position again
    # if still not graspable either skip to next item or try to reposition
    pass
  
  def check_objects_in_tray(self, robot_name, order):
    """ Return the easier to find objects from a birdview """
    self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, 
                                                   end_effector_link=robot_name + "_outside_camera_color_frame",
                                                   speed=.5, acceleration=.3, wait=True, move_lin=False)
    
    success = False
    while not success:
      rospy.sleep(0.5)
      success = self.get_3d_poses_from_ssd()

    order_ids = [self.assembly_database.name_to_id(object_id) for object_id in order]
    object_in_tray = copy.deepcopy(self.objects_in_tray)
    res = []
    for item in order_ids:
      if object_in_tray.get(item, None):
        res.append(self.assembly_database.id_to_name(item))
    return res 

  def prep_taskboard_task(self):
    """
    Equip the set screw tool and M3 tool, and move to the position before task start.
    """

    self.a_bot.go_to_named_pose("home")

    self.equip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    self.b_bot.go_to_named_pose("home")
    
    self.equip_tool("b_bot", "set_screw_tool")
    self.b_bot.go_to_named_pose("horizontal_screw_ready")

    self.move_b_bot_to_setscrew_initial_pos()
  
  def move_b_bot_to_setscrew_initial_pos(self):
    screw_approach = copy.deepcopy(self.at_set_screw_hole)
    screw_approach.pose.position.x = -0.03
    self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.5)
    screw_approach.pose.position.x = -0.005
    self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.1)

  def do_screw_tasks_from_prep_position(self):
    ### - Set screw
    
    # Move into the screw hole with motor on
    self.vision.activate_camera("b_bot_inside_camera")
    self.do_task("M2 set screw")
    
    # TODO: check set screw success with a_bot, do spiral motion with b_bot otherwise
    
    ### SCREW M3 WITH A_BOT
    self.vision.activate_camera("a_bot_outside_camera")
    screw_picked = self.pick_screw_from_feeder("a_bot", screw_size = 3)
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    # Move b_bot back, a_bot to screw
    self.unequip_tool("b_bot", "set_screw_tool")
    self.equip_tool("b_bot", "screw_tool_m4")
    self.b_bot.go_to_named_pose("feeder_pick_ready")

    if screw_picked:
      self.subtask_completed["M3 screw"] = self.do_task("M3 screw")
    self.unequip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("home")
    
    #### SCREW M4 WITH B_BOT
    self.subtask_completed["M4 screw"] = self.do_task("M4 screw")

    self.b_bot.go_to_named_pose("home")
    self.unequip_tool("b_bot", "screw_tool_m4")

  def full_taskboard_task(self, do_screws=True, skip_tray_placing=True):
    """
    Start the taskboard task from the fully prepped position (set screw tool and M3 tool equipped)
    """
    #####
    self.subtask_completed = {
      "M2 set screw": True,  # FIXME
      "M3 screw": False,
      "M4 screw": False,
      "belt": False,
      "bearing": False,
      "screw_bearing": False,
      "motor pulley": False,
      "shaft": False,  
      "idler pulley": False,
    }
    if do_screws:
      self.do_screw_tasks_from_prep_position()

    if not skip_tray_placing:
      self.take_tray_from_agv()

    # self.subtask_completed["belt"] = self.do_task("belt")
    
    # self.subtask_completed["idler pulley"] = self.do_task("idler pulley")
    # self.unequip_tool("b_bot")

    # self.subtask_completed["motor pulley"] = self.do_task("motor pulley")

    # self.subtask_completed["bearing"] = self.do_task("bearing")
    # if self.subtask_completed["bearing"]:
    #   self.subtask_completed["screw_bearing"] = self.do_task("screw_bearing")

    # self.subtask_completed["shaft"] = self.do_task("shaft")
    
    order = ["belt", "motor pulley", "shaft", "idler pulley", "bearing"]
    task_complete = False

    # found_items = self.check_objects_in_tray("b_bot", order)
    # rospy.loginfo("Items found so far: %s" % found_items)
    # for item in found_items:
    #   self.execute_step(item)
      
    # Then loop through the remaining items
    self.confirm_to_proceed("Continue into loop to retry parts?")
    unsuccessful_attempts = 0
    while not task_complete:
      for item in order:
        if not self.execute_step(item):
          unsuccessful_attempts += 1
      task_complete = all(self.subtask_completed.values())

    self.publish_status_text("FINISHED")

  def execute_step(self, item):
      assert not rospy.is_shutdown(), "did ros master die?"
      self.unequip_tool("b_bot") # Just in case
      self.b_bot.gripper.open(wait=False) # Release any possible item that got stuck in the gripper

      if not self.subtask_completed[item]:
        self.confirm_to_proceed("Reattempt " + str(item) + "?")
        self.subtask_completed[item] = self.do_task(item)
        if item == "bearing" and self.subtask_completed[item]: 
          if not self.subtask_completed["screw_bearing"]:
            self.do_task("screw_bearing")
        
        rospy.loginfo("============================================")
        rospy.loginfo("==== Task: %s completed with status: %s ====" % (item, self.subtask_completed[item]))
        rospy.loginfo("============================================")      
      
      return self.subtask_completed[item]

  def do_task(self, task_name, fake_execution_for_calibration=False):
    self.publish_status_text("Target: " + task_name)

    if task_name == "belt":
      self.ab_bot.go_to_named_pose("home")
      
      self.a_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="a_bot_outside_camera_color_frame", speed=.8, move_lin=False)

      self.vision.activate_camera("a_bot_outside_camera")
      self.activate_led("a_bot")
      res = self.get_3d_poses_from_ssd()
      r2 = self.get_feasible_grasp_points("belt")
      if r2:
        pick_goal = r2[0]
        pick_goal.pose.position.z = -0.001
      else:
        rospy.logerr("Could not find belt grasp pose! Aborting.")
        return False
      
      self.confirm_to_proceed("Pick tool with b_bot?")
      # Equip the belt tool with b_bot
      if not self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_pick_hook.urp"):
        return False
      rospy.sleep(2)  # Wait for b_bot to have moved out of the way

      self.simple_pick("a_bot", pick_goal, gripper_force=100.0, grasp_width=.08, axis="z")
      self.a_bot.move_lin_rel(relative_translation=[0,0,.1])
      a_bot_wait_with_belt_pose = [0.646294116973877, -1.602117200891012, 2.0059760252581995, -1.3332312864116211, -0.8101084868060511, -2.4642069975482386]
      self.a_bot.move_joints(a_bot_wait_with_belt_pose)
      
      # Check for pick success
      b_bot_look_at_belt = [1.95739448, -1.40047674, 1.92903739, -1.98750128, -2.1883457, 1.7778782]
      self.b_bot.move_joints(b_bot_look_at_belt, speed=0.3)
      self.vision.activate_camera("b_bot_outside_camera")
      
      if not self.vision.check_pick_success("belt"):
        rospy.logerr("Belt pick has failed. Return tool and abort.")
        self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
        rospy.sleep(2)
        pick_goal.pose.position.x = 0  # In tray_center
        pick_goal.pose.position.y = 0
        pick_goal.pose.position.z += 0.06
        self.a_bot.move_lin(pick_goal)
        self.a_bot.gripper.open(opening_width=0.07, wait=False)
        self.a_bot.go_to_named_pose("home")
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
        return False

      self.confirm_to_proceed("Load and execute the belt threading programs?")
      success_a = self.a_bot.load_program(program_name="wrs2020/taskboard_belt_v7.urp")
      success_b = self.b_bot.load_program(program_name="wrs2020/taskboard_belt_v7.urp")
      if success_a and success_b:
        print("Loaded belt program on a_bot.")
        rospy.sleep(1)
        success = self.a_bot.execute_loaded_program()
        success = self.b_bot.execute_loaded_program()
        if success:
          print("Starting belt threading execution.")
          rospy.sleep(2)
          self.a_bot.close_ur_popup()
          self.b_bot.close_ur_popup()
      else:
        print("Problem loading. Not executing belt procedure.")
        self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
        rospy.sleep(3)
        self.drop_in_tray("a_bot")
        self.a_bot.go_to_named_pose("home")
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
        return False
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))

      # b_bot is now above the tray, looking at the 
      # TODO(felixvd): Use vision to check belt threading success
      
      self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
      rospy.sleep(2)
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
      return True

    # ==========================================================

    if task_name == "M2 set screw":
      # Equip and move to the screw hole
      # self.equip_tool("b_bot", "set_screw_tool")
      # self.b_bot.go_to_named_pose("horizontal_screw_ready")
      self.vision.activate_camera("b_bot_inside_camera")
      screw_approach = copy.deepcopy(self.at_set_screw_hole)
      screw_approach.pose.position.x = -0.005
      self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.b_bot.go_to_pose_goal(self.at_set_screw_hole, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.02)

      # This expects to be exactly above the set screw hole
      self.confirm_to_proceed("Turn on motor and move into screw hole?")
      dist = .003
      self.b_bot.move_lin_rel(relative_translation=[-dist, 0, 0], speed=0.03, wait=False)
      # self.skill_server.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .002)
      self.tools.set_motor("set_screw_tool", "tighten", duration = 12.0)
      rospy.sleep(4.0) # Wait for the screw to be screwed in a little bit
      d = .003
      rospy.loginfo("Moving in further by " + str(d) + " m.")
      self.b_bot.move_lin_rel(relative_translation=[-d, 0, 0], speed=0.002, wait=False)
      
      rospy.sleep(8.0)
      self.confirm_to_proceed("Go back?")
      if self.b_bot.is_protective_stopped():
        rospy.logwarn("Robot was protective stopped after set screw insertion!")
        self.b_bot.unlock_protective_stop()
        rospy.sleep(1)
        if self.b_bot.is_protective_stopped():
          return False

      # Go back
      self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)

      self.b_bot.go_to_named_pose("horizontal_screw_ready", speed=0.5, acceleration=0.5)
      # self.confirm_to_proceed("Unequip tool?")
      # self.b_bot.go_to_named_pose("home", speed=0.5, acceleration=0.5)
      # self.unequip_tool("b_bot", "set_screw_tool")
    
    # ==========================================================

    if task_name == "M3 screw":
      if not self.a_bot.robot_status.carrying_tool and not self.a_bot.robot_status.held_tool_id == "screw_tool_m3":
        self.equip_tool("a_bot", "screw_tool_m3")
      if not fake_execution_for_calibration:
        self.pick_screw_from_feeder("a_bot", screw_size = 3)
      self.a_bot.go_to_named_pose("horizontal_screw_ready")
      approach_pose = geometry_msgs.msg.PoseStamped()
      approach_pose.header.frame_id = "taskboard_m3_screw_link"
      approach_pose.pose.position.x = -.04
      approach_pose.pose.position.y = -.12
      approach_pose.pose.position.z = -.05
      self.a_bot.go_to_pose_goal(approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

      approach_pose.pose.position.y = -.0
      approach_pose.pose.position.z = -.004  # MAGIC NUMBER (z-axis of the frame points down)
      approach_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      self.a_bot.go_to_pose_goal(approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m3_screw_link"
      hole_pose.pose.position.y = -.000  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.004  # MAGIC NUMBER (z-axis of the frame points down)
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      if not fake_execution_for_calibration:
        self.screw("a_bot", hole_pose, screw_size = 3, skip_final_loosen_and_retighten=False)
      else:
        hole_pose.pose.position.x -= 0.005
        self.a_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        self.a_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
      self.a_bot.go_to_named_pose("horizontal_screw_ready")
      self.a_bot.go_to_named_pose("home")
      if not fake_execution_for_calibration:
        self.unequip_tool("a_bot", "screw_tool_m3")

    # ==========================================================

    if task_name == "M4 screw":
      if not self.b_bot.robot_status.carrying_tool and not self.b_bot.robot_status.held_tool_id == "screw_tool_m4":
        self.equip_tool("b_bot", "screw_tool_m4")
      self.vision.activate_camera("b_bot_outside_camera")
      if not fake_execution_for_calibration:
        self.pick_screw_from_feeder("b_bot", screw_size = 4)
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m4_screw_link"
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
      hole_pose.pose.position.y = -.001  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.001  # MAGIC NUMBER (z-axis of the frame points down)
      if not fake_execution_for_calibration:
        self.screw("b_bot", hole_pose, screw_size = 4, skip_final_loosen_and_retighten=False)
      else:
        hole_pose.pose.position.x = -.01
        approach_pose = copy.deepcopy(hole_pose)
        approach_pose.pose.position.x -= .05
        self.b_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.b_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        self.b_bot.go_to_pose_goal(approach_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.confirm_to_proceed("Did it go back?")
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      self.b_bot.go_to_named_pose("home")
      if not fake_execution_for_calibration:
        self.unequip_tool("b_bot", "screw_tool_m4")

    # ==========================================================

    if task_name == "motor pulley":
      success = self.pick_and_insert_motor_pulley(task = "taskboard")
      self.unlock_base_plate()  # To ensure that nothing moved due to a failed insertion
      self.lock_base_plate()
      return success

    # ==========================================================

    if task_name == "bearing":
      return self.pick_up_and_insert_bearing(task="taskboard")
      
    if task_name == "screw_bearing":
      self.equip_tool('b_bot', 'screw_tool_m4')
      success = self.fasten_bearing(task="taskboard")
      self.unequip_tool('b_bot', 'screw_tool_m4')
      return success
    
    # ==========================================================

    if task_name == "shaft":
      return self.pick_and_insert_shaft("taskboard")
    
    # ==========================================================
    
    if task_name == "idler pulley":
      return self.pick_and_insert_idler_pulley("taskboard")