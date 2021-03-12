#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, OMRON SINIC X
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

from o2ac_routines.base import *





class O2ACCommon(O2ACBase):
  """
  This class contains the higher-level routines 
  """
  def __init__(self):
    super(O2ACCommon, self).__init__()
    self.rospack = rospkg.RosPack()

  ######## Higher-level routines used in both assembly and taskboard

  def pick(self, object_name, grasp_parameter_location = '', lift_direction_reference_frame = '', lift_direction = [], robot_name = '', save_solution_to_file=''):
    """This function creates a motion-plan for picking the item referred to by 'object_name' input.
    The item needs to be in the planning scene as a collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_pick_action(object_name, grasp_parameter_location, lift_direction_reference_frame, lift_direction, robot_name)
    for solution in result.solution.sub_trajectory:
      scene_diff = solution.scene_diff
      planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
      planning_scene_diff_req.scene = scene_diff
      # self.apply_planning_scene_diff.call(planning_scene_diff_req)   # DEBUG: Update the scene pretending the action has been completed

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting pick() after writing solution")
    return result.success

  def place(self, object_name, object_target_pose, release_object_after_place = True, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion-plan for placing the item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_place_action(object_name, object_target_pose, release_object_after_place, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting place() after writing solution")
    return result.success

  def release(self, object_name, pose_to_retreat_to = '', save_solution_to_file=''):
    """This function creates a motion-plan for releasing the placed item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_release_action(object_name, pose_to_retreat_to)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting release() after writing solution")
    return result.success

  def pick_place(self, object_name, object_target_pose, grasp_parameter_location = '', release_object_after_place = True, object_subframe_to_place = '',
    lift_direction_reference_frame = '', lift_direction = [], approach_place_direction_reference_frame = '', approach_place_direction = [], robot_names = '', force_robot_order = False, save_solution_to_file=''):
    """This function creates a motion-plan for picking and then placing item referred to by 'object_name' input.
    The item needs to be in the planning scene as a collision object.
    If the 'object_subframe_to_place' input provided, this subframe will be positioned instead of the object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_pickplace_action(object_name, object_target_pose, grasp_parameter_location, release_object_after_place, object_subframe_to_place, lift_direction_reference_frame, 
      lift_direction, approach_place_direction_reference_frame, approach_place_direction, robot_names, force_robot_order)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting pickplace() after writing solution")
    return result.success

  def fasten(self, object_name, object_target_pose, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion-plan for moving an item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The motion is: going to approach pose->approach->retreat.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_fastening_action(object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting fasten() after writing solution")
    return result.success

  def plan_wrs_subtask_b(self, object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion=plan for the subassembly task of attaching the 'object' on the base plate
    """
    result = self.do_plan_wrs_subtask_b_action(object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting subassembly() after writing solution")
    return result.success
  
  def load_MTC_solution(self, solution_file):
    """Load the result of a motion-plan from a file."""
    if not solution_file == '':
      # Load the solution
      file = self.rospack.get_path('o2ac_routines') + '/MP_solutions/' + solution_file
      with open(file, 'rb') as f:
        result = pickle.load(f)
    return result

  def execute_MTC_solution(self, solution, speed = 1.0):
    """
    Execute the result of a task plan.
    The type of the input 'solution' is mtc_msgs.msg.Solution
    """
    
    skip_stage_execution = False   # True while a subroutine like "equip/unequip"
    success = False
    screw_counter = 0
    if speed > 1.0:
      speed = 1.0
    start_state = self.robots.get_current_state()
    currently_attached_collision_objects = start_state.attached_collision_objects
    for solution in solution.sub_trajectory:
      stage_name = solution.info.creator_name
      if stage_name == 'Fasten screw (dummy)':
        rospy.sleep(2)

      # If a "start" block is encountered, skip the trajectories inside and execute the actual action they represent
      if stage_name == 'equip_tool_m4_start':
        skip_stage_execution = True
        self.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if stage_name == 'unequip_tool_m4_start':
        skip_stage_execution = True
        self.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if stage_name == 'pick_screw_m4_start':
        skip_stage_execution = True
        self.pick_screw_from_feeder('b_bot', screw_size=4)
      if stage_name == 'fasten_screw_m4_start':
        skip_stage_execution = True
        target_pose = geometry_msgs.msg.PoseStamped()
        if screw_counter == 0:
          target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
          screw_counter += 1
        else:
          target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_2'
        target_pose.pose.orientation.w = 1
        self.fasten_screw('b_bot', target_pose)
      
      # Resume executing the trajectories
      if (stage_name == 'equip_tool_m4_end' or stage_name == 'unequip_tool_m4_end' or stage_name == 'pick_screw_m4_end' or 
         stage_name == 'fasten_screw_m4_end'):
        skip_stage_execution = False
        currently_attached_collision_objects = self.robots.get_current_state().attached_collision_objects
        # The collision objects need to be updated because they might have changed during the trajectory
        continue
      
      # Check for any self-contained stages to be executed
      if stage_name == 'push plate with b_bot' and not skip_stage_execution:
        # Execute positioning UR program
        self.load_and_execute_program(robot="b_bot", program_name="wrs2020_push_motor_plate.urp", wait=True)
        continue
      if stage_name == 'move a_bot right wrs_subtask_motor_plate':
        rospy.logwarn("===================== DEBUG1")
        self.move_lin_rel("a_bot", relative_translation=[0, -0.02, 0], use_robot_base_csys=True, max_wait=5.0)
        rospy.logwarn("===================== DEBUG1b")
      if stage_name == 'move a_bot back wrs_subtask_motor_plate':
        rospy.logwarn("===================== DEBUG2")
        self.move_lin_rel("a_bot", relative_translation=[0,  0.02, 0], use_robot_base_csys=True, max_wait=5.0)
        rospy.logwarn("===================== DEBUG2b")

      # Execute trajectories
      if solution.scene_diff.robot_state.joint_state.name and not skip_stage_execution:  # If the robot state is changed (robot moved, object attached/detached)
        # Update attached collision objects
        if not currently_attached_collision_objects == solution.scene_diff.robot_state.attached_collision_objects:
          # for co in currently_attached_collision_objects:
          #   print('IN MEMORY CO: ', co.object.id)
          # for co in solution.scene_diff.robot_state.attached_collision_objects:
          #   print('IN SOLUTION CO: ', co.object.id)
          coll_objs_to_detach = [collision_object for collision_object in currently_attached_collision_objects if collision_object not in solution.scene_diff.robot_state.attached_collision_objects]
          coll_objs_to_attach = [collision_object for collision_object in solution.scene_diff.robot_state.attached_collision_objects if collision_object not in currently_attached_collision_objects]
          for attached_object in coll_objs_to_detach:
            # print('Detaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].detach_object(attached_object_name)
          for attached_object in coll_objs_to_attach:
            # print('Attaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].attach_object(attached_object_name, attached_object.link_name, touch_links=  # MODIFY attach_tool in base.py to attach_object ++ ROBOT NAME???
              [attach_to + "_robotiq_85_tip_link", 
              attach_to + "_robotiq_85_left_finger_tip_link", 
              attach_to + "_robotiq_85_left_inner_knuckle_link", 
              attach_to + "_robotiq_85_right_finger_tip_link", 
              attach_to + "_robotiq_85_right_inner_knuckle_link"])
            currently_attached_collision_objects.append(attached_object)
          currently_attached_collision_objects = [attached_collision_object for attached_collision_object in currently_attached_collision_objects if attached_collision_object not in coll_objs_to_detach]

        # Skip stage if joint_names is empty, because the stage performs no motions
        if not solution.trajectory.joint_trajectory.joint_names: 
          continue

        # Execute stage
        robot_name = solution.trajectory.joint_trajectory.joint_names[0][:5]
        arm_group = self.groups[robot_name]
        if len(solution.trajectory.joint_trajectory.joint_names) == 1:  # If only one joint is in the group, it is the gripper
          # Gripper motion
          hand_group = self.groups[robot_name + '_robotiq_85']
          hand_closed_joint_values = hand_group.get_named_target_values('close')
          hand_open_joint_values = hand_group.get_named_target_values('open')
          if stage_name == 'open hand':
            self.send_gripper_command(robot_name, 'open')
          elif stage_name == 'close hand':
            self.send_gripper_command(robot_name, 'close')
          elif 0.01 > abs(hand_open_joint_values[solution.trajectory.joint_trajectory.joint_names[0]] - solution.trajectory.joint_trajectory.points[-1].positions[0]):
            rospy.logwarn("Actuating gripper (backup path)!")
            self.send_gripper_command(robot_name, 'open')
          elif 0.01 < abs(hand_open_joint_values[solution.trajectory.joint_trajectory.joint_names[0]] - solution.trajectory.joint_trajectory.points[-1].positions[0]):
            rospy.logwarn("Actuating gripper (backup path)!")
            self.send_gripper_command(robot_name, 'close', True)
          
        else: # The robots move
          rospy.loginfo("========")
          rospy.logwarn("self.groups[robot_name].get_current_joint_values():")
          print(self.groups[robot_name].get_current_joint_values())
          rospy.logwarn("solution.trajectory.joint_trajectory.points[0].positions:")
          print(solution.trajectory.joint_trajectory.points[0].positions)
          # First check that the trajectory is safe to execute (= robot is at start of trajectory)
          if not all_close(self.groups[robot_name].get_current_joint_values(), 
                            solution.trajectory.joint_trajectory.points[0].positions,
                            0.01):
            rospy.logerr("Robot " + robot_name + " is not at the start of the next trajectory! Aborting.")
            rospy.logerr("Stage name: " + stage_name)
            rospy.logwarn("self.groups[robot_name].get_current_joint_values():")
            print(self.groups[robot_name].get_current_joint_values())
            rospy.logwarn("solution.trajectory.joint_trajectory.points[0].positions:")
            print(solution.trajectory.joint_trajectory.points[0].positions)
            return False

          # Prepare robot motion
          self.activate_ros_control_on_ur(robot_name)
          plan = arm_group.retime_trajectory(self.robots.get_current_state(), solution.trajectory, speed)
          arm_group.set_max_velocity_scaling_factor(speed)

          # Execute
          plan_success = arm_group.execute(plan, wait=True)
          success = success and plan_success
    return True

  def pick_and_move_object_with_robot(self, item_name, item_target_pose, robot_name, speed=0.1):
    """This function picks the item and move it to the target pose.
    It needs to be in the planning scene as a collision object."""
    # TODO: Implement this with MTC
    success = False
    return success
  

  ####### Vision

  def look_for_item_in_tray(self, item_name, robot_name="b_bot"):
    """
    This function will look for an item in the tray. After calling this function, the item
    is published to the planning scene.
    """

    # Look from top first
    self.go_to_pose_goal(robot_name, self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    success = self.detect_object_in_camera_view(item_name)

    # # TODO: Also move robot if object requires a close-up view (shaft, pin...)
    # if not success:
    #   poses = [self.tray_view_close_front_b, self.tray_view_close_back_b, self.tray_view_close_front_a, self.tray_view_close_back_a]
    #   for pose in poses:
    #     self.go_to_pose_goal(robot_name, pose, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    #     success = self.detect_object_in_camera_view(item_name)
    #     if success:
    #       break

    if not success:
      rospy.logdebug("Failed to find " + item_name)
    else:
      rospy.logdebug("Found " + item_name)
    return success

  def adjust_tip_position_visually(self, robot_name, use_inside_camera=False, use_outside_camera=False):
    """WIP: This function will adjust the position of the robot so that the visible tip of the object
    or tool is aligned with the target hole.
    Only one camera can be used.
    use_inside_camera should be True for grasped parts (and maybe the set screw tool).
    use_outside_camera should be True for the screw tools."""
    # TODO: Call a vision action
    success = False
    return success

  def touch_environment_with_grasped_object_to_reduce_uncertainty(self, robot_name, initial_uncertainty=[0,0,0]):
    """WIP: This touches the environment with a recently grasped object to ascertain its position."""
    # TODO: Use in-hand pose estimation particle filter
    success = False
    return success

  ########

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=0.1, speed_slow=0.02, gripper_command="close", 
          gripper_force=40.0, grasp_width=0.140,
          approach_height=0.05, item_id_to_attach = "", lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.1, 
          gripper_velocity = .1, axis="x", sign=+1):
    """
    This function (outdated) performs a grasp with the robot, but it is not updated in the planning scene.
    It does not use the object in simulation. It can be used for simple tests and prototyping, but should
    be replaced by the pick() function for the real competition.

    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    The attachment will be visible in the MoveIt planning scene. The object and its subframes can be used
    as an end effector.
    """
    
    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    if axis =="x":
      object_pose.pose.position.x += approach_height * sign
    if axis =="z":
      object_pose.pose.position.z += approach_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= approach_height * sign
    if axis =="z":
      object_pose.pose.position.z -= approach_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command=grasp_width) # Open

    rospy.loginfo("Moving down to object")
    if axis =="x":
      object_pose.pose.position.x += grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z += grasp_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acc_slow, high_precision=True, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z -= grasp_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command="close", force = gripper_force, velocity = gripper_velocity)

    if item_id_to_attach:
      try:
        self.groups[robot_name].attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_robotiq_85_tip_link", 
        robot_name + "_robotiq_85_left_finger_tip_link", 
        robot_name + "_robotiq_85_left_inner_knuckle_link", 
        robot_name + "_robotiq_85_right_finger_tip_link", 
        robot_name + "_robotiq_85_right_inner_knuckle_link"])
      except:
        rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

    if lift_up_after_pick:
      rospy.sleep(1.0)
      rospy.loginfo("Going back up")

      if axis =="x":
        object_pose.pose.position.x += approach_height * sign
      if axis =="z":
        object_pose.pose.position.z += approach_height * sign
      rospy.loginfo("Going to height " + str(object_pose.pose.position.z))
      self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
      if axis =="x":
        object_pose.pose.position.x -= approach_height * sign
      if axis =="z":
        object_pose.pose.position.z -= approach_height * sign
    return True

  def simple_place(self, robot_name, object_pose, place_height=0.05, speed_fast=0.1, speed_slow=0.02, 
      gripper_command="open", approach_height=0.05, item_id_to_detach = "", lift_up_after_place = True, acc_fast=1.0, acc_slow=.1):
    """
    A very simple place operation. item_id_to_detach is used to update the planning scene by
    removing an item that has been attached (=grasped) by the robot in the MoveIt planning scene.
    It is ignored if empty.

    This procedure works by changing the x axis of the target pose's frame. 
    It may produce dangerous motions in other configurations.
    """
    #self.publish_marker(object_pose, "place_pose")
    self.log_to_debug_monitor("Place", "operation")
    rospy.loginfo("Going above place target")
    object_pose.pose.position.x -= approach_height
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=False)
   
    rospy.loginfo("Moving to place target")
    object_pose.pose.position.x += place_height
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=False)
    object_pose.pose.position.x -= place_height
    
    #gripper open
    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command="open")

    if item_id_to_detach:
      self.groups[robot_name].detach_object(item_id_to_detach)

    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, move_lin=False)  
    return True

  def simple_grasp_sanity_check(self, grasp_pose, grasp_width=0.08, border_dist=0.08):
    """
    Returns true if the grasp pose is further than 5 cm away from the tray border,
    and no other detected objects are closer than 5 cm.

    grasp_pose is a PoseStamped.
    """
    d = self.distance_from_tray_border(grasp_pose)
    print("border distance was: ")
    print(d)
    if d[0] < border_dist or d[1] < border_dist:
      print("too close to border. discarding")
      return False
    for obj, pose in self.objects_in_tray.items():
      if obj == 6: # Hard-code skipping the belt
        continue
      if pose_dist(pose.pose, grasp_pose.pose) < 0.05:
        if pose_dist(pose.pose, grasp_pose.pose) < 1e-6:
          continue  # It's the item itself or a duplicate
        print("too close to another item. discarding. distance: " + str(pose_dist(pose.pose, grasp_pose.pose)) + ", id: " + str(obj))
        return False
    return True
  
  def is_grasp_pose_feasible(self, grasp_pose, border_dist=0.08):
    # TODO: Consider the grasp width and actual collisions using the PlanningScene
    return self.simple_grasp_sanity_check(grasp_pose, border_dist)
    
  def get_feasible_grasp_points(self, object_in_scene):
    """
    Returns a list of PoseStamped grasp points for an object that is currently in the scene.
    object_in_scene can be the string or the id number of the object.
    """
    if isinstance(object_in_scene, str):
      object_id = self.assembly_database.name_to_id(object_in_scene)
    else:
      object_id = object_in_scene

    if object_id not in self.objects_in_tray:
      rospy.logerr("Grasp points requested for " + str(object_id) + " but it is not seen in tray.")
      return False

    small_items = [8,9,10,14]
    large_items = [1,2,3,4,5,7,11,12,13]
    belt        = [6]

    if object_id in belt:
      res = self.get_3d_poses_from_ssd()
      grasp_poses = []
      for idx, pose in enumerate(res.poses):
        if res.class_ids[idx] == 6:
          if self.is_grasp_pose_feasible(pose, border_dist=0.05):
            grasp_poses.append(pose)
      return grasp_poses

    if object_id in small_items:
      # For small items, the object should be the only grasp pose.
      grasp_pose = self.objects_in_tray[object_id]
      return [grasp_pose]
      # TODO: Consider the idler spacer, which can stand upright or lie on the side.
      
    if object_id in large_items:
      # TODO: Generate alternative grasp poses
      # TODO: Get grasp poses from database
      grasp_pose = self.objects_in_tray[object_id]
      grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
      grasp_pose.pose.position.z = 0.02
      if self.is_grasp_pose_feasible(grasp_pose, border_dist=0.05):
        return [grasp_pose]
    
    return False
  
  def look_and_get_grasp_point(self, object_id):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """
    # TODO: Merge with detect_object_in_camera_view in base.py
    self.go_to_pose_goal("b_bot", self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.1)
    res = self.get_3d_poses_from_ssd()
    r2 = self.get_feasible_grasp_points(object_id)
    if r2:
      goal = r2[0]
    else:
      rospy.logerr("Could not find suitable grasp pose! Aborting.")
      return False
    return r2[0]

  def distance_from_tray_border(self, object_pose):
    """
    Returns the distance from the tray border as an (x, y) tuple.
    x, y are in the tray coordinate system.
    Distance is signed (negative is outside the tray).
    """
    # Inside tray width and length: 25.5 cm, 37.5 cm
    l_x_half = .255/2.0
    l_y_half = .375/2.0
    # print("object_pose = ")
    # print(object_pose)
    object_pose_in_world = self.listener.transformPose("tray_center", object_pose)
    xdist = l_x_half - abs(object_pose_in_world.pose.position.x)
    ydist = l_y_half - abs(object_pose_in_world.pose.position.y)
    return (xdist, ydist)
  
  def center_with_gripper(self, robot_name, object_pose, object_width, use_ur_script=False):
    """
    Centers cylindrical object by moving the gripper, by moving the robot to the pose and closing/opening.
    Rotates once and closes/opens again. Does not move back afterwards.
    """
    if use_ur_script:
      success = self.load_program(robot=robot_name, program_name="wrs2020/center_object.urp", recursion_depth=3)
      if success:
        rospy.sleep(1)
        self.execute_loaded_program(robot=robot_name)
      else:
        rospy.logerr("Problem loading. Not executing center_with_gripper.")
        return False
      wait_for_UR_program("/"+robot_name, rospy.Duration.from_sec(20))
      if self.is_robot_protective_stopped(robot_name):
        self.unlock_protective_stop(robot_name)
        return False
    object_pose_in_world = self.listener.transformPose("world", object_pose)
    object_pose_in_world.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/2, 0))
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    self.send_gripper_command(gripper=robot_name, command="close", force = 1.0, velocity = 0.1)
    self.send_gripper_command(gripper=robot_name, command=object_width+0.02, force = 90.0, velocity = 0.001)
    object_pose_in_world_rotated = copy.deepcopy(object_pose_in_world)
    object_pose_in_world_rotated.pose = rotatePoseByRPY(0,0,pi, object_pose_in_world_rotated.pose)
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    self.send_gripper_command(gripper=robot_name, command="close", force = 1.0, velocity = 0.1)
    self.send_gripper_command(gripper=robot_name, command=object_width+0.02, force = 90.0, velocity = 0.001)
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    return True

  def centering_pick(self, robot_name, pick_pose, speed_fast=0.1, speed_slow=0.02, object_width=0.08, approach_height=0.05, 
          item_id_to_attach = "", lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.1, gripper_force=40.0):
    """
    This function picks an object with the robot directly from above, but centers the object with the gripper first.
    Should be used only for cylindrical objects.
    
    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    """
    
    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    object_pose_in_world = self.listener.transformPose("world", pick_pose)
    object_pose_in_world.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    object_pose_in_world.pose.position.z += approach_height
    rospy.logdebug("Going to height " + str(object_pose_in_world.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    object_pose_in_world.pose.position.z -= approach_height

    self.send_gripper_command(gripper=robot_name, command=object_width+0.03)

    rospy.loginfo("Moving down to object")
    
    # Center object
    self.center_with_gripper(robot_name, object_pose_in_world, object_width)

    # Grasp object
    self.send_gripper_command(gripper=robot_name, command="close", force = gripper_force)

    if item_id_to_attach:
      try:
        self.groups[robot_name].attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_robotiq_85_tip_link", 
        robot_name + "_robotiq_85_left_finger_tip_link", 
        robot_name + "_robotiq_85_left_inner_knuckle_link", 
        robot_name + "_robotiq_85_right_finger_tip_link", 
        robot_name + "_robotiq_85_right_inner_knuckle_link"])
      except:
        rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

    if lift_up_after_pick:
      rospy.sleep(0.5)
      rospy.loginfo("Going back up")

      object_pose_in_world.pose.position.z += approach_height
      rospy.loginfo("Going to height " + str(object_pose_in_world.pose.position.z))
      self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    return True

  ########

  def fasten_screw(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):

    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    self.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.go_to_named_pose("feeder_pick_ready", robot_name)
    return True

  def pick_nut(self, robot_name):
    """Pick the nut from the holder. The nut tool has to be equipped.
    Use this command to equip: do_change_tool_action(self, "a_bot", equip=True, screw_size = 66)"""
    rospy.logerr("Not implemented yet")
    return False
    
    self.go_to_named_pose("home", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("nut_pick_ready", "a_bot", speed=1.0, acceleration=1.0, force_ur_script=self.use_real_robot)

    nut_pose = geometry_msgs.msg.PoseStamped()
    nut_pose.header.frame_id = "nut_holder_collar_link"
    nut_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    approach_pose = copy.deepcopy(nut_pose)
    approach_pose.pose.position.x -= .03
    self.go_to_pose_goal(robot_name, approach_pose, speed=self.speed_fast, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    # spiral_axis = "Y"
    # push_direction = "Z+"
    # self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.go_to_pose_goal(robot_name, nut_pose, speed=.005, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    rospy.sleep(3)
    # self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.go_to_pose_goal(robot_name, approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def move_camera_to_pose(self, pose_goal, robot_name="b_bot", camera_name="inside_camera"):
    return self.go_to_pose_goal(robot_name, pose_goal, end_effector_link=robot_name+"_"+camera_name+"_color_optical_frame")

  def jigless_recenter(self, robot_carrying_the_item):
      pass
