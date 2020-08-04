#include "o2ac_skill_server.h"

SkillServer::SkillServer() : 
                  pickScrewActionServer_(n_, "o2ac_skills/pick_screw", boost::bind(&SkillServer::executePickScrew, this, _1),false),
                  placeActionServer_(n_, "o2ac_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  regraspActionServer_(n_, "o2ac_skills/regrasp", boost::bind(&SkillServer::executeRegrasp, this, _1),false),
                  screwActionServer_(n_, "o2ac_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  changeToolActionServer_(n_, "o2ac_skills/change_tool", boost::bind(&SkillServer::executeChangeTool, this, _1),false),
                  b_bot_group_("b_bot"),
                  a_bot_gripper_client_("/a_bot/gripper_action_controller", true),
                  b_bot_gripper_client_("/b_bot/gripper_action_controller", true),
                  fastening_tool_client("/screw_tool_control", true),
                  suction_client("/suction_control", true)
{ 
  // Topics to publish
  pubMarker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Topics to subscribe to
  subRunMode_ = n_.subscribe("run_mode", 1, &SkillServer::runModeCallback, this);
  subPauseMode_ = n_.subscribe("pause_mode", 1, &SkillServer::pauseModeCallback, this);
  subTestMode_ = n_.subscribe("test_mode", 1, &SkillServer::testModeCallback, this);

  // Services to advertise
  goToNamedPoseService_ = n_.advertiseService("o2ac_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);
  publishMarkerService_ = n_.advertiseService("o2ac_skills/publishMarker", &SkillServer::publishMarkerCallback,
                                        this);
  toggleCollisionsService_ = n_.advertiseService("o2ac_skills/toggleCollisions", &SkillServer::toggleCollisionsCallback,
                                        this);

  // Services to subscribe to
  sendScriptToURClient_ = n_.serviceClient<o2ac_msgs::sendScriptToUR>("o2ac_skills/sendScriptToUR");

  a_bot_get_loaded_program_ = n_.serviceClient<ur_dashboard_msgs::GetLoadedProgram>("/a_bot/ur_hardware_interface/dashboard/get_loaded_program");
  a_bot_program_running_ = n_.serviceClient<ur_dashboard_msgs::IsProgramRunning>("/a_bot/ur_hardware_interface/dashboard/program_running");
  a_bot_load_program_ = n_.serviceClient<ur_dashboard_msgs::Load>("/a_bot/ur_hardware_interface/dashboard/load_program");
  a_bot_play_ = n_.serviceClient<std_srvs::Trigger>("/a_bot/ur_hardware_interface/dashboard/play");
  b_bot_get_loaded_program_ = n_.serviceClient<ur_dashboard_msgs::GetLoadedProgram>("/b_bot/ur_hardware_interface/dashboard/get_loaded_program");
  b_bot_program_running_ = n_.serviceClient<ur_dashboard_msgs::IsProgramRunning>("/b_bot/ur_hardware_interface/dashboard/program_running");
  b_bot_load_program_ = n_.serviceClient<ur_dashboard_msgs::Load>("/b_bot/ur_hardware_interface/dashboard/load_program");
  b_bot_play_ = n_.serviceClient<std_srvs::Trigger>("/b_bot/ur_hardware_interface/dashboard/play");
  
  // Actions we serve
  pickScrewActionServer_.start();
  placeActionServer_.start();
  regraspActionServer_.start();
  screwActionServer_.start();
  changeToolActionServer_.start();

  // Action clients
  // ROS_INFO("Waiting for action servers to start.");
  // a_bot_gripper_client.waitForServer(); 
  // b_bot_gripper_client_.waitForServer();
  // ROS_INFO("Action servers started.");

  // Set up MoveGroups
  b_bot_group_.setPlanningTime(PLANNING_TIME);
  b_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  b_bot_group_.setEndEffectorLink("b_bot_robotiq_85_tip_link");
  b_bot_group_.setNumPlanningAttempts(5);

  get_planning_scene_client = n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  // Get the planning scene of the movegroup
  updatePlanningScene();
  initializeCollisionObjects();

  // Initialize robot statuses
  o2ac_msgs::RobotStatus r1, r2;
  robot_statuses_["a_bot"] = r1;
  robot_statuses_["b_bot"] = r2;
  
  n_.getParam("use_real_robot", use_real_robot_);
  ROS_INFO_STREAM((use_real_robot_ ? "Using real robot!" : "Using simulated robot."));
  if (use_real_robot_)
  {
    ROS_INFO_STREAM("Using real robot!");
  }
  else
  {
    ROS_INFO_STREAM("Using simulated robot.");
  }
}

void SkillServer::initializeCollisionObjects()
{
  // --- Define the tools as collision objects, so they can be used for planning
  // THIS IS OUTDATED AND NOW DEFINED IN YAML FILES.
  
  //M4 tool
  screw_tool_m4.header.frame_id = "screw_tool_m4_link";
  screw_tool_m4.id = "screw_tool_m4";

  screw_tool_m4.primitives.resize(3);
  screw_tool_m4.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m4.primitives[0].type = screw_tool_m4.primitives[0].BOX;
  screw_tool_m4.primitives[0].dimensions.resize(3);
  screw_tool_m4.primitives[0].dimensions[0] = 0.026;
  screw_tool_m4.primitives[0].dimensions[1] = 0.04;
  screw_tool_m4.primitives[0].dimensions[2] = 0.055;
  screw_tool_m4.primitive_poses[0].position.x = 0;
  screw_tool_m4.primitive_poses[0].position.y = -0.009;
  screw_tool_m4.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m4.primitives[1].type = screw_tool_m4.primitives[1].BOX;
  screw_tool_m4.primitives[1].dimensions.resize(3);
  screw_tool_m4.primitives[1].dimensions[0] = 0.02;
  screw_tool_m4.primitives[1].dimensions[1] = 0.03;
  screw_tool_m4.primitives[1].dimensions[2] = 0.08;
  screw_tool_m4.primitive_poses[1].position.x = 0;
  screw_tool_m4.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m4.primitives[2].type = screw_tool_m4.primitives[2].CYLINDER;
  screw_tool_m4.primitives[2].dimensions.resize(2);
  screw_tool_m4.primitives[2].dimensions[0] = 0.038;    // Cylinder height
  screw_tool_m4.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m4.primitive_poses[2].position.x = 0;
  screw_tool_m4.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[2].position.z = -0.099;
  screw_tool_m4.operation = screw_tool_m4.ADD;

  // The tool tip
  screw_tool_m4.subframe_poses.resize(1);
  screw_tool_m4.subframe_names.resize(1);
  screw_tool_m4.subframe_poses[0].position.z = -.12;
  screw_tool_m4.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  screw_tool_m4.subframe_names[0] = "screw_tool_m4_tip";


  //M3 tool
  screw_tool_m3.header.frame_id = "screw_tool_m3_link";
  screw_tool_m3.id = "screw_tool_m3";

  screw_tool_m3.primitives.resize(3);
  screw_tool_m3.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m3.primitives[0].type = screw_tool_m3.primitives[0].BOX;
  screw_tool_m3.primitives[0].dimensions.resize(3);
  screw_tool_m3.primitives[0].dimensions[0] = 0.026;
  screw_tool_m3.primitives[0].dimensions[1] = 0.04;
  screw_tool_m3.primitives[0].dimensions[2] = 0.055;
  screw_tool_m3.primitive_poses[0].position.x = 0;
  screw_tool_m3.primitive_poses[0].position.y = -0.009;
  screw_tool_m3.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m3.primitives[1].type = screw_tool_m3.primitives[1].BOX;
  screw_tool_m3.primitives[1].dimensions.resize(3);
  screw_tool_m3.primitives[1].dimensions[0] = 0.02;
  screw_tool_m3.primitives[1].dimensions[1] = 0.03;
  screw_tool_m3.primitives[1].dimensions[2] = 0.08;
  screw_tool_m3.primitive_poses[1].position.x = 0;
  screw_tool_m3.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m3.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m3.primitives[2].type = screw_tool_m3.primitives[2].CYLINDER;
  screw_tool_m3.primitives[2].dimensions.resize(2);
  screw_tool_m3.primitives[2].dimensions[0] = 0.018;    // Cylinder height
  screw_tool_m3.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m3.primitive_poses[2].position.x = 0;
  screw_tool_m3.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m3.primitive_poses[2].position.z = -0.089;
  screw_tool_m3.operation = screw_tool_m3.ADD;

  // The tool tip
  screw_tool_m3.subframe_poses.resize(1);
  screw_tool_m3.subframe_names.resize(1);
  screw_tool_m3.subframe_poses[0].position.z = -.11;
  screw_tool_m3.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  screw_tool_m3.subframe_names[0] = "screw_tool_m3_tip";

  //Suction tool
  suction_tool.header.frame_id = "suction_tool_link";
  suction_tool.id = "suction_tool";

  suction_tool.primitives.resize(2);
  suction_tool.primitive_poses.resize(2);
  // The upper box
  suction_tool.primitives[0].type = suction_tool.primitives[0].BOX;
  suction_tool.primitives[0].dimensions.resize(3);
  suction_tool.primitives[0].dimensions[0] = 0.03;
  suction_tool.primitives[0].dimensions[1] = 0.06;
  suction_tool.primitives[0].dimensions[2] = 0.04;
  suction_tool.primitive_poses[0].position.x = 0;
  suction_tool.primitive_poses[0].position.y = 0.02;
  suction_tool.primitive_poses[0].position.z = 0.02;

  // The cylinder representing the tip
  suction_tool.primitives[1].type = suction_tool.primitives[1].CYLINDER;
  suction_tool.primitives[1].dimensions.resize(2);
  suction_tool.primitives[1].dimensions[0] = 0.1;    // Cylinder height
  suction_tool.primitives[1].dimensions[1] = 0.004;   // Cylinder radius
  suction_tool.primitive_poses[1].position.x = 0;
  suction_tool.primitive_poses[1].position.y = 0.02;  // 21 mm distance from axis
  suction_tool.primitive_poses[1].position.z = -0.05;
  suction_tool.operation = suction_tool.ADD;

  // The tool tip
  suction_tool.subframe_poses.resize(1);
  suction_tool.subframe_names.resize(1);
  suction_tool.subframe_poses[0].position.z = -.1;
  suction_tool.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  suction_tool.subframe_names[0] = "suction_tool_tip";

  // ==== Nut tool M6
  // Note: Y points "forward" to the front of the holder
  nut_tool.header.frame_id = "nut_tool_link";
  nut_tool.id = "nut_tool";

  nut_tool.primitives.resize(2);
  nut_tool.primitive_poses.resize(2);
  // The upper box
  nut_tool.primitives[0].type = nut_tool.primitives[0].BOX;
  nut_tool.primitives[0].dimensions.resize(3);
  nut_tool.primitives[0].dimensions[0] = 0.059;
  nut_tool.primitives[0].dimensions[1] = 0.032;
  nut_tool.primitives[0].dimensions[2] = 0.052;
  nut_tool.primitive_poses[0].position.x = 0;
  nut_tool.primitive_poses[0].position.y = -.0115;  // 59/2 mm - 15.5 mm
  nut_tool.primitive_poses[0].position.z = 0.0275;

  // The cylinder with the tooltip
  nut_tool.primitives[1].type = nut_tool.primitives[1].CYLINDER;
  nut_tool.primitives[1].dimensions.resize(2);
  nut_tool.primitives[1].dimensions[0] = 0.011;    // Cylinder height
  nut_tool.primitives[1].dimensions[1] = 0.011;   // Cylinder radius
  nut_tool.primitive_poses[1].position.z = -0.055;
  nut_tool.operation = nut_tool.ADD;

  // The tool tip
  nut_tool.subframe_poses.resize(1);
  nut_tool.subframe_names.resize(1);
  nut_tool.subframe_poses[0].position.z = -.11;
  nut_tool.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  nut_tool.subframe_names[0] = "nut_tool_tip";

  // ==== Set screw tool
  set_screw_tool.header.frame_id = "set_screw_tool_link";
  set_screw_tool.id = "set_screw_tool";

  set_screw_tool.primitives.resize(3);
  set_screw_tool.primitive_poses.resize(3);
  // The upper box
  set_screw_tool.primitives[0].type = set_screw_tool.primitives[0].BOX;
  set_screw_tool.primitives[0].dimensions.resize(3);
  set_screw_tool.primitives[0].dimensions[0] = 0.059; // Box length
  set_screw_tool.primitives[0].dimensions[1] = 0.032; // Box width
  set_screw_tool.primitives[0].dimensions[2] = 0.052; // Box height
  set_screw_tool.primitive_poses[0].position.x = 0;
  set_screw_tool.primitive_poses[0].position.y = -.0115;  // 59/2 mm - 15.5 mm
  set_screw_tool.primitive_poses[0].position.z = 0.0275;

  // The cylinder holding the screw bit
  set_screw_tool.primitives[1].type = set_screw_tool.primitives[1].CYLINDER;
  set_screw_tool.primitives[1].dimensions.resize(2);
  set_screw_tool.primitives[1].dimensions[0] = 0.008;    // Cylinder height
  set_screw_tool.primitives[1].dimensions[1] = 0.008;   // Cylinder radius
  set_screw_tool.primitive_poses[1].position.z = -0.04;
  set_screw_tool.operation = set_screw_tool.ADD;

  // The screw bit (approximated (I wish it was a cone (could be done with moveit_visual_tools)))
  set_screw_tool.primitives[2].type = set_screw_tool.primitives[2].CYLINDER;
  set_screw_tool.primitives[2].dimensions.resize(2);
  set_screw_tool.primitives[2].dimensions[0] = 0.02;    // Cylinder height
  set_screw_tool.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  set_screw_tool.primitive_poses[2].position.z = -0.18;
  set_screw_tool.operation = set_screw_tool.ADD;

  // The tool tip
  set_screw_tool.subframe_poses.resize(1);
  set_screw_tool.subframe_names.resize(1);
  set_screw_tool.subframe_poses[0].position.y = -.0015;   // Offset because the bit's tip is inclined. Magic number.
  set_screw_tool.subframe_poses[0].position.z = -.028;
  set_screw_tool.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  set_screw_tool.subframe_names[0] = "set_screw_tool_tip";
}

bool SkillServer::activateROSControlOnUR(std::string robot_name)
{
  if (!use_real_robot_)
    return true;
  if ((robot_name != "b_bot") && (robot_name != "a_bot"))
  {
    ROS_ERROR("Robot name was not found or the robot is not a UR!");
    return false;
  }

  ros::ServiceClient get_loaded_program_, program_running_, load_program_, play_;
  if (robot_name == "a_bot")
  {
    get_loaded_program_ = a_bot_get_loaded_program_;
    program_running_ = a_bot_program_running_;
    load_program_ = a_bot_load_program_;
    play_ = a_bot_play_;
  }
  else // b_bot
  {
    get_loaded_program_ = b_bot_get_loaded_program_;
    program_running_ = b_bot_program_running_;
    load_program_ = b_bot_load_program_;
    play_ = b_bot_play_;
  }
  ros::Duration(1.0).sleep();
  
  // Check if URCap is already running on UR
  ur_dashboard_msgs::IsProgramRunning srv1;  
  if (srv1.response.program_running)
  {
    ur_dashboard_msgs::GetLoadedProgram srv2;
    get_loaded_program_.call(srv2);
    ROS_WARN_STREAM("Got loaded program name:");
    ROS_WARN_STREAM(srv2.response.program_name);
    if (srv2.response.program_name == "/programs/ROS_external_control.urp")
      return true;
  }

  // Load program
  ur_dashboard_msgs::Load srv3;
  srv3.request.filename = "ROS_external_control.urp";
  load_program_.call(srv3);
  if (!srv3.response.success)
  {
    ROS_ERROR("Could not load the ROS_external_control.urp URCap. Is the UR robot set up correctly and the program installed with the correct name?");
    ROS_ERROR_STREAM("service answer: " << srv3.response.answer);
    return false;
  }

  // Run the program
  std_srvs::Trigger srv4;
  play_.call(srv4);
  ros::Duration(2.0).sleep();
  return srv4.response.success;
}

bool SkillServer::moveToJointPose(std::vector<double> joint_positions, std::string robot_name, bool wait, double velocity_scaling_factor, bool use_UR_script, double acceleration)
{
  if (pause_mode_ || test_mode_)
  {
    if (velocity_scaling_factor > reduced_speed_limit_)
    {
      ROS_INFO_STREAM("Reducing velocity_scaling_factor from " << velocity_scaling_factor << " to " << reduced_speed_limit_ << " because robot is in test or pause mode!");
      velocity_scaling_factor = reduced_speed_limit_;
    }
  }
  if (joint_positions.size() != 6)
  {
    ROS_ERROR_STREAM("Size of joint positions in moveToJointPose is not correct! Expected 6, got " << joint_positions.size());
    return false;
  }
  if (use_UR_script && use_real_robot_)
  {
    o2ac_msgs::sendScriptToUR UR_srv;
    UR_srv.request.program_id = "move_j";
    UR_srv.request.robot_name = robot_name;
    UR_srv.request.joint_positions = joint_positions;
    UR_srv.request.velocity = velocity_scaling_factor;
    UR_srv.request.acceleration = acceleration;
    sendScriptToURClient_.call(UR_srv);
    if (UR_srv.response.success == true)
    {
      ROS_INFO("Successfully called the URScript client to move joints.");
      ros::Duration(1.0).sleep();
      return waitForURProgram("/" + robot_name);
    }
    else
    {
      ROS_ERROR("Could not move joints with URscript.");
      return false;
    }
  }

  if (!activateROSControlOnUR(robot_name))
  {
    ROS_ERROR("Could not activate robot. Aborting move.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface* group_pointer = robotNameToMoveGroup(robot_name);;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success, motion_done;
  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor);
  group_pointer->setJointValueTarget(joint_positions);
    
  success = (group_pointer->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    motion_done = group_pointer->execute(my_plan);
    return (motion_done == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  else
  {
    ROS_ERROR("Could not plan to before_tool_pickup joint state. Abort!");
    return false;
  }
}

// This works only for a single robot.
bool SkillServer::moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor)
{
  if (pause_mode_ || test_mode_)
  {
    if (velocity_scaling_factor > reduced_speed_limit_)
    {
      ROS_INFO_STREAM("Reducing velocity_scaling_factor from " << velocity_scaling_factor << " to " << reduced_speed_limit_ << " because robot is in test or pause mode!");
      velocity_scaling_factor = reduced_speed_limit_;
    }
  }

  if (!activateROSControlOnUR(robot_name))
  {
    ROS_ERROR("Could not activate robot. Aborting move.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);
  
  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor);  // TODO: Check if this works
  if (end_effector_link == "")  // Define default end effector link explicitly
  {
    if (robot_name == "a_bot") group_pointer->setEndEffectorLink("a_bot_gripper_tip_link");
    else group_pointer->setEndEffectorLink(robot_name + "_robotiq_85_tip_link");
  }
  else group_pointer->setEndEffectorLink(end_effector_link);
  group_pointer->setPoseTarget(pose);

  ROS_INFO_STREAM("Planning motion for robot " << robot_name << " and EE link " << end_effector_link + "_tip_link, to pose:");
  ROS_INFO_STREAM(pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z);
  success_plan = group_pointer->plan(myplan);
  if (success_plan) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) {
      group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
      return true;
    }
  }
  ROS_WARN("Failed to perform motion.");
  group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
  return false;
}

// This works only for a single robot.
// Acceleration is only used for the real robot.
bool SkillServer::moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor, double acceleration, bool force_UR_script, bool force_moveit)
{
  if (pause_mode_ || test_mode_)
  {
    if (velocity_scaling_factor > reduced_speed_limit_)
    {
      ROS_INFO_STREAM("Reducing velocity_scaling_factor from " << velocity_scaling_factor << " to " << reduced_speed_limit_ << " because robot is in test or pause mode!");
      velocity_scaling_factor = reduced_speed_limit_;
    }
  }
  if (use_real_robot_ || force_UR_script)
  {
    if (!force_moveit)
    {
      if (end_effector_link == "")
      {
        if (robot_name == "c_bot")
          end_effector_link = "c_bot_robotiq_85_tip_link";
        else if (robot_name == "b_bot")
          end_effector_link = "b_bot_robotiq_85_tip_link";
        else if (robot_name == "a_bot")
          end_effector_link = "a_bot_gripper_tip_link";
      }
      ROS_DEBUG("Real robot is being used. Sending linear motion to robot controller directly via URScript.");
      o2ac_msgs::sendScriptToUR UR_srv;
      UR_srv.request.program_id = "lin_move";
      UR_srv.request.robot_name = robot_name;  
      UR_srv.request.target_pose = transformTargetPoseFromTipLinkToURTCP(pose, robot_name, end_effector_link, tflistener_);
      publishMarker(transformTargetPoseFromTipLinkToURTCP(pose, robot_name, end_effector_link, tflistener_), "pose");
      UR_srv.request.velocity = velocity_scaling_factor;
      UR_srv.request.acceleration = acceleration;
      sendScriptToURClient_.call(UR_srv);
      if (UR_srv.response.success == true)
      {
        ROS_DEBUG("Successfully called the URScript client to do linear motion.");
        ros::Duration(1.0).sleep();
        waitForURProgram("/" + robot_name);
        return true;
      }
      else
      {
        ROS_ERROR("Could not go LIN to pose via UR script.");
        return false;
      }
    }
  }
  
  if (!activateROSControlOnUR(robot_name))
  {
    ROS_ERROR("Could not activate robot. Aborting move.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->clearPoseTargets();
  group_pointer->setPoseReferenceFrame("world");
  group_pointer->setStartStateToCurrentState();
  group_pointer->setEndEffectorLink(end_effector_link);
  
  // Plan cartesian motion
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped start_pose, end_pose;
  if (end_effector_link == "") {
    if (robot_name == "a_bot") start_pose.header.frame_id = "a_bot_gripper_tip_link";
    else start_pose.header.frame_id = robot_name + "_robotiq_85_tip_link";
  }
  else start_pose.header.frame_id = end_effector_link;
  start_pose.pose = makePose();
  start_pose = transform_pose_now(start_pose, "world", tflistener_);
  end_pose = transform_pose_now(pose, "world", tflistener_);
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(end_pose.pose);

  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor); // This doesn't work for linear paths: https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/
  b_bot_group_.setPlanningTime(LIN_PLANNING_TIME);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  ros::Time start_time = ros::Time::now();
  double cartesian_success = group_pointer->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ros::Duration d = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Cartesian motion plan took " << d.toSec() << " s and was " << cartesian_success * 100.0 << "% successful.");

  // Scale the trajectory. This is a workaround to setting the VelocityScalingFactor. Copied from k-okada
  if (cartesian_success > 0.95)
  {
    moveit_msgs::RobotTrajectory scaled_trajectory = moveit_msgs::RobotTrajectory(trajectory);
    // Scaling (https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4)
    // The trajectory needs to be modified so it will include velocities as well.
    // First: create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group_pointer->getRobotModel(), group_pointer->getName());
    // Second: get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group_pointer->getCurrentState(), scaled_trajectory);
    // Third: create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth: compute computeTimeStamps
    bool success =
        iptp.computeTimeStamps(rt, velocity_scaling_factor, velocity_scaling_factor);  // The second value is actually acceleration
    ROS_INFO_STREAM("Computing time stamps for iptp scaling with factor " << velocity_scaling_factor << ": " << (success ? "SUCCEEDED" : "FAILED"));
    for (int i = 0; i < scaled_trajectory.joint_trajectory.points.size(); i++)
      ROS_WARN_STREAM(scaled_trajectory.joint_trajectory.points[i].time_from_start.toSec() << " " << scaled_trajectory.joint_trajectory.points[i].positions[2]);
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(scaled_trajectory);
    if (scaled_trajectory.joint_trajectory.points[0].time_from_start.toSec() == scaled_trajectory.joint_trajectory.points[1].time_from_start.toSec()){
      scaled_trajectory.joint_trajectory.points.erase(scaled_trajectory.joint_trajectory.points.begin());
    }
    // Fill in move_group_
    myplan.trajectory_ = scaled_trajectory;
  
    if (true) 
    {
      if (wait) motion_done = group_pointer->execute(myplan);
      else motion_done = group_pointer->asyncExecute(myplan);
      if (motion_done) 
      {
        group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
        group_pointer->setPlanningTime(1.0);
        if (cartesian_success > .95) return true;
        else return false;
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("Cartesian motion plan failed.");
    group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
    group_pointer->setPlanningTime(1.0);
    return false;
  }
}

bool SkillServer::goToNamedPose(std::string pose_name, std::string robot_name, double speed, double acceleration, bool use_UR_script)
{
  ROS_INFO_STREAM("Going to named pose " << pose_name << " with robot group " << robot_name << ".");
  if (pause_mode_ || test_mode_)
  {
    if (speed > reduced_speed_limit_)
    {
      ROS_INFO_STREAM("Reducing speed from " << speed << " to " << reduced_speed_limit_ << " because robot is in test or pause mode!");
      speed = reduced_speed_limit_;
    }
  }

  if (!activateROSControlOnUR(robot_name))
  {
    ROS_ERROR("Could not activate robot. Aborting move.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);
  if (use_UR_script && use_real_robot_)
  {
    std::map<std::string, double> d = group_pointer->getNamedTargetValues(pose_name);
    std::vector<double> joint_pose = {d[robot_name+"_shoulder_pan_joint"], 
                                      d[robot_name+"_shoulder_lift_joint"],
                                      d[robot_name+"_elbow_joint"],
                                      d[robot_name+"_wrist_1_joint"],
                                      d[robot_name+"_wrist_2_joint"],
                                      d[robot_name+"_wrist_3_joint"] };
    return moveToJointPose(joint_pose, robot_name, true, speed, use_UR_script, acceleration);
  }
  if (speed > 1.0)
    speed = 1.0;

  group_pointer->setStartStateToCurrentState();
  group_pointer->setNamedTarget(pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;
  
  success_plan = group_pointer->move();
  
  return true;
}

bool SkillServer::stop()
{
  return true;
}

moveit::planning_interface::MoveGroupInterface* SkillServer::robotNameToMoveGroup(std::string robot_name)
{
  // This function converts the name of the robot to a pointer to the member variable containing the move group
  // Returning the move group itself does not seem to work, sadly.
  if (robot_name == "b_bot") return &b_bot_group_;
}

std::string SkillServer::getEELink(std::string robot_name)
{
  std::string ee_link_name;
  moveit::planning_interface::MoveGroupInterface* group_pointer = robotNameToMoveGroup(robot_name);
  ee_link_name = group_pointer->getEndEffectorLink();
  if (ee_link_name == "")
  {
    ROS_ERROR("Requested end effector was returned empty!");
  }
  return ee_link_name;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(std::string robot_name, std::string screw_tool_id)
{
  ROS_INFO_STREAM("Equipping screw tool " << screw_tool_id);
  return equipUnequipScrewTool(robot_name, screw_tool_id, "equip");
}

bool SkillServer::unequipScrewTool(std::string robot_name)
{
  ROS_INFO_STREAM("Unequipping screw tool " << held_screw_tool_);
  return equipUnequipScrewTool(robot_name, held_screw_tool_, "unequip");
}

bool SkillServer::equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip)
{
  // Sanity check on the input instruction
  bool equip = (equip_or_unequip == "equip");
  bool unequip = (equip_or_unequip == "unequip");
  double lin_speed = 0.01;
  // The second comparison is not always necessary, but readability comes first.
  if ((!equip) && (!unequip))
  {
    ROS_ERROR_STREAM("Cannot read the instruction " << equip_or_unequip << ". Returning false.");
    return false;
  }

  if ((robot_statuses_[robot_name].carrying_object == true))
  {
    ROS_ERROR_STREAM("Robot holds an object. Cannot " << equip_or_unequip << " screw tool.");
    return false;
  }
  if ( (robot_statuses_[robot_name].carrying_tool == true) && (equip))
  {
    ROS_ERROR_STREAM("Robot already holds a tool. Cannot equip another.");
    return false;
  }
  if ( (robot_statuses_[robot_name].carrying_tool == false) && (unequip))
  {
    ROS_ERROR_STREAM("Robot is not holding a tool. Cannot unequip any.");
    return false;
  }
  
  // ==== STEP 0: Set up poses
  geometry_msgs::PoseStamped ps_approach, ps_tool_holder, ps_move_away, ps_high_up, ps_end;
  ps_approach.header.frame_id = screw_tool_id + "_pickup_link";

  // Define approach pose
  // z = 0 is at the holder surface, and z-axis of pickup_link points downwards!
  ps_approach.pose.position.x = -.06;
  ps_approach.pose.position.z = -.008;
  ROS_INFO_STREAM("screw_tool_id: " << screw_tool_id);
  if (screw_tool_id == "nut_tool_m6")
    ps_approach.pose.position.z = -.025;
  else if (screw_tool_id == "set_screw_tool")
    ps_approach.pose.position.z = -.025;
  else if (screw_tool_id == "suction_tool")
  {
    ROS_ERROR("Suction tool is not implemented!");
    return false;
  }

  ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/6, 0);
  ps_move_away = ps_approach;

  // Define pickup pose
  ps_tool_holder = ps_approach;
  ps_tool_holder.pose.position.x = 0.017;
  if (screw_tool_id == "nut_tool_m6")
    ps_tool_holder.pose.position.x = 0.01;
  else if (screw_tool_id == "set_screw_tool")
    ps_tool_holder.pose.position.x = 0.02;
    
  if (unequip) 
  {
    ps_tool_holder.pose.position.x -= 0.001; // Don't move all the way into the magnet
    ps_approach.pose.position.z -= 0.01; // Approach diagonally so nothing gets stuck
  }

  ps_high_up = ps_approach;
  ps_end = ps_high_up;


  // STEP 2: Move to keypose in front of tools, go to approach pose, move to final pose, 
  // retreat to approach pose, then back to keypose
  moveit::planning_interface::MoveGroupInterface* group_pointer;
  ROS_INFO("Going to tool_pick_ready.");
  
  if (!goToNamedPose("tool_pick_ready", robot_name, 0.2, 0.2, false))
  {
    ROS_ERROR("Could not plan to before_tool_pickup joint state. Abort!");
    return false;
  }

  if (equip) {
    openGripper(robot_name);
    // ROS_INFO("Spawning tool.");
    // spawnTool(screw_tool_id);
    held_screw_tool_ = screw_tool_id;
  }

  // Disable all collisions to allow movement into the tool
  // NOTE: This could be done cleaner by disabling only gripper + tool, but it is good enough for now.
  updatePlanningScene();
  ROS_INFO("Disabling all collisions. Updating collision matrix.");
  collision_detection::AllowedCollisionMatrix acm_no_collisions(planning_scene_.allowed_collision_matrix),
                                              acm_original(planning_scene_.allowed_collision_matrix);
  acm_no_collisions.setEntry(screw_tool_id, true);   // Allow collisions with screw tool during pickup,
  acm_original.setEntry(screw_tool_id, false); // but not afterwards.
  std::vector<std::string> entries;
  acm_no_collisions.getAllEntryNames(entries);
  for (auto i : entries)
  {
    acm_no_collisions.setEntry(i, true);
  }
  moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
  acm_no_collisions.getMessage(ps_no_collisions.allowed_collision_matrix);
  planning_scene_interface_.applyPlanningScene(ps_no_collisions);


  ROS_INFO("Moving to screw tool approach pose LIN.");
  bool preparation_succeeded = moveToCartPoseLIN(ps_approach, robot_name, true, robot_name + "_robotiq_85_tip_link", 0.2, 0.2, use_real_robot_, true);
  if (!preparation_succeeded)
  {
    ROS_ERROR("Could not go to approach pose. Aborting tool pickup.");
    planning_scene_interface_.applyPlanningScene(planning_scene_);
    return false;
  }

  ROS_INFO("Moving to pose in tool holder LIN.");
  bool moved_to_tool_holder = true;

  if (equip)        lin_speed = 0.5;
  else if (unequip) lin_speed = 0.08;  
  moved_to_tool_holder = moveToCartPoseLIN(ps_tool_holder, robot_name, true, "", lin_speed, lin_speed, use_real_robot_, true);
  
  if (!moved_to_tool_holder)
  {
    ROS_ERROR("Was not able to move to tool holder. ABORTING!");
    return false;
  }

  // Close gripper, attach the tool object to the gripper in the Planning Scene.
  // Its collision with the parent link is set to allowed in the original planning scene.
  if (equip)
  {
    closeGripper(robot_name);
    attachTool(screw_tool_id, robot_name);
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_left_finger_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_left_inner_knuckle_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_right_finger_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_right_inner_knuckle_link", true);  // For afterwards
    
    acm_no_collisions.setEntry(screw_tool_id, true);      // To allow collisions now
    planning_scene_interface_.applyPlanningScene(ps_no_collisions);
    
    robot_statuses_[robot_name].carrying_tool = true;
    robot_statuses_[robot_name].held_tool_id = screw_tool_id;
  }
  else if (unequip) 
  {
    openGripper(robot_name);
    detachTool(screw_tool_id, robot_name);
    held_screw_tool_ = "";
    acm_original.removeEntry(screw_tool_id);
    robot_statuses_[robot_name].carrying_tool = false;
    robot_statuses_[robot_name].held_tool_id = "";
  }
  acm_original.getMessage(planning_scene_.allowed_collision_matrix);
  ros::Duration(.5).sleep();
  
  // Plan & execute linear motion away from the tool change position
  ROS_INFO("Moving back to screw tool approach pose LIN.");
  if (equip)        lin_speed = 1.0;
  else if (unequip) lin_speed = 1.0;

  moveToCartPoseLIN(ps_move_away, robot_name, true, "", lin_speed, lin_speed, use_real_robot_, true);
  
  // Reactivate the collisions, with the updated entry about the tool
  planning_scene_interface_.applyPlanningScene(planning_scene_);

  ROS_INFO("Moving back to tool_pick_ready.");
  goToNamedPose("tool_pick_ready", robot_name, 0.2, 0.2, false);
  
  // Delete tool collision object only after collision reinitialization to avoid errors
  // if (unequip) despawnTool(screw_tool_id);

  return true;
}

// This forces a refresh of the planning scene.
bool SkillServer::updatePlanningScene()
{
  moveit_msgs::GetPlanningScene srv;
  // Request only the collision matrix
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  get_planning_scene_client.call(srv);
  if (get_planning_scene_client.call(srv))
  {
    ROS_INFO("Got planning scene from move group.");
    planning_scene_ = srv.response.scene;
    planning_scene_.is_diff = true;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to get planning scene from move group.");
    return false;
  }
}


bool SkillServer::toggleCollisions(bool collisions_on)
{
  if (!collisions_on)
  {
    updatePlanningScene();
    ROS_INFO("Disabling all collisions.");
    collision_detection::AllowedCollisionMatrix acm_no_collisions(planning_scene_.allowed_collision_matrix),
                                                acm_original(planning_scene_.allowed_collision_matrix);
    std::vector<std::string> entries;
    acm_no_collisions.getAllEntryNames(entries);
    for (auto i : entries)
    {
      acm_no_collisions.setEntry(i, true);
    }
    moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
    acm_no_collisions.getMessage(ps_no_collisions.allowed_collision_matrix);
    planning_scene_interface_.applyPlanningScene(ps_no_collisions);
  }
  else
  {
    ROS_INFO("Reenabling collisions with the scene as remembered.");
    planning_scene_interface_.applyPlanningScene(planning_scene_);
  }
  return true;
}


bool SkillServer::openGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.085, gripper_name);
}

bool SkillServer::closeGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.0, gripper_name);
}

bool SkillServer::sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name)
{
  bool finished_before_timeout;
  ROS_INFO_STREAM("Sending opening_width " << opening_width << " to gripper of: " << robot_name);
  if ((robot_name == "a_bot") || (robot_name == "b_bot"))
  {
    // Send a goal to the action
    robotiq_msgs::CModelCommandGoal goal;
    robotiq_msgs::CModelCommandResultConstPtr result;

    goal.position = opening_width;    // Opening width. 0 to close, 0.085 to open the gripper.
    goal.velocity = 0.1;              // From 0.013 to 0.1
    goal.force = 100;                 // From 40 to 100
    if (robot_name == "a_bot")
    {
      a_bot_gripper_client_.sendGoal(goal);
      ros::Duration(0.5).sleep();
      finished_before_timeout = a_bot_gripper_client_.waitForResult(ros::Duration(4.0));
      result = a_bot_gripper_client_.getResult();
    }
    else if (robot_name == "b_bot")
    {
      b_bot_gripper_client_.sendGoal(goal);
      ros::Duration(0.5).sleep();
      finished_before_timeout = b_bot_gripper_client_.waitForResult(ros::Duration(2.0));
      result = b_bot_gripper_client_.getResult();
    }
    ROS_DEBUG_STREAM("Action " << (finished_before_timeout ? "returned" : "did not return before timeout") <<", with result: " << result->reached_goal);
  }
  else
  {
    ROS_ERROR("The specified gripper is not defined!");
    return false;
  }
  ROS_DEBUG("Returning from gripper command.");
  return finished_before_timeout;
}

bool SkillServer::sendFasteningToolCommand(std::string fastening_tool_name, std::string direction, bool wait, double duration, int speed)
{
  if (!use_real_robot_)
    return true;
  // Send a goal to the action
  o2ac_msgs::FastenerGripperControlGoal goal;
  o2ac_msgs::FastenerGripperControlResultConstPtr result;

  ROS_INFO_STREAM("Requesting " << fastening_tool_name << " to go " << direction << " with duration " << duration << ", speed " << speed);

  goal.fastening_tool_name = fastening_tool_name;
  goal.direction = direction;
  goal.duration = duration;
  goal.speed = speed;
  fastening_tool_client.sendGoal(goal);
  ros::Duration(0.5).sleep();
  bool finished_before_timeout = false;
  if (wait)
  {
    finished_before_timeout = fastening_tool_client.waitForResult(ros::Duration(10.0));
    result = fastening_tool_client.getResult();
    ROS_DEBUG_STREAM("Action " << (finished_before_timeout ? "returned" : "did not return before timeout") <<", with result: " << result->control_result);
    return result->control_result;
  }
  else
    return true;
  result = fastening_tool_client.getResult();
  ROS_DEBUG("Returning from motor command.");
  return result->control_result;
}

bool SkillServer::setSuctionEjection(std::string fastening_tool_name, bool turn_suction_on, bool eject_screw)
{
  if (!use_real_robot_)
    return true;
  // Send a goal to the action
  o2ac_msgs::SuctionControlGoal goal;
  o2ac_msgs::SuctionControlResultConstPtr result;

  ROS_INFO_STREAM("Setting suction of  " << fastening_tool_name << " to " << turn_suction_on << ", ejection = " << eject_screw);

  goal.fastening_tool_name = fastening_tool_name;
  goal.turn_suction_on = turn_suction_on;
  goal.eject_screw = eject_screw;
  suction_client.sendGoal(goal);
  ros::Duration(0.1).sleep();
  bool finished_before_timeout = suction_client.waitForResult(ros::Duration(2.0));
  result = suction_client.getResult();
  ROS_DEBUG_STREAM("Suction action " << (finished_before_timeout ? "returned" : "did not return before timeout") <<", with result: " << result->success);
  return result->success;
}

// Add the screw tool as a Collision Object to the scene, so that it can be attached to the robot
bool SkillServer::spawnTool(std::string screw_tool_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  if (screw_tool_id == "screw_tool_m4") collision_objects[0] = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") collision_objects[0] = screw_tool_m3;
  else if (screw_tool_id == "set_screw_tool") collision_objects[0] = set_screw_tool;
  else if (screw_tool_id == "nut_tool_m6") collision_objects[0] = nut_tool;
  else if (screw_tool_id == "suction_tool") collision_objects[0] = suction_tool;

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  return true;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool SkillServer::despawnTool(std::string screw_tool_id)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = screw_tool_id;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    return true;
}

bool SkillServer::attachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "attach");
}

bool SkillServer::detachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "detach");
}

bool SkillServer::attachDetachTool(std::string screw_tool_id, std::string robot_name, std::string attach_or_detach)
{
  moveit_msgs::AttachedCollisionObject att_coll_object;

  if (screw_tool_id == "screw_tool_m6") att_coll_object.object = screw_tool_m6;
  else if (screw_tool_id == "screw_tool_m4") att_coll_object.object = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") att_coll_object.object = screw_tool_m3;
  else if (screw_tool_id == "suction_tool") att_coll_object.object = suction_tool;
  else { ROS_WARN_STREAM("No screw tool specified to " << attach_or_detach); }

  att_coll_object.link_name = robot_name + "_robotiq_85_tip_link";

  if (attach_or_detach == "attach") att_coll_object.object.operation = att_coll_object.object.ADD;
  else if (attach_or_detach == "detach") att_coll_object.object.operation = att_coll_object.object.REMOVE;
  
  ROS_INFO_STREAM(attach_or_detach << "ing tool " << screw_tool_id);
  planning_scene_interface_.applyAttachedCollisionObject(att_coll_object);
  return true;
}

bool SkillServer::placeFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "place_pose");
  ROS_DEBUG_STREAM("Received placeFromAbove command.");

  // Move above the target pose
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving above object target place.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move down to the target pose
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to place object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = 0;
  // bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  // if (!success) 
  // {
    // ROS_INFO_STREAM("Linear motion plan to target place pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.01);  // Force the move even if LIN fails
  // }
  openGripper(robot_name, gripper_name);
  
  // // Move back up a little
  // target_tip_link_pose.pose.position.z += .05;
  // ROS_INFO_STREAM("Moving back up after placing object.");
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  // if (!success) 
  // {
  //   ROS_INFO_STREAM("Linear motion plan back from place pose failed. Performing PTP.");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //   moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  // }

  ROS_DEBUG_STREAM("Finished placing object.");
  return true;
}

bool SkillServer::pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "pick_pose");
  ROS_DEBUG_STREAM("Received pickFromAbove command.");
  
  // Move above the object
  openGripper(robot_name, gripper_name);
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Opening gripper, moving above object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 1.0);

  // Move onto the object
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to pick object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan to target pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);  // Force the move even if LIN fails
  }
  closeGripper(robot_name, gripper_name);

  // Move back up a little
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving back up after picking object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan back from pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  }

  ROS_INFO_STREAM("Finished picking object.");
  return true;
}

bool SkillServer::pickScrew(geometry_msgs::PoseStamped screw_head_pose, std::string screw_tool_id, std::string robot_name, std::string screw_tool_link, std::string fastening_tool_name)
{
  // Strategy: 
  // - Move 1 cm above the screw head pose
  // - Go down real slow for 2 cm while turning the motor in the direction that would loosen the screw
  // - Move up again slowly
  // - If the suction reports success, return true
  // - If not, try the same a few more times in nearby locations (spiral-search-like)

  // The frame needs to be the outlet_link of the screw feeder
  ROS_INFO_STREAM("Received pickScrew command.");
  
  geometry_msgs::PoseStamped above_screw_head_pose_ = screw_head_pose;
  // TODO (felixvd): Test that this actually works
  if (robot_name == "a_bot")
    above_screw_head_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/3, 0, 0);
  else  // robot_name == "b_bot"
    above_screw_head_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/3, 0, 0);
  ROS_INFO_STREAM("Moving close to screw.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  above_screw_head_pose_.pose.position.x -= .01;
  bool success = moveToCartPoseLIN(above_screw_head_pose_, robot_name, true, screw_tool_link, 0.3, 0.3, use_real_robot_, true);
  if (!success)
  {
    ROS_INFO_STREAM("Linear motion plan to target pick pose failed. Returning false.");
    return false;
  }

  if (screw_tool_id == "screw_tool_m3")
    planning_scene_interface_.allowCollisions(screw_tool_id, "m3_feeder_link");
  else if (screw_tool_id == "screw_tool_m4")
    planning_scene_interface_.allowCollisions(screw_tool_id, "m4_feeder_link");

  auto adjusted_pose = above_screw_head_pose_;
  auto search_start_pose = above_screw_head_pose_;
  bool screw_picked = false;
  boost::shared_ptr<std_msgs::Bool const> bool_msg_pointer;
  
  double max_radius = .0025;
  double theta_incr = M_PI/3;
  double r, radius_increment;
  r=0.0002;
  radius_increment = .001;
  double radius_inc_set = radius_increment / (2*M_PI / theta_incr);
  double theta=0;
  double RealRadius=0;
  double y, z;
  
  // Try to pick the screw, but go around in a spiral while trying to pick it
  setSuctionEjection(screw_tool_id, true);
  while (!screw_picked)
  {
    sendFasteningToolCommand(fastening_tool_name, "loosen", false, 2.0);

    ROS_INFO_STREAM("Moving into screw to pick it up.");
    adjusted_pose.pose.position.x += .02;
    moveToCartPoseLIN(adjusted_pose, robot_name, true, screw_tool_link, 0.05, 0.05, use_real_robot_, true);

    ROS_INFO_STREAM("Moving back a bit slowly.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    adjusted_pose.pose.position.x -= .02;
    moveToCartPoseLIN(adjusted_pose, robot_name, true, screw_tool_link, 0.1, 0.1, use_real_robot_, true);

    // Break out of loop if screw suctioned or max search radius exceeded
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    bool_msg_pointer = ros::topic::waitForMessage<std_msgs::Bool>("/" + screw_tool_id + "/screw_suctioned", ros::Duration(1.0));
    if (bool_msg_pointer != NULL){
      screw_picked = bool_msg_pointer->data;
      if (screw_picked)
        break;
    }
    else if (!use_real_robot_) screw_picked = true;

    if ((RealRadius > max_radius) || (!ros::ok()))
      break;

    // Adjust the position (spiral search)
    ROS_INFO("Retrying pickup with adjusted position");
    theta=theta+theta_incr;
    y=cos(theta)*r;
    z=sin(theta)*r;
    adjusted_pose = search_start_pose;
    adjusted_pose.pose.position.y += y;
    adjusted_pose.pose.position.z += z;
    r = r + radius_inc_set;
    RealRadius = sqrt(pow(y,2)+pow(z,2));
  }

  if (screw_tool_id == "screw_tool_m3")
    planning_scene_interface_.disallowCollisions(screw_tool_id, "m3_feeder_link");
  else if (screw_tool_id == "screw_tool_m4")
    planning_scene_interface_.disallowCollisions(screw_tool_id, "m4_feeder_link");
  ROS_INFO_STREAM("Moving back up completely.");
  above_screw_head_pose_.pose.position.x -= .05;
  moveToCartPoseLIN(above_screw_head_pose_, robot_name, true, screw_tool_link, 0.5, 0.5, use_real_robot_, true);
  
  ROS_INFO_STREAM((screw_picked ? "Finished picking up screw successfully." : "Failed to pick screw."));
  return screw_picked;
}

bool SkillServer::publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type)
{
    visualization_msgs::Marker marker;
    marker.header = marker_pose.header;
    marker.header.stamp = ros::Time::now();
    marker.pose = marker_pose.pose;

    marker.ns = "markers";
    marker.id = marker_id_count++;
    marker.lifetime = ros::Duration(60.0);
    marker.action = visualization_msgs::Marker::ADD;

    if (marker_type == "pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.g = 1.0;
      marker.color.a = 0.8;
      pubMarker_.publish(marker);
      return true;
    }
    if (marker_type == "place_pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.g = 1.0;
      marker.color.a = 0.8;
      pubMarker_.publish(marker);
      return true;
    }
    if (marker_type == "pick_pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.r = 0.8;
      marker.color.g = 0.4;
      marker.color.a = 0.8;
    }
    if (marker_type == "aist_vision_result")
    {
      // Add a sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .01;
      marker.scale.z = .01;
      marker.color.r = 0.8;
      marker.color.g = 0.4;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
    }
    else if (marker_type == "")
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .02;
      marker.scale.y = .1;
      marker.scale.z = .1;
      
      marker.color.g = 1.0;
      marker.color.a = 0.8;
    }
    else 
    {ROS_WARN("No useful marker message received.");}
    pubMarker_.publish(marker);
    if (marker_id_count > 50) marker_id_count = 0;
    return true;
}

// This is a helper function for publishMarker. Publishes a TF-like frame.
bool SkillServer::publishPoseMarker(geometry_msgs::PoseStamped marker_pose)
{
  visualization_msgs::Marker marker;
  marker.header = marker_pose.header;
  marker.header.stamp = ros::Time::now();
  marker.pose = marker_pose.pose;

  marker.ns = "markers";
  marker.id = marker_id_count++;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  // This draws a TF-like frame.
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = .1;
  marker.scale.y = .01;
  marker.scale.z = .01;
  marker.color.a = .8;

  visualization_msgs::Marker arrow_x, arrow_y, arrow_z;
  arrow_x = marker; arrow_y = marker; arrow_z = marker;
  arrow_x.id = marker_id_count++; arrow_y.id = marker_id_count++; arrow_z.id = marker_id_count++;
  arrow_x.color.r = 1.0; arrow_y.color.g = 1.0; arrow_z.color.b = 1.0;

  rotatePoseByRPY(0, 0, M_PI/2, arrow_y.pose);
  rotatePoseByRPY(0, -M_PI/2, 0, arrow_z.pose);

  pubMarker_.publish(arrow_x); pubMarker_.publish(arrow_y); pubMarker_.publish(arrow_z);
  return true;
}
// ----------- Service definitions
bool SkillServer::goToNamedPoseCallback(o2ac_msgs::goToNamedPose::Request &req,
                                           o2ac_msgs::goToNamedPose::Response &res)
{
  ROS_INFO("Received goToNamedPose callback.");
  res.success = goToNamedPose(req.named_pose, req.planning_group);
  return true;
}

bool SkillServer::publishMarkerCallback(o2ac_msgs::publishMarker::Request &req,
                                           o2ac_msgs::publishMarker::Response &res)
{
  ROS_INFO("Received publishMarker callback.");
  return publishMarker(req.marker_pose, req.marker_type);
}
bool SkillServer::toggleCollisionsCallback(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res)
{
  ROS_INFO("Received toggleCollisions callback.");
  return toggleCollisions(req.data);
}

void SkillServer::runModeCallback(const std_msgs::BoolConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  run_mode_ = msg->data;
}
void SkillServer::pauseModeCallback(const std_msgs::BoolConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  pause_mode_ = msg->data;
}
void SkillServer::testModeCallback(const std_msgs::BoolConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  test_mode_ = msg->data;
}

// ----------- Action servers


// pickScrewAction
void SkillServer::executePickScrew(const o2ac_msgs::pickScrewGoalConstPtr& goal)
{
  ROS_INFO("pickScrewAction was called");
  geometry_msgs::PoseStamped target_pose = goal->screw_pose;
  if (target_pose.header.frame_id == "")
  {
    ROS_ERROR("FIXME");
    target_pose.header.frame_id = "world";
  }
  target_pose = transform_pose_now(target_pose, "world", tflistener_);

  if ((robot_statuses_[goal->robot_name].carrying_tool == true) && (goal->tool_name != "screw_tool"))
  {
    ROS_ERROR("Robot is carrying a tool. Nothing can be picked except screws.");
    pickScrewActionServer_.setAborted();
    return;
  }
  
  if (goal->tool_name == "screw_tool")
  {
    std::string screw_tool_id = "screw_tool_m" + std::to_string(goal->screw_size);
    std::string screw_tool_link = goal->robot_name + "_screw_tool_m" + std::to_string(goal->screw_size) + "_tip_link";
    std::string fastening_tool_name = "screw_tool_m" + std::to_string(goal->screw_size);
    bool screw_picked = pickScrew(goal->screw_pose, screw_tool_id, goal->robot_name, screw_tool_link, fastening_tool_name);
    if (!screw_picked)
    {
      ROS_INFO("pickScrewAction has failed to pick the screw");
      pickScrewActionServer_.setAborted();
      return;
    }
  }

  ROS_INFO("pickScrewAction is set as succeeded");
  pickScrewActionServer_.setSucceeded();
}

// placeAction
void SkillServer::executePlace(const o2ac_msgs::placeGoalConstPtr& goal)
{
  ROS_INFO("placeAction was called");
  // TODO: Calculate the target pose with the item height currently held
  std::string ee_link_name;
  if (goal->tool_name == "suction")
  {
    ; // TODO: Set the ee_link_name correctly and pass a flag to placeFromAbove
  }
  else
  {
    if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
    else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}
  }
  
  placeFromAbove(goal->item_pose, ee_link_name, goal->robot_name);
  ROS_INFO("placeAction is set as succeeded");
  placeActionServer_.setSucceeded();
}

// regraspAction
void SkillServer::executeRegrasp(const o2ac_msgs::regraspGoalConstPtr& goal)
{
  ROS_INFO_STREAM("regraspAction was called with grasp_distance = " << goal->grasp_distance);
  openGripper(goal->receiver_robot_name);
  double gripper_distance_before_grasp = goal->gripper_distance_before_grasp, 
         grasp_distance = goal->grasp_distance, 
         gripper_distance_after_grasp = goal->gripper_distance_after_grasp;
  
  // Set default values
  if (gripper_distance_before_grasp < 0.001)
  {
    gripper_distance_before_grasp = 0.1;
  }
  if (grasp_distance == 0.0)
  {
    grasp_distance = 0.02;
  }
  if (gripper_distance_after_grasp < 0.001)
  {
    gripper_distance_after_grasp = 0.1;
  }

  // Create the handover_pose for Giver and Receiver robot
  std::string holder_robot_name = goal->giver_robot_name, 
              picker_robot_name = goal->receiver_robot_name;

  // The giver holds the item steady, the receiver picks it up.
  // Priority: c_bot, b_bot, a_bot (a_bot only gives passively)
  // If c_bot is involved, it always picks up (receiver), because its configuration is advantageous.
  if (holder_robot_name == "c_bot")
  {
    holder_robot_name = picker_robot_name;
    picker_robot_name = "c_bot";
  }
  else if ((holder_robot_name == "b_bot") && (picker_robot_name == "a_bot"))
  {
    holder_robot_name = "a_bot";
    picker_robot_name = "b_bot";
  }

  tf::Transform t;
  tf::Quaternion q;
  if (picker_robot_name == "c_bot")
  {
    t.setOrigin(tf::Vector3(-.28, .11, .55));  // x y z of the handover position
    double c_tilt = 0.0;
    if (holder_robot_name == "b_bot")
    { 
      c_tilt = 10.0;  // degrees. Tilts during the handover to c (this makes the pose nicer for b_bot)
    } 
    q.setRPY(M_PI, -c_tilt/180.0*M_PI, 0);   // r p y of the handover position (for the receiver)    
  }
  else if ((picker_robot_name == "b_bot") || (picker_robot_name == "a_bot"))
  {
    t.setOrigin(tf::Vector3(0.1, 0.0, 0.65)); 
    if (picker_robot_name == "b_bot")
    {
      q.setRPY(M_PI/2, 0, -M_PI/2);
      ROS_INFO_STREAM("picker_robot: " << picker_robot_name);
    }
    
    else
    {
      ROS_ERROR("This function should not arrive here.");
      q.setRPY(0, 0, 0);   
    }
  }
  t.setRotation(q);
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));

  geometry_msgs::PoseStamped handover_pose_holder, handover_pose_picker;
  handover_pose_holder.header.frame_id = "handover_frame";
  handover_pose_picker.header.frame_id = "handover_frame";
  handover_pose_picker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  if(holder_robot_name == "a_bot")
  {
    handover_pose_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI); // Facing the receiver, rotated
  }
  else handover_pose_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, 0, M_PI); 
   
  publishMarker(handover_pose_picker, "pick_pose");
  publishMarker(handover_pose_holder, "place_pose");
  ros::Duration(.1).sleep();
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));

  goToNamedPose("back", picker_robot_name, 2.0, 2.0, use_real_robot_);
  if (holder_robot_name == "b_bot")
    goToNamedPose("regrasp_ready", holder_robot_name, 1.0, 0.5, use_real_robot_);

  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));
  // Move the Giver to the regrasp_pose
  ROS_INFO_STREAM("Moving giver robot (" << holder_robot_name << ") to handover pose.");
  moveToCartPoseLIN(handover_pose_holder, holder_robot_name);
  
  // Move the Receiver to an approach pose, then on to grasp
  ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") to approach pose.");
  handover_pose_picker.pose.position.x = -gripper_distance_before_grasp;
  moveToCartPoseLIN(handover_pose_picker, picker_robot_name);

  ros::Duration(1).sleep();
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));
  ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") to grasp pose.");
  handover_pose_picker.pose.position.x = -grasp_distance;
  ROS_WARN_STREAM("Position x is: " << -grasp_distance);
  moveToCartPoseLIN(handover_pose_picker, picker_robot_name);
  
  // Close the Receiver's gripper
  
  if(holder_robot_name != "a_bot")
  {
    closeGripper(goal->receiver_robot_name);
    ros::Duration(1).sleep();
    openGripper(goal->giver_robot_name);

    // Move back.
    ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") back to approach pose.");
    handover_pose_picker.pose.position.x = -gripper_distance_after_grasp;
    moveToCartPoseLIN(handover_pose_picker, picker_robot_name);
    ROS_INFO("regraspAction is set as succeeded");
    regraspActionServer_.setSucceeded();
  }
}

// screwAction
void SkillServer::executeScrew(const o2ac_msgs::screwGoalConstPtr& goal)
{
  ROS_INFO("screwAction was called");

  // Set target pose for the end effector
  geometry_msgs::PoseStamped target_tip_link_pose = goal->target_hole;
  std::string screw_tool_link = goal->robot_name + "_screw_tool_" + "m" + std::to_string(goal->screw_size) + "_tip_link";
  std::string screw_tool_id = "screw_tool_m" + std::to_string(goal->screw_size);
  // std::string screw_tool_link = held_screw_tool_ + "_tip";
  ROS_INFO_STREAM("screw tool link:  " << screw_tool_link);

  target_tip_link_pose.pose.position.x -= goal->screw_height;  // Add the screw height

  if (goal->screw_height < 0.001) {target_tip_link_pose.pose.position.x -= .01;}   // In case screw_height was not set
  double approach_height = .01; // This should include tolerance
  double insertion_amount = .01;   // This is the depth that is compensated for by the bit cushion/spring

  // Move above the screw hole
  ROS_INFO("Moving above the screw hole.");
  target_tip_link_pose.pose.position.x -= approach_height;
  moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link, 0.2, 0.2, use_real_robot_, true);

  sendFasteningToolCommand(screw_tool_id, "tighten", false, 15.0, 800);
  // Disable collision for screw tool 
  updatePlanningScene();
  collision_detection::AllowedCollisionMatrix acm_original(planning_scene_.allowed_collision_matrix);
  planning_scene_interface_.allowCollisions(screw_tool_id);

  // Move 1 cm into to the screw hole
  ROS_INFO("Pushing into the screw hole and doing spiral motion.");
  target_tip_link_pose.pose.position.x += approach_height + insertion_amount;
  bool success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link, 0.02, 0.02, use_real_robot_, true);

  // Do spiral motion
  if (use_real_robot_)
  {
    o2ac_msgs::sendScriptToUR srv;
    srv.request.program_id = "spiral_motion";
    srv.request.robot_name = goal->robot_name;
    srv.request.max_radius = .002;
    srv.request.radius_increment = .0005;
    srv.request.spiral_axis = "Y";
    if (goal->robot_name=="c_bot")
      srv.request.spiral_axis = "YZ";
    sendScriptToURClient_.call(srv);
    if (srv.response.success == true)
    {
      ROS_DEBUG("Successfully called the service client to do spiral motion.");
      ros::Duration(1.0).sleep();
      waitForURProgram("/" + goal->robot_name);
    }
    else
      ROS_ERROR("Could not call the service client to do spiral motion.");
  }

  if (use_real_robot_) {
    bool finished_before_timeout = fastening_tool_client.waitForResult(ros::Duration(10.0));
    auto result = fastening_tool_client.getResult();
    ROS_INFO_STREAM("Screw tool motor command " << (finished_before_timeout ? "returned" : "did not return before timeout") <<". Result: " << result->control_result);
  }


  if (!goal->stay_put_after_screwing)
  {
    // Move up and away
    target_tip_link_pose.pose.position.x -= approach_height + insertion_amount;
    success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link, 0.02, 0.02, use_real_robot_, true);
  }

  auto bool_msg_pointer = ros::topic::waitForMessage<std_msgs::Bool>("/" + screw_tool_id + "/screw_suctioned", ros::Duration(1.0));
  bool screw_not_suctioned_anymore = false;
  if (bool_msg_pointer != NULL){
    screw_not_suctioned_anymore = !bool_msg_pointer->data;
  }
  else if (!use_real_robot_) screw_not_suctioned_anymore = true;

  // Enable collision for screw tool again
  moveit_msgs::PlanningScene ps_reset_collisions = planning_scene_;
  acm_original.getMessage(ps_reset_collisions.allowed_collision_matrix);
  planning_scene_interface_.applyPlanningScene(ps_reset_collisions);

  if (screw_not_suctioned_anymore)
  {
    setSuctionEjection(screw_tool_id, false, false);    // Turn off both suction and ejection
    ROS_INFO("screwAction is set as succeeded");
    screwActionServer_.setSucceeded();
  }
  else
  {
    ROS_INFO("screwAction did not succeed: screw is still suctioned.");
    screwActionServer_.setAborted();
  }
}

void SkillServer::executeChangeTool(const o2ac_msgs::changeToolGoalConstPtr& goal)
{
  ROS_INFO("Received changeToolAction goal.");
  std::string equip_or_unequip = "equip";
  if (!goal->equip_the_tool) { equip_or_unequip = "unequip"; }

  std::string screw_tool_id = "screw_tool_m" + std::to_string(goal->screw_size);
  if (goal->screw_size == 66)
    screw_tool_id = "nut_tool_m6";
  else if (goal->screw_size == 50)
    screw_tool_id = "suction_tool";
  else if (goal->screw_size == 1)
    screw_tool_id = "set_screw_tool";

  bool success = equipUnequipScrewTool(goal->robot_name, screw_tool_id, equip_or_unequip);
  
  if (success) { changeToolActionServer_.setSucceeded(); }
  else changeToolActionServer_.setAborted();
}

// ----------- End of the class definitions

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o2ac_skills");
  ros::AsyncSpinner spinner(1); // Needed for MoveIt to work.
  spinner.start();

  // Create an object of class SkillServer that will take care of everything
  SkillServer ss;
  ROS_INFO("o2ac skill server started");
  while (ros::ok())
  {
    ros::Duration(.1).sleep();
    ros::spinOnce();
  }
  
  return 0;
}
