#include "sobit_edu_library/sobit_edu_joint_action_server.hpp"


namespace sobit_edu{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));
  this->service_server_get_hand_to_coord_ = this->create_service<GetHandToTargetCoord>(
      "get_hand_to_coord",
      std::bind(&JointActionServer::serve_get_hand_to_coord, this, std::placeholders::_1, std::placeholders::_2));
  this->service_server_get_hand_to_tf_ = this->create_service<GetHandToTargetTF>(
      "get_hand_to_tf",
      std::bind(&JointActionServer::serve_get_hand_to_tf, this, std::placeholders::_1, std::placeholders::_2));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
  this->pub_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_position_controller/joint_trajectory", qos_profile);
  this->pub_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_position_controller/joint_trajectory", qos_profile);
  this->pub_head_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "head_position_controller/joint_trajectory", qos_profile);


  //Declare the pose parameters

  this->declare_parameter("poses", std::vector<std::string>());
  auto pose_names = this->get_parameter("poses").as_string_array();

  poses_.clear();
  for (auto pose_name : pose_names) {
    // Declare parameters for each pose
    this->declare_parameter(pose_name + ".arm_shoulder_roll" , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_shoulder_pitch", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_forearm_roll"  , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_roll"    , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand"              , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_camera_pan"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_camera_tilt"  , rclcpp::PARAMETER_DOUBLE);

    // Read parameters for each pose
    PoseParams params;
    params.pose_name           = pose_name;
    params.arm_shoulder_roll = this->get_parameter(pose_name + ".arm_shoulder_roll").as_double();
    params.arm_shoulder_pitch  = this->get_parameter(pose_name + ".arm_shoulder_pitch").as_double();
    params.arm_elbow_pitch    = this->get_parameter(pose_name + ".arm_elbow_pitch").as_double();
    params.arm_forearm_roll    = this->get_parameter(pose_name + ".arm_forearm_roll").as_double();
    params.arm_wrist_pitch              = this->get_parameter(pose_name + ".arm_wrist_pitch").as_double();
    params.arm_wrist_roll = this->get_parameter(pose_name + ".arm_wrist_roll").as_double();
    params.hand  = this->get_parameter(pose_name + ".hand").as_double();
    params.head_camera_pan    = this->get_parameter(pose_name + ".head_camera_pan").as_double();
    params.head_camera_tilt    = this->get_parameter(pose_name + ".head_camera_tilt").as_double();
    poses_.push_back(params);
  }

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
}
JointActionServer::~JointActionServer()
{
  this->action_server_move_joints_.reset();
  this->action_server_move_to_pose_.reset();

  this->sub_joint_state_.reset();
  this->pub_arm_joint_control_.reset();
  this->pub_hand_joint_control_.reset();
  this->pub_head_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been terminated.");
}


rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void JointActionServer::handle_move_joints_accepted(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
}

void JointActionServer::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}


void JointActionServer::exe_move_joints(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request. The number of joint names and joint rad are different");
    result->success = false;
    result->message = "Invalid goal request. The number of joint names and joint rad are different";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(kJointNames.begin(), kJointNames.end(), goal->target_joint_names[i]) == kJointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "The joint name does not exist: %s", goal->target_joint_names[i].c_str());
      result->success = false;
      result->message = "The joint name does not exist: " + goal->target_joint_names[i];
      result->total_elapsed_time.sec = 0;
      result->total_elapsed_time.nanosec = 0;
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory hand_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory head_joint_trajectory;
  arm_joint_trajectory  = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "arm");
  hand_joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "hand");
  head_joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "head");
  
  try {
    if (!arm_joint_trajectory.joint_names.empty())
      this->pub_arm_joint_control_->publish(arm_joint_trajectory);
    if (!hand_joint_trajectory.joint_names.empty())
      this->pub_hand_joint_control_->publish(hand_joint_trajectory);
    if (!head_joint_trajectory.joint_names.empty())
      this->pub_head_joint_control_->publish(head_joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      this->pub_head_joint_control_->publish(set_joints({}, {}, dt, "head"));
      this->pub_arm_joint_control_->publish(set_joints({}, {}, dt, "arm"));
      this->pub_hand_joint_control_->publish(set_joints({}, {}, dt, "hand"));

      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();

  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::exe_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  // Check if the pose name is valid
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose) { return pose.pose_name == goal->pose_name; }) == poses_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    result->success = false;
    result->message = "Invalid pose name: " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad;
  for (const auto &pose : poses_) {
    if (pose.pose_name == goal->pose_name) {
      target_joint_rad.push_back(pose.arm_shoulder_roll);
      target_joint_rad.push_back(pose.arm_shoulder_pitch);
      target_joint_rad.push_back(pose.arm_elbow_pitch);
      target_joint_rad.push_back(pose.arm_forearm_roll);
      target_joint_rad.push_back(pose.arm_wrist_pitch);
      target_joint_rad.push_back(pose.arm_wrist_roll);
      target_joint_rad.push_back(pose.hand);
      target_joint_rad.push_back(pose.head_camera_pan);
      target_joint_rad.push_back(pose.head_camera_tilt);
      break;
    }
  }

  if (target_joint_rad.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to not find the pose name : %s", goal->pose_name.c_str());

    result->success = false;
    result->message = "[FAIL] Failed to not find the pose name : " +  goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
  }

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory hand_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory head_joint_trajectory;
  arm_joint_trajectory  = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "arm");
  hand_joint_trajectory = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "hand");
  head_joint_trajectory = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "head");

  try {
    if (!arm_joint_trajectory.joint_names.empty())
      this->pub_arm_joint_control_->publish(arm_joint_trajectory);
    if (!hand_joint_trajectory.joint_names.empty())
      this->pub_hand_joint_control_->publish(hand_joint_trajectory);
    if (!head_joint_trajectory.joint_names.empty())
      this->pub_head_joint_control_->publish(head_joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);
  
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = kJointNames;
    for (const auto &joint_name : kJointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < kJointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[kJointNames[i]] - target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->message = "[SUCCESS] Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::serve_get_hand_to_coord(
  const std::shared_ptr<GetHandToTargetCoord::Request> request,
  std::shared_ptr<GetHandToTargetCoord::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  // Transform to robot base from 'sobit_edu/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      request->target_coord, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  
  target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);


  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (z: 0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::serve_get_hand_to_tf(
  const std::shared_ptr<GetHandToTargetTF::Request> request,
  std::shared_ptr<GetHandToTargetTF::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  geometry_msgs::msg::TransformStamped goal_coord_shift;


  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->tf_differential.header.frame_id, request->target_frame, 
      tf2::TimePointZero);

    geometry_msgs::msg::Vector3 euler_target, euler_shift;
    euler_target = get_euler_from_quat(goal_coord_shift.transform.rotation);
    euler_shift = get_euler_from_quat(request->tf_differential.transform.rotation);
    euler_target.x += euler_shift.x;
    euler_target.y += euler_shift.y;
    euler_target.z += euler_shift.z;

    goal_coord_shift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_coord_shift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_coord_shift.transform.translation.z += request->tf_differential.transform.translation.z;
    goal_coord_shift.transform.rotation = get_quat_from_euler(euler_target);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s to %s: %s", request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(),ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform: " + request->target_frame + " to: " + request->tf_differential.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // Transform to robot base from 'sobit_edu/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      goal_coord_shift, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s", goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);

  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (z: 0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "arm_shoulder_pitch_sub_joint") continue;  // Skip sub joints

    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
  const std::vector<std::string> &target_joint_names,
  const std::vector<double> &target_joint_rad,
  const builtin_interfaces::msg::Duration &time_allowance,
  const std::string &group_name)
{
  auto joint_trajectory  = trajectory_msgs::msg::JointTrajectory();
  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (size_t i = 0; i < target_joint_names.size(); i++) {
    // Check if the joint belongs to the specified group
    if (group_name == "arm" &&
        std::find(kArmJointNames.begin(), kArmJointNames.end(), target_joint_names[i]) == kArmJointNames.end()) {
      continue;
    }
    else if (group_name == "hand" &&
        std::find(kHandJointNames.begin(), kHandJointNames.end(), target_joint_names[i]) == kHandJointNames.end()) {
      continue;
    }
    else if (group_name == "head" &&
        std::find(kHeadJointNames.begin(), kHeadJointNames.end(), target_joint_names[i]) == kHeadJointNames.end()) {
      continue;
    }
    joint_trajectory.joint_names.push_back(target_joint_names[i]);
    point.positions.push_back(target_joint_rad[i]);

    // Subjoint to turn opposite direction
    if (target_joint_names[i] == "arm_shoulder_pitch_joint") {
      joint_trajectory.joint_names.push_back("arm_shoulder_pitch_sub_joint");
      point.positions.push_back(-target_joint_rad[i]);
      continue;
    } 
  }

  joint_trajectory.points.push_back(point);
  joint_trajectory.points[0].time_from_start = time_allowance;

  return joint_trajectory;
}

// ここは，もしも今後逆運動学が3次元に発展したときに，それに対応させるために3次元での順運動学を算出
geometry_msgs::msg::TransformStamped JointActionServer::forward_kinematics(
  const std::vector<double> &target_joint_rad,
  const double target_yaw)
{
  geometry_msgs::msg::TransformStamped final_coord;

  // hand_pt <=> final_coord
  geometry_msgs::msg::Point shoulder_pt, elbow_pt, wrist_pt/*, hand_pt*/;

  // Calculate the coordinates of the shoulder. This coordinate is static one.
  shoulder_pt.x = base_to_shoulder_x;
  shoulder_pt.y = 0.;
  shoulder_pt.z = base_to_shoulder_z;

  // Calculate the coordinates of the elbow.
  elbow_pt.x = shoulder_pt.x + arm_upper_link*std::sin(target_joint_rad[0]);
  elbow_pt.y = shoulder_pt.y;
  elbow_pt.z = shoulder_pt.z + arm_upper_link*std::cos(target_joint_rad[0]);

  // Calculate the coordinates of the wrist.
  wrist_pt.x = elbow_pt.x + arm_lower_link*std::cos(-target_joint_rad[1]-target_joint_rad[0]);
  wrist_pt.y = elbow_pt.y;
  wrist_pt.z = elbow_pt.z + arm_lower_link*std::sin(-target_joint_rad[1]-target_joint_rad[0]);

  // Calculate the coordinates of the grasp position. // TODO : Calculate the orientation from posture of elbow2wrist.
  final_coord.transform.translation.x = wrist_pt.x + arm_gripper_link*std::cos(-target_joint_rad[2]-target_joint_rad[1]-target_joint_rad[0]);
  final_coord.transform.translation.y = wrist_pt.y;
  final_coord.transform.translation.z = wrist_pt.z + arm_gripper_link*std::sin(-target_joint_rad[2]-target_joint_rad[1]-target_joint_rad[0]);
  final_coord.transform.rotation.w = 1.;
  final_coord.transform.rotation.x = 0.;
  final_coord.transform.rotation.y = 0.;
  final_coord.transform.rotation.z = 0.;

  // Consider target_yaw
  double temp_x, temp_y;
  temp_x = final_coord.transform.translation.x;
  temp_y = final_coord.transform.translation.y;
  final_coord.transform.translation.x = temp_x*std::cos(target_yaw) - temp_y*std::sin(target_yaw);
  final_coord.transform.translation.y = temp_y*std::cos(target_yaw) + temp_x*std::sin(target_yaw);

  return final_coord;
}

std::vector<double> JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord,  // 'goal_coord' is the coordinates of robot base.
  const double target_yaw)
// 三角関数
// {

//   (void)target_yaw;

//   // "arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"
//   // return msg
//   std::vector<double> target_joint_rad = {0.0, 0.0, 0.0};

//   if (goal_coord.transform.translation.z < (base_to_shoulder_z + arm_upper_link*std::cos(3*M_PI/4)-arm_lower_link)) {
//     RCLCPP_WARN(this->get_logger(), "The target position is too low (%.2f[m] < min:%.2f[m])", goal_coord.transform.translation.z, (base_to_shoulder_z + arm_upper_link*std::cos(3*M_PI/4)-arm_lower_link));
//     target_joint_rad.clear();
//     return target_joint_rad;
//   }
//   if ((base_to_shoulder_z + arm_upper_link + arm_lower_link*std::cos(M_PI/12)) < goal_coord.transform.translation.z) {
//     RCLCPP_WARN(this->get_logger(), "The target position is too tall (max:%.2f[m] < %.2f[m])", (base_to_shoulder_z + arm_upper_link + arm_lower_link*std::cos(M_PI/12)), goal_coord.transform.translation.z);
//     target_joint_rad.clear();
//     return target_joint_rad;
//   }

//   double target_z = goal_coord.transform.translation.z - base_to_shoulder_z;

//   if (target_z < -(arm_upper_link+arm_lower_link)*std::cos(M_PI/4.)) {
//     target_joint_rad[0] = 3*M_PI/4.;

//   } else if (target_z < -arm_lower_link) {
//     target_joint_rad[0] = std::atan2(std::sqrt(std::pow(arm_upper_link+arm_lower_link, 2) - std::pow(target_z, 2)), target_z);

//   } else if (target_z <= 0.) {
//     target_joint_rad[0] = M_PI/2.;

//   } else if (target_z < arm_upper_link*std::cos(M_PI/4.)) {
//     target_joint_rad[0] = M_PI/4.;

//   } else {
//     target_joint_rad[0] = 0.0;

//   }

//   geometry_msgs::msg::Point elbow_pt, wrist_pt;
//   elbow_pt.x = arm_upper_link*std::sin(target_joint_rad[0]);
//   elbow_pt.z = arm_upper_link*std::cos(target_joint_rad[0]);
//   wrist_pt.x = elbow_pt.x + std::sqrt(std::pow(arm_lower_link, 2) - std::pow(target_z-elbow_pt.z, 2));
//   wrist_pt.z = target_z;
//   target_joint_rad[1] = std::atan2(wrist_pt.x-elbow_pt.x, wrist_pt.z-elbow_pt.z) - M_PI/2. - target_joint_rad[0];

//   target_joint_rad[2] = -target_joint_rad[0] -target_joint_rad[1];
//   return target_joint_rad;

// }

// ヤコビ行列
{
  (void)target_yaw;

  double dt = 0.001;
  geometry_msgs::msg::Point initial_wrist_pt;
  initial_wrist_pt.x = -arm_upper_link;
  initial_wrist_pt.y = 0.;
  initial_wrist_pt.z = arm_lower_link;

  // "arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"
  // return msg
  std::vector<double> target_joint_rad = {-M_PI/2., 0.0, 0.0};

  if (goal_coord.transform.translation.z < (base_to_shoulder_z + arm_upper_link*std::cos(3*M_PI/4)-arm_lower_link)) {
    RCLCPP_WARN(this->get_logger(), "The target position is too low (%.2f[m] < min:%.2f[m])", goal_coord.transform.translation.z, (base_to_shoulder_z + arm_upper_link*std::cos(3*M_PI/4)-arm_lower_link));
    target_joint_rad.clear();
    return target_joint_rad;
  }
  if ((base_to_shoulder_z + (arm_upper_link+arm_lower_link)*std::cos(M_PI/4)) < goal_coord.transform.translation.z) {
    RCLCPP_WARN(this->get_logger(), "The target position is too tall (max:%.2f[m] < %.2f[m])", (base_to_shoulder_z + (arm_upper_link+arm_lower_link)*std::cos(M_PI/4)), goal_coord.transform.translation.z);
    target_joint_rad.clear();
    return target_joint_rad;
  }

  double r = std::sqrt(std::pow(arm_upper_link*std::cos(M_PI/4), 2) + std::pow(arm_upper_link*std::sin(M_PI/4)+arm_lower_link, 2));
  double target_x = std::sqrt(std::pow(r, 2) - std::pow(goal_coord.transform.translation.z-base_to_shoulder_z, 2))*1.1;
  double target_z = goal_coord.transform.translation.z - base_to_shoulder_z;


  for (int i=0; i<(int)(1./dt); i++) {
    double j_[2][2] = {
      {
        arm_upper_link*std::cos(target_joint_rad[0]) + arm_lower_link*std::sin(-target_joint_rad[0]-target_joint_rad[1]),
        arm_lower_link*std::sin(-target_joint_rad[0]-target_joint_rad[1])
      }, {
        -arm_upper_link*std::sin(target_joint_rad[0]) - arm_lower_link*std::cos(-target_joint_rad[0]-target_joint_rad[1]),
        -arm_lower_link*std::cos(-target_joint_rad[0]-target_joint_rad[1])
      }
    };
    double norm = j_[0][0]*j_[1][1] - j_[0][1]*j_[1][0];

    if (norm == 0.) return {};

    double j__[2][2] = {{j_[1][1]/norm, -j_[0][1]/norm},
                        {-j_[1][0]/norm, j_[0][0]/norm}};

    target_joint_rad[0] += (target_x-initial_wrist_pt.x)*dt*j__[0][0] + (target_z-initial_wrist_pt.z)*dt*j__[0][1];
    target_joint_rad[1] += (target_x-initial_wrist_pt.x)*dt*j__[1][0] + (target_z-initial_wrist_pt.z)*dt*j__[1][1];
  }

  if (std::abs(target_joint_rad[0]) >= 2*M_PI) target_joint_rad[0] -= 2*M_PI * (int)(target_joint_rad[0] / 2*M_PI);
  if (std::abs(target_joint_rad[1]) >= 2*M_PI) target_joint_rad[1] -= 2*M_PI * (int)(target_joint_rad[1] / 2*M_PI);
  if (std::abs(target_joint_rad[0]) >= M_PI) target_joint_rad[0] -= 2*M_PI * (target_joint_rad[0] / std::abs(target_joint_rad[0]));
  if (std::abs(target_joint_rad[1]) >= M_PI) target_joint_rad[1] -= 2*M_PI * (target_joint_rad[1] / std::abs(target_joint_rad[1]));

  target_joint_rad[2] = -target_joint_rad[0] -target_joint_rad[1];


  return target_joint_rad;
}
} // namespace sobit_edu


