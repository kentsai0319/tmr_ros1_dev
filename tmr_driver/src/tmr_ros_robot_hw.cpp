
#include "tmr_driver/tmr_ros_robot_hw.h"

namespace tmr_hardware_interface
{

TmrRobotHW::TmrRobotHW(ros::NodeHandle &nh, tmr::Driver* iface)
  : nh_(nh)
  , robot(iface)
{
  //num_joints_ = robot->state.DOF;
}

bool TmrRobotHW::init(ros::NodeHandle& robot_nh, ros::NodeHandle& robot_hw_nh)
{
  // Get joint names
  ROS_INFO_STREAM(
    "TM_RHW: Reading rosparams from namespace: " << robot_hw_nh.getNamespace()
  );
  robot_hw_nh.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0) {
    ROS_ERROR(
      "TM_RHW: No joints found on parameter server for controller"
    );
    return false;
  }
  if (joint_names_.size() != robot->state.DOF) {
    ROS_ERROR(
      "TM_RHW: Number of joints is not equal to robot->state.DOF"
    );
    return false;
  }
  num_joints_ = joint_names_.size();

  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);

  joint_pos_buf_.resize(num_joints_);
  joint_vel_buf_.resize(num_joints_);
  joint_tor_buf_.resize(num_joints_);

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {
    ROS_INFO_STREAM(
      "TM_RHW: Loading joint name: " << joint_names_[i]
    );
    // joint state interface
    joint_state_iface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]
      )
    );
    // Create position joint interface
    position_joint_iface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_iface_.getHandle(joint_names_[i]), &joint_position_command_[i]
      )
    );
    // Create velocity joint interface
    velocity_joint_iface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_iface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]
      )
    );
  }
  registerInterface( &joint_state_iface_ );
  registerInterface( &position_joint_iface_ );
  registerInterface( &velocity_joint_iface_ );
  return true;
}

void TmrRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  {
    std::lock_guard<std::mutex> lck(robot->state.mtx);
    joint_pos_buf_ = robot->state.joint_angle();
    joint_vel_buf_ = robot->state.joint_speed();
    joint_tor_buf_ = robot->state.joint_torque();
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    joint_position_[i] = joint_pos_buf_[i];
    joint_velocity_[i] = joint_vel_buf_[i];
    joint_effort_[i] = joint_tor_buf_[i];
  }
}

void TmrRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  // robot is not ok to write
  if (false) {
    running_iface_ = NO_IFACE;
    running_iface_last_ = running_iface_;
    return;
  }
  // running interface
  if (position_iface_running_) {
    running_iface_ = POSITION_IFACE;
  }
  else if (velocity_iface_running_) {
    running_iface_ = VELOCITY_IFACE;
  }
  else {
    running_iface_ = NO_IFACE;
  }
  if (running_iface_last_!= running_iface_) {
    // position mode: ...

    // velocity mode:
    std::vector<double> zero_vel(num_joints_, 0.0);
    // leave velocity mode
    if (running_iface_last_ == VELOCITY_IFACE) {
      robot->set_vel_mode_target(tmr::VelMode::Joint, zero_vel);
      robot->set_vel_mode_stop();
      ROS_INFO("TM_RHW: leave velocity mode");
    }
    // enter velocity mode
    if (running_iface_ == VELOCITY_IFACE) {
      ROS_INFO("TM_RHW: enter velocity mode");
      robot->set_vel_mode_start(tmr::VelMode::Joint, 0.1, 1.0);
      robot->set_vel_mode_target(tmr::VelMode::Joint, zero_vel);
    }
  }
  running_iface_last_ = running_iface_;
  // write command
  switch (running_iface_) {
  case POSITION_IFACE:

    break;
  case VELOCITY_IFACE:
    robot->set_vel_mode_target(tmr::VelMode::Joint, joint_velocity_command_);
    break;
  }
}

void TmrRobotHW::doSwitch(
    const std::list<hardware_interface::ControllerInfo> &start_list,
    const std::list<hardware_interface::ControllerInfo> &stop_list)
{
  for (auto& controller_it : stop_list) {
    for (auto& resource_it : controller_it.claimed_resources) {
      if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
        position_iface_running_ = false;
      }
      if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface") {
        velocity_iface_running_ = false;
      }
    }
  }
  for (auto& controller_it : start_list) {
    for (auto& resource_it : controller_it.claimed_resources) {
      if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
        position_iface_running_ = true;
      }
      if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface") {
        velocity_iface_running_ = true;
      }
    }
  }
}

void TmrRobotHW::halt()
{
  std::cout << "TM_RHW: halt\n";

  position_iface_running_ = false;
  velocity_iface_running_ = false;
  running_iface_ = NO_IFACE;

  if (running_iface_last_ == VELOCITY_IFACE) {
    robot->set_vel_mode_stop();
  }
}

}
