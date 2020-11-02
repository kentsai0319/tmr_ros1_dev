#pragma once

#include "tmr/tmr_print.h"
#include "tmr/tmr_driver.h"

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


namespace tmr_hardware_interface
{

/// \brief Hardware interface for a tm-robot
class TmrRobotHW: public hardware_interface::RobotHW {
public:
  enum IFace
  {
    NO_IFACE,
    POSITION_IFACE,
    VELOCITY_IFACE
  };

  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  TmrRobotHW(ros::NodeHandle &nh, tmr::Driver *iface);
  virtual ~TmrRobotHW() = default;

  /// \brief Initialize the hardware interface
  virtual bool init(ros::NodeHandle& robot_nh, ros::NodeHandle& robot_hw_nh) override;

  /// \brief Read the state from the robot hardware.
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  
  /// \brief write the command to the robot hardware.
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

  virtual void doSwitch(
    const std::list<hardware_interface::ControllerInfo> &start_list,
    const std::list<hardware_interface::ControllerInfo> &stop_list) override;

  void halt();

private:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  tmr::Driver *robot;

  bool is_fake_;

  IFace running_iface_ = NO_IFACE;
  IFace running_iface_last_ = NO_IFACE;

  bool position_iface_running_ = false;
  bool velocity_iface_running_ = false;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_iface_;
  hardware_interface::PositionJointInterface position_joint_iface_;
  hardware_interface::VelocityJointInterface velocity_joint_iface_;
  //hardware_interface::ForceTorqueSensorInterface force_torque_iface_;

  // Shared memory !!
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::size_t num_joints_;

  std::vector<double> joint_pos_buf_;
  std::vector<double> joint_vel_buf_;
  std::vector<double> joint_tor_buf_;
};

}