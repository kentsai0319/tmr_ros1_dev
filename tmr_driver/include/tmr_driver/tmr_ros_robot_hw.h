#pragma once

#include "tmrl/driver/driver.h"
#include "tmrl/utils/logger.h"

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

namespace tmr_ros
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
  enum {
    DOF = 6
  };

  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  TmrRobotHW(ros::NodeHandle &nh, tmrl::driver::Driver *iface);
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

  tmrl::driver::Driver *robot_;

  const std::size_t num_joints_;

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
  tmrl::vector6d joint_position_ {0};
  tmrl::vector6d joint_velocity_ {0};
  tmrl::vector6d joint_effort_ {0};
  tmrl::vector6d joint_position_command_ {0};
  tmrl::vector6d joint_velocity_command_ {0};

  tmrl::vector6d joint_pos_buf_ {0};
  tmrl::vector6d joint_vel_buf_ {0};
  tmrl::vector6d joint_tor_buf_ {0};
};

}