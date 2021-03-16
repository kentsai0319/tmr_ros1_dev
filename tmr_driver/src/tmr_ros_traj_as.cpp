
#include "tmr_driver/tmr_ros_node.h"

// follow_joint_trajectory action part

namespace tmr_ros
{

// helper function

bool TmrRosNode::has_points(const trajectory_msgs::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
        point.velocities.size() != traj.joint_names.size())
      return false;
  }
  return true;
}
bool TmrRosNode::has_limited_velocities(const trajectory_msgs::JointTrajectory &traj)
{
  return true;
}
bool TmrRosNode::is_traj_finite(const trajectory_msgs::JointTrajectory &traj)
{
  for (size_t i = 0; i < traj.points.size(); ++i) {
    for (size_t j = 0; j < traj.points[i].positions.size(); ++j) {
      if (!std::isfinite(traj.points[i].positions[j]))
        return false;
      if (!std::isfinite(traj.points[i].velocities[j]))
        return false;
    }
  }
  return true;
}
void TmrRosNode::reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(joint_names_.size(), joint_names_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      if (traj.joint_names[i] == joint_names_[j])
        mapping[j] = i;
    }
  }
  std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj.push_back(new_point);
  }
  traj.points = new_traj;
}
bool TmrRosNode::is_start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps)
{
  tmrl::driver::RobotState::Ulock lck(iface_.state.mtx);
  auto q_act = iface_.state.joint_angle();
  lck.unlock();

  for (size_t i = 0; i < traj.points[0].positions.size(); ++i) {
    if (fabs(traj.points[0].positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}
void TmrRosNode::set_result(int32_t err_code, const std::string &err_str)
{
  result_.error_code = err_code;
  result_.error_string = err_str;
  if (err_code != result_.SUCCESSFUL && err_str.length()) {
    ROS_ERROR_STREAM(err_str);
  }
}

void TmrRosNode::set_pvt_traj(tmrl::driver::PvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj, double Tmin)
{
  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  tmrl::driver::PvtPoint point;

  if (traj.points.size() == 0) return;

  pvts.mode = tmrl::driver::PvtMode::Joint;

  // first point
  if (traj.points[i].time_from_start.toSec() != 0.0) {
    ROS_WARN("TM_ROS: Traj.: first point should be the current position, with time_from_start set to 0.0");
    point.time = traj.points[i].time_from_start.toSec();
    point.positions = traj.points[i].positions;
    point.velocities = traj.points[i].velocities;
    pvts.points.push_back(point);
  }
  for (i = 1; i < traj.points.size() - 1; ++i) {
    point.time = traj.points[i].time_from_start.toSec() - traj.points[i_1].time_from_start.toSec();
    if (point.time >= Tmin) {
      i_2 = i_1;
      i_1 = i;
      point.positions = traj.points[i].positions;
      point.velocities = traj.points[i].velocities;
      pvts.points.push_back(point);
    }
    else {
      ++skip_count;
    }
  }
  if (skip_count > 0) {
    ROS_WARN("TM_ROS: Traj.: skip %d points", skip_count);
  }
  // last point
  if (traj.points.size() > 1) {
    i =  traj.points.size() - 1;
    point.time = traj.points[i].time_from_start.toSec() - traj.points[i_1].time_from_start.toSec();
    point.positions = traj.points[i].positions;
    point.velocities = traj.points[i].velocities;
    if (point.time >= Tmin) {
      pvts.points.push_back(point);
    }
    else {
      point.time = traj.points[i].time_from_start.toSec() - traj.points[i_2].time_from_start.toSec();
      pvts.points.back() = point;
      ++skip_count;
      ROS_WARN("TM_ROS: Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = traj.points.back().time_from_start.toSec();
}

std::shared_ptr<tmrl::driver::PvtTraj> TmrRosNode::get_pvt_traj(const trajectory_msgs::JointTrajectory &traj, double Tmin)
{
  std::shared_ptr<tmrl::driver::PvtTraj> pvts = std::make_shared<tmrl::driver::PvtTraj>();

  set_pvt_traj(*pvts, traj, Tmin);

  return pvts;
}

void TmrRosNode::execute_traj(tmrl::driver::PvtTraj pvts)
{
  ROS_INFO("TM_ROS: trajectory thread begin");
  if (!is_fake_)
    iface_.run_pvt_traj(pvts);
  else
    iface_.fake_run_pvt_traj(pvts);

  if (has_goal_) {
    result_.error_code = result_.SUCCESSFUL;
    gh_.setSucceeded(result_);
    has_goal_ = false;
  }
  ROS_INFO("TM_ROS: trajectory thread end");
}
void TmrRosNode::execute_traj_call_by_ptr(std::shared_ptr<tmrl::driver::PvtTraj> pvts)
{
  ROS_INFO("TM_ROS: trajectory thread begin");
  if (!is_fake_)
    iface_.run_pvt_traj(*pvts);
  else
    iface_.fake_run_pvt_traj(*pvts);

  if (has_goal_) {
    result_.error_code = result_.SUCCESSFUL;
    gh_.setSucceeded(result_);
    has_goal_ = false;
  }
  ROS_INFO("TM_ROS: trajectory thread end");
}

// action callback

void TmrRosNode::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
  bool has_goal = has_goal_;
  auto goal = *(gh.getGoal());

  ROS_INFO("TM_ROS: on goal");

  // check robot

  if (!is_fake_) {

    if (!iface_.tmsct.client().is_connected()) {
      set_result(-101, "TM_ROS: Cannot accept new trajectories. TM_SCT is not connected");
      gh.setRejected(result_, result_.error_string);
      return;
    }
    if (iface_.state.has_error()) {
      set_result(-102, "TM_ROS: Cannot accept new trajectories. Robot has error");
      gh.setRejected(result_, result_.error_string);
      return;
    }

  }

  // check goal

  gh_ = gh;

  if (has_goal) {
    ROS_WARN("TM_ROS: Received new goal while still executing previous trajectory. Canceling previous trajectory");
    iface_.stop_pvt_traj();
    has_goal_ = false;
    set_result(-100, "TM_ROS: Received another trajectory");
    gh_.setAborted(result_, result_.error_string);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
  }
  if (!has_points(goal.trajectory)) {
    set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal without points");
    gh.setRejected(result_, result_.error_string);
    return;
  }
  if (!has_limited_velocities(goal.trajectory)) {
    set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal with velocities that are higher than max velocity");
    gh.setRejected(result_, result_.error_string);
    return;
  }
  if (!is_traj_finite(goal.trajectory)) {
    set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal with infinities or NaNs");
    gh.setRejected(result_, result_.error_string);
    return;
  }
  reorder_traj_joints(goal.trajectory);

  if (!is_start_positions_match(goal.trajectory, 0.01)) {
    set_result(result_.INVALID_GOAL, "TM_ROS: Start point doesn't match current pose");
    gh.setRejected(result_, result_.error_string);
    return;
  }

  gh_.setAccepted();

  has_goal_ = true;

  //tmrl::driver::PvtTraj pvts;
  //set_pvt_traj(pvts, goal.trajectory);
  //std::thread(std::bind(&TmRosNode::execute_traj, this, pvts)).detach();

  std::thread(std::bind(
    &TmrRosNode::execute_traj_call_by_ptr, this, get_pvt_traj(goal.trajectory))
  ).detach();

}

void TmrRosNode::cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
  //bool has_goal = has_goal_;

  ROS_INFO("TM_ROS: on cancel");

  iface_.stop_pvt_traj();
  has_goal_ = false;

  set_result(-99, "TM_ROS: Goal cancelled by client");
  gh.setCanceled(result_);
}

}