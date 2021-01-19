
#include "tmr_driver/tmr_ros_node.h"

namespace tmr_ros
{

////////////////////////////////
// Node
////////////////////////////////

TmrRosNode::TmrRosNode(const std::string &host, bool is_fake)
  //: nh_("~"")
  //, iface_(host, nullptr, &sct_cv_)
  : iface_(host, nullptr, nullptr)
  , is_fake_(is_fake)
  , as_(nh_, "follow_joint_trajectory",
    boost::bind(&TmrRosNode::goalCB, this, _1),
    boost::bind(&TmrRosNode::cancelCB, this, _1),
    false)
  , has_goal_(false)
{
  ROS_INFO_STREAM("TM_ROS: node namespace: " << nh_.getNamespace());

  ////////////////////////////////
  // Param.
  ////////////////////////////////

  prefix_ = "";
  if (ros::param::get("~prefix", prefix_)) {
    ROS_INFO_STREAM("TM_ROS: set prefix to " << prefix_);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no prefix param.");
  }
  ns_ = "";
  if (ros::param::get("~ns", ns_)) {
    ROS_INFO_STREAM("TM_ROS: set ns. to " << ns_);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no ns. param.");
  }
  joint_names_.push_back(prefix_ + "joint_1");
  joint_names_.push_back(prefix_ + "joint_2");
  joint_names_.push_back(prefix_ + "joint_3");
  joint_names_.push_back(prefix_ + "joint_4");
  joint_names_.push_back(prefix_ + "joint_5");
  joint_names_.push_back(prefix_ + "joint_6");

  std::string frame_name = "base";
  if (ros::param::get("~base_frame", frame_name)) {
    ROS_INFO("TM_ROS: set base_frame to %s", frame_name.c_str());
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no base_frame param");
  }
  base_frame_name_ = prefix_ + frame_name;
  frame_name = "flange";
  if (ros::param::get("~tool_frame", frame_name)) {
    ROS_INFO("TM_ROS: set tool_frame to %s", frame_name.c_str());
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no tool_frame param");
  }
  tool_frame_name_ = prefix_ + frame_name;

  ////////////////////////////////
  // Tmr::Driver
  ////////////////////////////////
  if (!is_fake_) {
    iface_.start(5000);
  }
  else {
    std::vector<double> zeros(iface_.state.DOF);
    iface_.svr.state.set_joint_states(zeros, zeros, zeros);
  }

  ////////////////////////////////
  // Publisher
  ////////////////////////////////

  // for TMSVR
  pm_.fbs_pub = nh_.advertise<tmr_msgs::FeedbackState>(ns_ + "feedback_states", 1);
  pm_.joint_pub = nh_.advertise<sensor_msgs::JointState>(ns_ + "joint_states", 1);
  pm_.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(ns_ + "tool_pose", 1);

  if (!is_fake_) {
    pm_.svr_pub = nh_.advertise<tmr_msgs::SvrResponse>(ns_ + "tmr/svr_response", 1);

    // for TMSCT
    sm_.sct_pub = nh_.advertise<tmr_msgs::SctResponse>(ns_ + "tmr/sct_response", 1);
    sm_.sta_pub = nh_.advertise<tmr_msgs::StaResponse>(ns_ + "tmr/sta_response", 1);
  }

  //
  svr_updated_ = false;
  pub_reconnect_timeout_ms_ = 1000;
  pub_reconnect_timeval_ms_ = 3000;
  if (!is_fake_)
    pub_thread_ = boost::thread(boost::bind(&TmrRosNode::publisher, this));
  else
    pub_thread_ = boost::thread(boost::bind(&TmrRosNode::fake_publisher, this));

  sta_updated_ = false;
  sct_reconnect_timeout_ms_ = 1000;
  sct_reconnect_timeval_ms_ = 3000;
  if (!is_fake_) {
    sct_thread_ = boost::thread(boost::bind(&TmrRosNode::sct_responsor, this));
  }

  ////////////////////////////////
  // Action server
  ////////////////////////////////
  as_.start();

  ////////////////////////////////
  // Subscriber
  ////////////////////////////////

  if (!is_fake_)
    pvt_cmd_sub_ = nh_.subscribe("tmr/pvt_command", 1, &TmrRosNode::pvt_cmd_cb, this);
  else
    pvt_cmd_sub_ = nh_.subscribe("tmr/pvt_command", 1, &TmrRosNode::fake_pvt_cmd_cb, this);

  ////////////////////////////////
  // Service server
  ////////////////////////////////

  if (!is_fake_) {

  // for connection
  connect_srv_ = nh_.advertiseService(ns_ + "tmr/connect_tm", &TmrRosNode::connect_tm, this);

  // for TMSVR
  write_item_srv_ = nh_.advertiseService(ns_ + "tmr/write_item", &TmrRosNode::write_item, this);
  ask_item_srv_ = nh_.advertiseService(ns_ + "tmr/ask_item", &TmrRosNode::ask_item, this);

  // for TMSCT
  send_script_srv_ = nh_.advertiseService(ns_ + "tmr/send_script", &TmrRosNode::send_script, this);

  set_event_srv_ = nh_.advertiseService(ns_ + "tmr/set_event", &TmrRosNode::set_event, this);
  set_io_srv_ = nh_.advertiseService(ns_ + "tmr/set_io", &TmrRosNode::set_io, this);

  set_positions_srv_ = nh_.advertiseService(ns_ + "tmr/set_positions", &TmrRosNode::set_positions, this);

  ask_sta_srv_ = nh_.advertiseService(ns_ + "tmr/ask_sta", &TmrRosNode::ask_sta, this);

  }

  ////////////////////////////////
  // ros_control
  ////////////////////////////////

  hw_iface_ = boost::make_shared<tmr_hardware_interface::TmrRobotHW>(nh_, &iface_);
  ctrl_manager_ = boost::make_shared<controller_manager::ControllerManager>(hw_iface_.get(), nh_);
  if (!hw_iface_inited_ && hw_iface_->init(nh_, nh_)) {
    hw_iface_inited_ = true;
  }
  if (hw_iface_inited_) {
    ros_control_running_ = true;
  }
}
TmrRosNode::~TmrRosNode()
{
  halt();
}
void TmrRosNode::halt()
{
  std::cout << "TM_ROS: halt\n";

  if (sct_thread_.joinable()) {
    sct_thread_.join();
  }
  sta_updated_ = true;
  sta_cv_.notify_all();

  if (pub_thread_.joinable()) {
    pub_thread_.join();
  }
  svr_updated_ = true;
  svr_cv_.notify_all();

  if (is_fake_) return;

  // Driver
  iface_.halt();
}

////////////////////////////////
// Action
////////////////////////////////

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
  auto q_act = iface_.state.mtx_joint_angle();

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

void TmrRosNode::set_pvt_traj(tmr::PvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj, double Tmin)
{
  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  tmr::PvtPoint point;

  if (traj.points.size() == 0) return;

  pvts.mode = tmr::PvtMode::Joint;

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

std::shared_ptr<tmr::PvtTraj> TmrRosNode::get_pvt_traj(const trajectory_msgs::JointTrajectory &traj, double Tmin)
{
  std::shared_ptr<tmr::PvtTraj> pvts = std::make_shared<tmr::PvtTraj>();

  set_pvt_traj(*pvts, traj, Tmin);

  return pvts;
}

void TmrRosNode::execute_traj(tmr::PvtTraj pvts)
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
void TmrRosNode::execute_traj_call_by_ptr(std::shared_ptr<tmr::PvtTraj> pvts)
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

// action function

void TmrRosNode::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
  bool has_goal = has_goal_;
  auto goal = *(gh.getGoal());

  ROS_INFO("TM_ROS: on goal");

  // check robot

  if (!is_fake_) {

    if (!iface_.sct.is_connected()) {
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

  //tmr::PvtTraj pvts;
  //set_pvt_traj(pvts, goal.trajectory);
  //boost::thread(boost::bind(&TmRosNode::execute_traj, this, pvts)).detach();

  boost::thread(boost::bind(
    &TmrRosNode::execute_traj_call_by_ptr, this, get_pvt_traj(goal.trajectory))
  ).detach();
}

void TmrRosNode::cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
  bool has_goal = has_goal_;

  ROS_INFO("TM_ROS: on cancel");

  iface_.stop_pvt_traj();
  has_goal_ = false;

  set_result(-99, "TM_ROS: Goal cancelled by client");
  gh.setCanceled(result_);
}

void TmrRosNode::fake_pvt_cmd_cb(const tmr_msgs::PvtPointConstPtr &pvt)
{
}
void TmrRosNode::pvt_cmd_cb(const tmr_msgs::PvtPointConstPtr &pvt)
{
}

void TmrRosNode::controlCB(ros::Time &time, ros::Duration &period)
{
  hw_iface_->read(time, period);

  ctrl_manager_->update(time, period);

  hw_iface_->write(time, period);
}
void TmrRosNode::control_spin()
{
  const double T = 0.1;
  double t;
  ros::Time time;
  ros::Duration period;
  ros::Duration duration;
  auto clock_last = std::chrono::steady_clock::now();
  auto clock_now = clock_last;

  while (ros::ok()) {
    if (ros_control_running_) {
      // reset
      period.fromSec(T);

      while (ros::ok() && ros_control_running_) {
        time = ros::Time::now();
        clock_last = std::chrono::steady_clock::now();

        controlCB(time, period);

        clock_now = std::chrono::steady_clock::now();
        period.fromSec(
          std::chrono::duration_cast<std::chrono::duration<double>>(clock_now - clock_last).count()
        );
        clock_last = clock_now;
        t = T - period.toSec();
        if (t < 0.0) { t = 0.0; }
        duration.fromSec(t);
        duration.sleep();
      }
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
}

}

////////////////////////////////

void _print_debug(const std::string &msg) { ROS_DEBUG_STREAM(msg); }
void _print_info (const std::string &msg) { ROS_INFO_STREAM (msg); }
void _print_warn (const std::string &msg) { ROS_WARN_STREAM (msg); }
void _print_error(const std::string &msg) { ROS_ERROR_STREAM(msg); }
void _print_fatal(const std::string &msg) { ROS_FATAL_STREAM(msg); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tmr_driver");

  //for (int i = 0; i < argc; ++i){ ROS_INFO("arg[%d]: %s", i, argv[i]); }

  std::string host;
  if (ros::param::get("~ip_address", host)) {
    ROS_INFO_STREAM("TM_ROS: set ip to " << host);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no ip_address param.");
    if (argc > 1) { host = argv[1]; }
  }
  bool is_fake = false;
  if (ros::param::get("~sim", is_fake)) {
    ROS_INFO_STREAM("TM_ROS: set is_fake to " << is_fake);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: no sim. param.");
  }
  if (is_fake) {
    ROS_INFO_STREAM("TM_ROS: fake mode");
  }
  else {
    if (host.length() < 7) {
      ROS_ERROR_STREAM("TM_ROS: invalid ip address");
      return 0;
    }
    ROS_INFO_STREAM("TM_ROS: ip:=" << host);
  }

  tmr::logger::ref().setup_level(tmr::logger::level::INFO);
  tmr::logger::ref().setup_debug(_print_debug);
  tmr::logger::ref().setup_info (_print_info);
  tmr::logger::ref().setup_warn (_print_warn);
  tmr::logger::ref().setup_error(_print_error);
  tmr::logger::ref().setup_fatal(_print_fatal);

  tmr_ros::TmrRosNode robot(host, is_fake);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  robot.control_spin();

  //ros::waitForShutdown();

  robot.halt();

  spinner.stop();
  std::cout << "TM_ROS: shutdown\n";
  return 0;
}
