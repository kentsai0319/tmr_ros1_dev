
#include "tmr_driver/tmr_ros_node.h"

namespace tmr_ros
{

TmrRosNode::TmrRosNode(const std::string &host, bool is_fake)
  : is_fake_(is_fake)
  , pnh_("~")
  , svr_(host)
  , sct_(host)
  , iface_(svr_, sct_)
  , as_(nh_, "follow_joint_trajectory",
      boost::bind(&TmrRosNode::goalCB, this, _1),
      boost::bind(&TmrRosNode::cancelCB, this, _1),
      false)
  , has_goal_(false)
{
  ROS_INFO_STREAM("TM_ROS: Namespace: " << nh_.getNamespace());

  ////////////////////////////////
  // Params
  ////////////////////////////////

  ns_ = "";
  if (ros::param::get("~ns", ns_)) {
    ROS_INFO_STREAM("TM_ROS: Set ns. to " << ns_);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: No ns. param.");
  }
  prefix_ = "";
  if (ros::param::get("~prefix", prefix_)) {
    ROS_INFO_STREAM("TM_ROS: Set prefix to " << prefix_);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: No prefix param.");
  }
  joint_names_.push_back(prefix_ + "joint_1");
  joint_names_.push_back(prefix_ + "joint_2");
  joint_names_.push_back(prefix_ + "joint_3");
  joint_names_.push_back(prefix_ + "joint_4");
  joint_names_.push_back(prefix_ + "joint_5");
  joint_names_.push_back(prefix_ + "joint_6");

  std::string frame_name = "base";
  if (ros::param::get("~base_frame", frame_name)) {
    ROS_INFO("TM_ROS: Set base_frame to %s", frame_name.c_str());
  }
  else {
    ROS_INFO_STREAM("TM_ROS: No base_frame param");
  }
  base_frame_name_ = prefix_ + frame_name;
  frame_name = "flange";
  if (ros::param::get("~tool_frame", frame_name)) {
    ROS_INFO("TM_ROS: Set tool_frame to %s", frame_name.c_str());
  }
  else {
    ROS_INFO_STREAM("TM_ROS: No tool_frame param");
  }
  tool_frame_name_ = prefix_ + frame_name;

  ////////////////////////////////
  // Publisher
  ////////////////////////////////
  
  // for TMSVR
  svr_pm_.fbs_msg.header.frame_id = base_frame_name_;
  svr_pm_.joint_msg.header.frame_id = base_frame_name_;
  svr_pm_.joint_msg.name = joint_names_;
  svr_pm_.tool_pose_msg.header.frame_id = base_frame_name_;

  svr_pm_.fbs_pub = nh_.advertise<tmr_msgs::FeedbackState>(ns_ + "feedback_state", 1);
  svr_pm_.joint_pub = nh_.advertise<sensor_msgs::JointState>(ns_ + "joint_states", 1);
  svr_pm_.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(ns_ + "tool_pose", 1);

  if (!is_fake_) {
    svr_pm_.svr_pub = nh_.advertise<tmr_msgs::TmsvrResponse>(ns_ + "tmsvr_response", 1);

    // for TMSCT
    sct_pm_.sct_pub = nh_.advertise<tmr_msgs::TmsctResponse>(ns_ + "tmsct_response", 1);
    sct_pm_.sta_pub = nh_.advertise<tmr_msgs::TmstaResponse>(ns_ + "tmsta_response", 1);
  }

  if (!is_fake) {
    auto ros_ok = [](){ return ros::ok(); };

    iface_.tmsvr.set_is_ok_predicate(ros_ok);
    iface_.tmsvr.set_feedback_callback(std::bind(&TmrRosNode::feedbackCB, this, std::placeholders::_1));
    iface_.tmsvr.set_response_callback(std::bind(&TmrRosNode::tmsvrCB, this, std::placeholders::_1));
    iface_.tmsvr.set_read_callback    (std::bind(&TmrRosNode::tmsvrCB, this, std::placeholders::_1));
    iface_.tmsvr.start();
    
    iface_.tmsct.set_is_ok_predicate(ros_ok);
    iface_.tmsct.set_tmsct_callback(std::bind(&TmrRosNode::tmsctCB, this, std::placeholders::_1));
    iface_.tmsct.set_tmsta_callback(std::bind(&TmrRosNode::tmstaCB, this, std::placeholders::_1));
    iface_.tmsct.start();
  }
  else {
    // fake_feedbackCB thread
    fake_fb_thd_ = std::thread{std::bind(&TmrRosNode::fake_feedback, this)};
  }

  ////////////////////////////////
  // Action server
  ////////////////////////////////
  as_.start();

  ////////////////////////////////
  // Subscriber
  ////////////////////////////////

  if (!is_fake_) {
    // for connection
    connect_srv_ = nh_.advertiseService(ns_ + "tmr/connect_tm", &TmrRosNode::connect_tmr, this);

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

  hw_iface_ = std::make_shared<TmrRobotHW>(nh_, &iface_);
  if (!hw_iface_inited_ && hw_iface_->init(nh_, nh_)) {
    ROS_INFO_STREAM("TM_ROS: TmrRobotHW is inited");
    hw_iface_inited_ = true;
  }
  if (hw_iface_inited_) {
    ros_control_running_ = true;
  }
  ctrl_manager_ = std::make_shared<controller_manager::ControllerManager>(hw_iface_.get(), nh_);

} // TmrRosNode ctor

TmrRosNode::~TmrRosNode()
{
  halt();
}

void TmrRosNode::halt()
{
  ros_control_running_ = false;

  iface_.stop_pvt_traj();

  // notify all

  sta_updated_ = true;
  sta_cv_.notify_all();

  //sct_updated_ = true;
  //sct_cv_.notify_all();

  svr_updated_ = true;
  svr_cv_.notify_all();

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (!is_fake_) {
    iface_.halt();
  }
  else {
    if (fake_fb_thd_.joinable()) {
      fake_fb_thd_.join();
    }
  }
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

  while (ros_control_running_ && ros::ok()) {
    // reset
    period.fromSec(T);

    while (ros::ok()) {
      time = ros::Time::now();
      clock_last = std::chrono::steady_clock::now();

      control_update(time, period);

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
void TmrRosNode::control_update(ros::Time &time, ros::Duration &period)
{
  hw_iface_->read(time, period);

  ctrl_manager_->update(time, period);

  hw_iface_->write(time, period);
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
  bool is_fake = false;

  if (ros::param::get("~sim", is_fake)) {
    ROS_INFO_STREAM("TM_ROS: Set is_fake to " << is_fake);
  }
  else {
    ROS_INFO_STREAM("TM_ROS: No sim. param.");
  }
  if (is_fake) {
    ROS_INFO_STREAM("TM_ROS: Fake mode");
  }
  else {
    if (ros::param::get("~robot_ip_address", host)) {
      ROS_INFO_STREAM("TM_ROS: Set robot_ip to " << host);
    }
    else {
      ROS_INFO_STREAM("TM_ROS: No robot_ip_address param.");
      if (argc > 1) { host = argv[1]; }
    }
    if (host.length() < 7) {
      ROS_ERROR_STREAM("TM_ROS: Invalid ip address");
      return 0;
    }
    ROS_INFO_STREAM("TM_ROS: robot_ip:=" << host);
  }

  tmrl::utils::get_logger().set_level(tmrl::utils::logger::level::INFO);
  tmrl::utils::get_logger().setup_debug(_print_debug);
  tmrl::utils::get_logger().setup_info (_print_info);
  tmrl::utils::get_logger().setup_warn (_print_warn);
  tmrl::utils::get_logger().setup_error(_print_error);
  tmrl::utils::get_logger().setup_fatal(_print_fatal);

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
