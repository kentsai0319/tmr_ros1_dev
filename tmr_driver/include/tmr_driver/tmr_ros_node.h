
#include "tmr/tmr_print.h"
#include "tmr/tmr_driver.h"

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <visualization_msgs/InteractiveMarkerUpdate.h>

//#include <boost/chrono/chrono.hpp>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/condition_variable.hpp>

#include "tmr_driver/tmr_pose_conversion.h"

#include "tmr_msgs/FeedbackState.h"
#include "tmr_msgs/SvrResponse.h"
#include "tmr_msgs/SctResponse.h"
#include "tmr_msgs/StaResponse.h"
#include "tmr_msgs/PvtPoint.h"

#include "tmr_msgs/ConnectTM.h"
#include "tmr_msgs/WriteItem.h"
#include "tmr_msgs/AskItem.h"
#include "tmr_msgs/SendScript.h"
#include "tmr_msgs/SetEvent.h"
#include "tmr_msgs/SetIO.h"
//#include "tmr_msgs/SetPayload"
#include "tmr_msgs/SetPositions.h"
#include "tmr_msgs/AskSta.h"

#include "tmr_driver/tmr_ros_robot_hw.h"


namespace tmr_ros
{

class TmrRosNode {
protected:
  ros::NodeHandle nh_;
  //ros::NodeHandle nh_priv_;

  std::condition_variable svr_scv_;
  std::condition_variable sct_scv_;
  tmr::Driver iface_;

  const bool is_fake_;

  ////////////////////////////////
  // Action server
  ////////////////////////////////

  // follow_joint_trajectory
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh_;
  control_msgs::FollowJointTrajectoryResult result_;
  bool has_goal_;

  ////////////////////////////////
  // Param.
  ////////////////////////////////

  std::vector<std::string> joint_names_;
  std::string prefix_;
  std::string ns_;
  std::string base_frame_name_;
  std::string tool_frame_name_;

  ////////////////////////////////
  // Publisher
  ////////////////////////////////

  struct PubMsg {
    ros::Publisher fbs_pub;
    ros::Publisher joint_pub;
    ros::Publisher tool_pose_pub;
    ros::Publisher svr_pub;

    tmr_msgs::FeedbackState fbs_msg;
    sensor_msgs::JointState joint_msg;
    geometry_msgs::PoseStamped tool_pose_msg;

    //tf::Transform transform;
    tf::TransformBroadcaster tfbc;

    tmr_msgs::SvrResponse svr_msg;
  } pm_;

  struct SctMsg {
      ros::Publisher sct_pub;
      ros::Publisher sta_pub;

      tmr_msgs::SctResponse sct_msg;
      tmr_msgs::StaResponse sta_msg;
  } sm_;

  bool svr_updated_;
  boost::mutex svr_mtx_;
  boost::condition_variable svr_cv_;

  int pub_reconnect_timeout_ms_;
  int pub_reconnect_timeval_ms_;
  boost::thread pub_thread_;

  bool sta_updated_;
  boost::mutex sta_mtx_;
  boost::condition_variable sta_cv_;

  int sct_reconnect_timeout_ms_;
  int sct_reconnect_timeval_ms_;
  boost::thread sct_thread_;

  ////////////////////////////////
  // Subscriber
  ////////////////////////////////

  ros::Subscriber pvt_cmd_sub_;

  ////////////////////////////////
  // Service for connection
  ////////////////////////////////

  ros::ServiceServer connect_srv_;

  ////////////////////////////////
  // Service server
  ////////////////////////////////

  ros::ServiceServer write_item_srv_;
  ros::ServiceServer ask_item_srv_;

  ros::ServiceServer send_script_srv_;

  ros::ServiceServer set_event_srv_;
  ros::ServiceServer set_io_srv_;

  ros::ServiceServer set_positions_srv_;

  ros::ServiceServer ask_sta_srv_;

  ////////////////////////////////
  // ros_control
  ////////////////////////////////

  bool hw_iface_inited_;
  bool ros_control_running_;
public:
  boost::shared_ptr<tmr_hardware_interface::TmrRobotHW> hw_iface_;
  boost::shared_ptr<controller_manager::ControllerManager> ctrl_manager_;

  ////////////////////////////////
  // Init.
  ////////////////////////////////
public:
  TmrRosNode(const std::string &host, bool is_fake = false);
  ~TmrRosNode();
  void halt();

private:
  ////////////////////////////////
  // Action server
  ////////////////////////////////

  // helper function

  bool has_points(const trajectory_msgs::JointTrajectory &traj);
  bool has_limited_velocities(const trajectory_msgs::JointTrajectory &traj);
  bool is_traj_finite(const trajectory_msgs::JointTrajectory &traj);
  void reorder_traj_joints(trajectory_msgs::JointTrajectory &traj);
  bool is_start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);
  void set_result(int32_t err_code, const std::string &err_str);
  //void print_traj(const trajectory_msgs::JointTrajectory &traj);

  void set_pvt_traj(tmr::PvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj, double Tmin = 0.1);
  std::shared_ptr<tmr::PvtTraj> get_pvt_traj(const trajectory_msgs::JointTrajectory &traj, double Tmin = 0.1);
  void execute_traj(tmr::PvtTraj pvts); // TODO: use shared_ptr
  void execute_traj_call_by_ptr(std::shared_ptr<tmr::PvtTraj> pvts);

  // action function

  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

  ////////////////////////////////
  // Publisher
  ////////////////////////////////

  void fake_publisher();

  // for TMSVR
  void publish_fbs();
  void publish_svr();
  bool publish_func();
  void publisher();

  // for TMSCT
  void sct_msg();
  void sta_msg();
  bool sct_func();
  void sct_responsor();

  ////////////////////////////////
  // Service server
  ////////////////////////////////

  // for connection
  bool connect_tm(tmr_msgs::ConnectTMRequest &req, tmr_msgs::ConnectTMResponse &res);

  // for TMSVR
  bool write_item(tmr_msgs::WriteItemRequest &req, tmr_msgs::WriteItemResponse &res);
  bool ask_item(tmr_msgs::AskItemRequest &req, tmr_msgs::AskItemResponse &res);

  // for TMSCT
  bool send_script(tmr_msgs::SendScriptRequest &req, tmr_msgs::SendScriptResponse &res);

  bool set_event(tmr_msgs::SetEventRequest &req, tmr_msgs::SetEventResponse &res);
  bool set_io(tmr_msgs::SetIORequest &req, tmr_msgs::SetIOResponse &res);

  bool set_positions(tmr_msgs::SetPositionsRequest &req, tmr_msgs::SetPositionsResponse &res);

  bool ask_sta(tmr_msgs::AskStaRequest &req, tmr_msgs::AskStaResponse &res);

  ////////////////////////////////
  // Subscriber
  ////////////////////////////////

  void fake_pvt_cmd_cb(const tmr_msgs::PvtPointConstPtr &pvt);

  void pvt_cmd_cb(const tmr_msgs::PvtPointConstPtr &pvt);

  ////////////////////////////////
  // ros_control
  ////////////////////////////////

  void controlCB(ros::Time &time, ros::Duration &period);
public:
  void control_spin();
};

}