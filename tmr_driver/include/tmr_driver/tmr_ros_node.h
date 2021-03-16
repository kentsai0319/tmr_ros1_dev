
#include "tmrl/driver/driver.h"
#include "tmrl/utils/logger.h"

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "tmr_msgs/FeedbackState.h"
#include "tmr_msgs/TmsvrResponse.h"
#include "tmr_msgs/TmsctResponse.h"
#include "tmr_msgs/TmstaResponse.h"
#include "tmr_msgs/PvtPoint.h"

#include "tmr_msgs/ConnectTMR.h"
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
public:
  explicit TmrRosNode(const std::string &host, bool is_fake = false);
  ~TmrRosNode();

  static const bool FAKE_ROBOT = true;
  static const bool REAL_ROBOT = false;

  void halt();

  // ros_control
  void control_spin();

private:
  ////////////////////////////////
  // Publisher
  ////////////////////////////////

  void fake_feedback();

  void feedbackCB(const tmrl::driver::RobotState &rs);
  void tmsvrCB(const tmrl::comm::TmsvrPacket &svr);

  void tmsctCB(const tmrl::comm::TmsctPacket &pack);
  void tmstaCB(const tmrl::comm::TmstaPacket &pack);

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

  void set_pvt_traj(tmrl::driver::PvtTraj &pvts,
    const trajectory_msgs::JointTrajectory &traj, double Tmin = 0.1);
  std::shared_ptr<tmrl::driver::PvtTraj> get_pvt_traj(
    const trajectory_msgs::JointTrajectory &traj, double Tmin = 0.1);
  void execute_traj(tmrl::driver::PvtTraj pvts); // TODO: use shared_ptr
  void execute_traj_call_by_ptr(std::shared_ptr<tmrl::driver::PvtTraj> pvts);

  // action callback

  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

  ////////////////////////////////
  // Service server
  ////////////////////////////////

  // for connection
  bool connect_tmr(tmr_msgs::ConnectTMRRequest &req, tmr_msgs::ConnectTMRResponse &res);

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
  // ros_control
  ////////////////////////////////

  void control_update(ros::Time &time, ros::Duration &period);

protected:
  const bool is_fake_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tmrl::driver::TmsvrClient svr_;
  tmrl::driver::TmsctClient sct_;
  tmrl::driver::Driver iface_;

  ////////////////////////////////
  // Param.
  ////////////////////////////////

  std::string ns_;
  std::string prefix_;

  std::vector<std::string> joint_names_;

  std::string base_frame_name_;
  std::string tool_frame_name_;

  ////////////////////////////////
  // Publisher
  ////////////////////////////////

  std::thread fake_fb_thd_;

  // tmsvr publisher msg.
  struct SvrMsg {
    ros::Publisher fbs_pub;
    ros::Publisher joint_pub;
    ros::Publisher tool_pose_pub;
    ros::Publisher svr_pub;

    tmr_msgs::FeedbackState fbs_msg;
    sensor_msgs::JointState joint_msg;
    geometry_msgs::PoseStamped tool_pose_msg;

    //tf::Transform transform;
    tf::TransformBroadcaster tfbc;

    tmr_msgs::TmsvrResponse svr_msg;
  } svr_pm_;

  std::mutex svr_mtx_;
  std::condition_variable svr_cv_;
  bool svr_updated_;

  // tmsct publisher msg.
  struct SctMsg {
    ros::Publisher sct_pub;
    ros::Publisher sta_pub;

    tmr_msgs::TmsctResponse sct_msg;
    tmr_msgs::TmstaResponse sta_msg;
  } sct_pm_;

  std::mutex sct_mtx_;
  std::condition_variable sct_cv_;
  bool sct_updated_;

  std::mutex sta_mtx_;
  std::condition_variable sta_cv_;
  bool sta_updated_;

  ////////////////////////////////
  // Action server
  ////////////////////////////////

  // follow_joint_trajectory

  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh_;
  control_msgs::FollowJointTrajectoryResult result_;
  bool has_goal_;

  ////////////////////////////////
  // Service server
  ////////////////////////////////

  ros::ServiceServer connect_srv_;

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

  std::shared_ptr<TmrRobotHW> hw_iface_;
  std::shared_ptr<controller_manager::ControllerManager> ctrl_manager_;
  bool hw_iface_inited_ = false;
  bool ros_control_running_ = false;
};

}
