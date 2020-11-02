#include "tmr_driver/tmr_ros_node.h"

namespace tmr_ros
{

void TmrRosNode::fake_publisher()
{
  PubMsg &pm = pm_;
  tmr::TmSvrCommunication &svr = iface_.svr;
  tmr::RobotState &state = iface_.state;

  tmr_INFO_STREAM("TM_ROS: fake publisher thread begin");

  pm.joint_msg.name = joint_names_;
  pm.joint_msg.position.assign(joint_names_.size(), 0.0);
  pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
  pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

  while (ros::ok()) {
    // Publish feedback state
    pm.fbs_msg.header.stamp = ros::Time::now();
    {
      std::lock_guard<std::mutex> lck(state.mtx);
      pm.fbs_msg.joint_pos = state.joint_angle();
      pm.fbs_msg.joint_vel = state.joint_speed();
      pm.fbs_msg.joint_tor = state.joint_torque();
    }
    pm.fbs_pub.publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_pub.publish(pm.joint_msg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
  std::cout << "TM_ROS: fake publisher thread end\n";
}

////////////////////////////////
// Topic for TMSVR
////////////////////////////////

void TmrRosNode::publish_fbs()
{
  PubMsg &pm = pm_;
  tmr::RobotState &state = iface_.state;

  // Publish feedback state
  pm.fbs_msg.header.stamp = ros::Time::now();

  pm.fbs_msg.is_svr_connected = iface_.svr.is_connected();
  pm.fbs_msg.is_sct_connected = iface_.sct.is_connected();

  pm.fbs_msg.joint_pos = state.joint_angle();
  pm.fbs_msg.joint_vel = state.joint_speed();
  pm.fbs_msg.joint_tor = state.joint_torque();
  //pm.fbs_msg.flange_pose = state.flange_pose(); 
  pm.fbs_msg.tool_pose = state.tool_pose();
  pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
  pm.fbs_msg.tcp_force = state.tcp_force_vec();
  pm.fbs_msg.robot_link = state.is_linked();
  pm.fbs_msg.robot_error = state.has_error();
  pm.fbs_msg.project_run = state.is_project_running();
  pm.fbs_msg.project_pause = state.is_project_paused();
  pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
  pm.fbs_msg.e_stop = state.is_EStop();
  pm.fbs_msg.camera_light = state.camera_light();
  pm.fbs_msg.error_code = state.error_code();
  pm.fbs_msg.project_speed = state.project_speed();
  pm.fbs_msg.ma_mode = state.ma_mode();
  pm.fbs_msg.robot_light = state.robot_light();
  pm.fbs_msg.cb_digital_output = state.ctrller_DO();
  pm.fbs_msg.cb_digital_input = state.ctrller_DI();
  pm.fbs_msg.cb_analog_output = state.ctrller_AO();
  pm.fbs_msg.cb_analog_input = state.ctrller_AI();
  pm.fbs_msg.ee_digital_output = state.ee_DO();
  pm.fbs_msg.ee_digital_input = state.ee_DI();
  //pm.fbs_msg.ee_analog_output = state.ee_AO();
  pm.fbs_msg.ee_analog_input = state.ee_AI();
  pm.fbs_msg.error_content = state.error_content();
  pm.fbs_pub.publish(pm.fbs_msg);

  // Publish joint state
  pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
  pm.joint_msg.position = pm.fbs_msg.joint_pos;
  pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
  pm.joint_msg.effort = pm.fbs_msg.joint_tor;
  pm.joint_pub.publish(pm.joint_msg);

  // Publish tool pose
  //TmrPoseConversion::msg_from_vec(pm.tool_pose_msg.pose, pm.fbs_msg.tool_pose);
  auto &pose = pm.fbs_msg.tool_pose;
  tf::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  tf::Transform Tbt(quat, tf::Vector3(pose[0], pose[1], pose[2]));
  tf::poseTFToMsg(Tbt, pm.tool_pose_msg.pose);
  pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
  pm.tool_pose_msg.header.frame_id = base_frame_name_;
  /*pm.tool_pose_msg.pose.position.x = pose[0];
  pm.tool_pose_msg.pose.position.y = pose[1];
  pm.tool_pose_msg.pose.position.z = pose[2];
  pm.tool_pose_msg.pose.orientation.x = quat.x();
  pm.tool_pose_msg.pose.orientation.y = quat.y();
  pm.tool_pose_msg.pose.orientation.z = quat.z();
  pm.tool_pose_msg.pose.orientation.w = quat.w();*/
  pm.tool_pose_pub.publish(pm.tool_pose_msg);

  // Boardcast transform (tool pose)
  //TmrPoseConversion::tf_from_vec(pm.transform, pm.fbs_msg.tool_pose);
  //pm.transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
  //pm.transform.setRotation(quat);
  pm.tfbc.sendTransform(tf::StampedTransform(
    Tbt, pm.joint_msg.header.stamp, base_frame_name_, tool_frame_name_));

}
void TmrRosNode::publish_svr()
{
  PubMsg &pm = pm_;
  tmr::TmSvrData &data = iface_.svr.data;
  {
    boost::lock_guard<boost::mutex> lck(svr_mtx_);
    pm.svr_msg.id = data.transaction_id();
    pm.svr_msg.mode = (int)(data.mode());
    pm.svr_msg.content = std::string{ data.content(), data.content_len() };
    pm.svr_msg.error_code = (int)(data.error_code());
    svr_updated_ = true;
  }
  svr_cv_.notify_all();

  tmr_INFO_STREAM("TM_ROS: (TM_SVR): (" <<
    pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);

  pm.svr_msg.header.stamp = ros::Time::now();
  pm.svr_pub.publish(pm.svr_msg);
}
bool TmrRosNode::publish_func()
{
  //PubMsg &pm = pm_;
  tmr::TmSvrCommunication &svr = iface_.svr;
  int n;
  auto rc = svr.recv_spin_once(1000, &n);
  if (rc == tmr::CommRC::ERR ||
    rc == tmr::CommRC::NOTREADY ||
    rc == tmr::CommRC::NOTCONNECT) {
    return false;
  }
  else if (rc != tmr::CommRC::OK) {
      return true;
  }
  bool fbs = false;
  std::vector<tmr::TmPacket> &pack_vec = svr.packet_list();

  for (auto &pack : pack_vec) {
    if (pack.type == tmr::TmPacket::Header::CPERR) {
      tmr_INFO_STREAM("TM_ROS: (TM_SVR): CPERR");
      svr.err_data.set_CPError(pack.data.data(), pack.data.size());
      tmr_ERROR_STREAM(svr.err_data.error_code_str());

      // cpe response

    }
    else if (pack.type == tmr::TmPacket::Header::TMSVR) {

      svr.err_data.error_code(tmr::TmCPError::Code::Ok);

      //TODO ? lock and copy for service response
      tmr::TmSvrData::build_TmSvrData(svr.data, pack.data.data(), pack.data.size(), tmr::TmSvrData::SrcType::Shallow);

      if (svr.data.is_valid()) {
        switch (svr.data.mode()) {
        case tmr::TmSvrData::Mode::RESPONSE:
          //tmr_INFO_STREAM("TM_SVR: RESPONSE (" << svr.data.transaction_id() << "): [" <<
          //  (int)(svr.data.error_code()) << "]: " << std::string(svr.data.content(), svr.data.content_len()));
          publish_svr();
          break;
        case tmr::TmSvrData::Mode::BINARY:
          svr.state.mtx_deserialize(svr.data.content(), svr.data.content_len());
          fbs = true;
          break;
        case tmr::TmSvrData::Mode::READ_STRING:
        case tmr::TmSvrData::Mode::READ_JSON:
          publish_svr();
          break;
        default:
          tmr_WARN_STREAM("TM_ROS: (TM_SVR): (" <<
            svr.data.transaction_id() << "): invalid mode (" << (int)(svr.data.mode()) << ")");
          break;
        }
      }
      else {
        tmr_WARN_STREAM("TM_ROS: (TM_SVR): invalid data");
      }
    }
    else {
      tmr_WARN_STREAM("TM_ROS: (TM_SVR): invalid header");
    }
  }
  if (fbs) {
    publish_fbs();
  }
  return true;
}
void TmrRosNode::publisher()
{
  PubMsg &pm = pm_;
  tmr::TmSvrCommunication &svr = iface_.svr;

  tmr_INFO_STREAM("TM_ROS: publisher thread begin");

  pm.joint_msg.name = joint_names_;
  pm.joint_msg.position.assign(joint_names_.size(), 0.0);
  pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
  pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

  while (ros::ok()) {
    //bool reconnect = false;
    if (!svr.recv_init()) {
      tmr_INFO_STREAM("TM_ROS: (TM_SVR): is not connected");
    }
    while (ros::ok() && svr.is_connected()) {
      if (!publish_func()) break;
    }
    svr.Close();

    // reconnect == true
    if (!ros::ok()) break;
    if (pub_reconnect_timeval_ms_ <= 0) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
    tmr_INFO_STREAM("TM_ROS: (TM_SVR): reconnect in ");
    int cnt = 0;
    while (ros::ok() && cnt < pub_reconnect_timeval_ms_) {
      if (cnt % 500 == 0) {
        tmr_INFO_STREAM(0.001 * (pub_reconnect_timeval_ms_ - cnt) << " sec...");
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
      ++cnt;
    }
    if (ros::ok() && pub_reconnect_timeval_ms_ >= 0) {
      tmr_INFO_STREAM("0 sec\nTM_ROS: (TM_SVR): connect" << pub_reconnect_timeout_ms_ << "ms)...");
      svr.Connect(pub_reconnect_timeout_ms_);
    }
  }
  svr.Close();
  std::cout << "TM_ROS: publisher thread end\n";
}

}
