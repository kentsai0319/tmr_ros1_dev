
#include "tmr_driver/tmr_ros_node.h"

// tmsvr publisher part

namespace tmr_ros
{

void TmrRosNode::fake_feedback()
{
   SvrMsg &pm = svr_pm_;
   tmrl::driver::RobotState &rs = iface_.state;

  tmrl_INFO_STREAM("TM_ROS: fake publisher thread begin");

  while (ros::ok()) {
    tmrl::driver::RobotState::Ulock lck(rs.mtx);
    pm.fbs_msg.joint_pos = tmrl::to_vectorXd(rs.joint_angle());
    pm.fbs_msg.joint_vel = tmrl::to_vectorXd(rs.joint_speed());
    pm.fbs_msg.joint_tor = tmrl::to_vectorXd(rs.joint_torque());
    lck.unlock();

    pm.fbs_msg.header.stamp = ros::Time::now();
    pm.fbs_pub.publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_pub.publish(pm.joint_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  std::cout << "TM_ROS: fake publisher thread end\n";
}

void TmrRosNode::feedbackCB(const tmrl::driver::RobotState &rs)
{
  SvrMsg &pm = svr_pm_;

  // Publish feedback state

  pm.fbs_msg.is_svr_connected = iface_.tmsvr.client().is_connected();
  pm.fbs_msg.is_sct_connected = iface_.tmsct.client().is_connected();

  pm.fbs_msg.joint_pos = tmrl::to_vectorXd(rs.joint_angle());
  pm.fbs_msg.joint_vel = tmrl::to_vectorXd(rs.joint_speed());
  pm.fbs_msg.joint_tor = tmrl::to_vectorXd(rs.joint_torque());
  //pm.fbs_msg.flange_pose = tmrl::to_vectorXd(rs.flange_pose()); 
  pm.fbs_msg.tool_pose = tmrl::to_vectorXd(rs.tool_pose());
  pm.fbs_msg.tcp_speed = tmrl::to_vectorXd(rs.tcp_speed_vec());
  pm.fbs_msg.tcp_force = tmrl::to_vectorXd(rs.tcp_force_vec());
  pm.fbs_msg.robot_link = rs.is_linked();
  pm.fbs_msg.robot_error = rs.has_error();
  pm.fbs_msg.project_run = rs.is_project_running();
  pm.fbs_msg.project_pause = rs.is_project_paused();
  pm.fbs_msg.safetyguard_a = rs.is_safeguard_A();
  pm.fbs_msg.e_stop = rs.is_EStop();
  pm.fbs_msg.camera_light = rs.camera_light();
  pm.fbs_msg.error_code = rs.error_code();
  pm.fbs_msg.project_speed = rs.project_speed();
  pm.fbs_msg.ma_mode = rs.ma_mode();
  pm.fbs_msg.robot_light = rs.robot_light();
  pm.fbs_msg.cb_digital_output = tmrl::to_vectorX(rs.ctrller_DO());
  pm.fbs_msg.cb_digital_input = tmrl::to_vectorX(rs.ctrller_DI());
  pm.fbs_msg.cb_analog_output = tmrl::to_vectorX(rs.ctrller_AO());
  pm.fbs_msg.cb_analog_input = tmrl::to_vectorX(rs.ctrller_AI());
  pm.fbs_msg.ee_digital_output = tmrl::to_vectorX(rs.ee_DO());
  pm.fbs_msg.ee_digital_input = tmrl::to_vectorX(rs.ee_DI());
  //pm.fbs_msg.ee_analog_output = tmrl::to_vectorX(rs.ee_AO());
  pm.fbs_msg.ee_analog_input = tmrl::to_vectorX(rs.ee_AI());
  pm.fbs_msg.error_content = rs.error_content();

  pm.fbs_msg.header.stamp = ros::Time::now();
  pm.fbs_pub.publish(pm.fbs_msg);

  // Publish joint state
  pm.joint_msg.position = pm.fbs_msg.joint_pos;
  pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
  pm.joint_msg.effort = pm.fbs_msg.joint_tor;
  pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
  pm.joint_pub.publish(pm.joint_msg);

  // Publish tool pose
  //TmrPoseConversion::msg_from_vec(pm.tool_pose_msg.pose, pm.fbs_msg.tool_pose);
  auto &pose = pm.fbs_msg.tool_pose;
  tf::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  tf::Transform Tbt(quat, tf::Vector3(pose[0], pose[1], pose[2]));
  tf::poseTFToMsg(Tbt, pm.tool_pose_msg.pose);
  pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
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

void TmrRosNode::tmsvrCB(const tmrl::comm::TmsvrPacket &pack)
{
  SvrMsg &pm = svr_pm_;

  std::unique_lock<std::mutex> lck(svr_mtx_);
  pm.svr_msg.id = pack.transaction_id();
  pm.svr_msg.mode = (int)(pack.mode());
  pm.svr_msg.content = pack.content();
  pm.svr_msg.error_code = (int)(pack.errcode());

  svr_updated_ = true;
  lck.unlock();
  svr_cv_.notify_all();

  tmrl_INFO_STREAM("$TMSVR: " << pm.svr_msg.id
    << ", " << (int)(pm.svr_msg.mode) << ", " << pm.svr_msg.content);

  pm.svr_msg.header.stamp = ros::Time::now();
  pm.svr_pub.publish(pm.svr_msg);
}

}
