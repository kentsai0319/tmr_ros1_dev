#include "tmr_driver/tmr_ros_node.h"

namespace tmr_ros
{

////////////////////////////////
// Topic for TMSCT
////////////////////////////////

void TmrRosNode::sct_msg()
{
  SctMsg &sm = sm_;
  tmr::TmSctData &data = iface_.sct.sct_data;

  sm.sct_msg.id = data.script_id();
  sm.sct_msg.script = std::string{ data.script(), data.script_len() };

  if (data.has_error()) {
    tmr_ERROR_STREAM("TM_ROS: (TM_SCT): err: (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
  }
  else {
    tmr_INFO_STREAM("TM_ROS: (TM_SCT): res: (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
  }

  sm.sct_msg.header.stamp = ros::Time::now();
  sm.sct_pub.publish(sm.sct_msg);
}
void TmrRosNode::sta_msg()
{
  SctMsg &sm = sm_;
  tmr::TmStaData &data = iface_.sct.sta_data;
  {
    boost::lock_guard<boost::mutex> lck(sta_mtx_);
    sm.sta_msg.subcmd = data.subcmd_str();
    sm.sta_msg.subdata = std::string{ data.subdata(), data.subdata_len() };
    sta_updated_ = true;
  }
  sta_cv_.notify_all();

  tmr_INFO_STREAM("TM_ROS: (TM_STA): res: (" << sm.sta_msg.subcmd << "): " << sm.sta_msg.subdata);

  sm.sta_msg.header.stamp = ros::Time::now();
  sm.sta_pub.publish(sm.sta_msg);
}
bool TmrRosNode::sct_func()
{
  tmr::TmSctCommunication &sct = iface_.sct;
  int n;
  auto rc = sct.recv_spin_once(1000, &n);
  if (rc == tmr::CommRC::ERR ||
    rc == tmr::CommRC::NOTREADY ||
    rc == tmr::CommRC::NOTCONNECT) {
    return false;
  }
  else if (rc != tmr::CommRC::OK) {
    return true;
  }
  std::vector<tmr::TmPacket> &pack_vec = sct.packet_list();

  for (auto &pack : pack_vec) {
    switch (pack.type) {
    case tmr::TmPacket::Header::CPERR:
      tmr_INFO_STREAM("TM_ROS: (TM_SCT): CPERR");
      sct.err_data.set_CPError(pack.data.data(), pack.data.size());
      tmr_ERROR_STREAM(sct.err_data.error_code_str());

      // cpe response

      break;

    case tmr::TmPacket::Header::TMSCT:

      sct.err_data.error_code(tmr::TmCPError::Code::Ok);

      //TODO ? lock and copy for service response
      tmr::TmSctData::build_TmSctData(sct.sct_data, pack.data.data(), pack.data.size(), tmr::TmSctData::SrcType::Shallow);

      sct_msg();
      break;

    case tmr::TmPacket::Header::TMSTA:

      sct.err_data.error_code(tmr::TmCPError::Code::Ok);

      tmr::TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), tmr::TmStaData::SrcType::Shallow);

      sta_msg();
      break;

    default:
      tmr_WARN_STREAM("TM_ROS: (TM_SCT): invalid header");
      break;
    }
  }
  return true;
}
void TmrRosNode::sct_responsor()
{
  SctMsg &sm = sm_;
  tmr::TmSctCommunication &sct = iface_.sct;

  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

  tmr_INFO_STREAM("TM_ROS: sct_response thread begin");

  while (ros::ok()) {
    //bool reconnect = false;
    if (!sct.recv_init()) {
      tmr_INFO_STREAM("TM_ROS: (TM_SCT): is not connected");
    }
    while (ros::ok() && sct.is_connected()) {
      if (!sct_func()) break;
    }
    hw_iface_->halt();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    sct.Close();

    // reconnect == true
    if (!ros::ok()) break;
    if (sct_reconnect_timeval_ms_ <= 0) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
    tmr_INFO_STREAM("TM_ROS: (TM_SCT) reconnect in ");
    int cnt = 0;
    while (ros::ok() && cnt < sct_reconnect_timeval_ms_) {
      if (cnt % 1000 == 0) {
        tmr_INFO_STREAM(0.001 * (sct_reconnect_timeval_ms_ - cnt) << " sec... ");
      }
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
      ++cnt;
    }
    if (ros::ok() && sct_reconnect_timeval_ms_ >= 0) {
      tmr_INFO_STREAM("0 sec\nTM_ROS: (TM_SCT) connect(" << sct_reconnect_timeout_ms_ << "ms)...");
      sct.Connect(sct_reconnect_timeout_ms_);
    }
  }
  if (sct.is_connected()) {
    sct.send_script_exit();
  }
  sct.Close();
  std::cout << "TM_ROS: sct_response thread end\n";
}

}