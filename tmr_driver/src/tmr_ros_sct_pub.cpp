
#include "tmr_driver/tmr_ros_node.h"

// tmsct publisher part

namespace tmr_ros
{

void TmrRosNode::tmsctCB(const tmrl::comm::TmsctPacket &pack)
{
  SctMsg &pm = sct_pm_;
  std::unique_lock<std::mutex> lck(sct_mtx_);
  
  pm.sct_msg.id = pack.id();
  pm.sct_msg.script = pack.script();

  lck.unlock();
  sct_cv_.notify_all();

  if (pack.has_error()) {
    tmrl_ERROR_STREAM("$TMSCT: err: " << pm.sct_msg.id << ", " << pm.sct_msg.script);
  }
  else {
    tmrl_INFO_STREAM("$TMSCT: res: " << pm.sct_msg.id << ", " << pm.sct_msg.script);
  }

  pm.sct_msg.header.stamp = ros::Time::now();
  pm.sct_pub.publish(pm.sct_msg);
}

void TmrRosNode::tmstaCB(const tmrl::comm::TmstaPacket &pack)
{
  SctMsg &pm = sct_pm_;
  std::unique_lock<std::mutex> lck(sta_mtx_);
  
  pm.sta_msg.subcmd = pack.subcmd();
  pm.sta_msg.subdata = pack.subdata();

  lck.unlock();
  sta_cv_.notify_all();

  tmrl_INFO_STREAM("$TMSTA: res: " << pm.sta_msg.subcmd << ", " << pm.sta_msg.subdata);

  pm.sta_msg.header.stamp = ros::Time::now();
  pm.sta_pub.publish(pm.sct_msg);
}

}
