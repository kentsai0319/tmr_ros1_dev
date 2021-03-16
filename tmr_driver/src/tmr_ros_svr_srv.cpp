
#include "tmr_driver/tmr_ros_node.h"

// tmsvr service part

namespace tmr_ros
{

bool TmrRosNode::connect_tmr(tmr_msgs::ConnectTMRRequest &req, tmr_msgs::ConnectTMRResponse &res)
{
  bool rb = true;
  int t_o = (int)(1000.0 * req.timeout);
  int t_v = (int)(1000.0 * req.timeval);

  tmrl::comm::ClientThread *client = nullptr;
  std::string hdr;

  switch (req.server) {
  case tmr_msgs::ConnectTMRRequest::TMSVR:
    client = &iface_.tmsvr;
    hdr = "TMSVR";
    break;
  case tmr_msgs::ConnectTMRRequest::TMSCT:
    client = &iface_.tmsct;
    hdr = "TMSCT";
    break;
  }
  if (client) {
    if (req.connect) {
      tmrl_INFO_STREAM("TM_ROS: (re)connect(" << t_o << ") " << hdr);
      client->stop();
      rb = client->start(t_o);
    }
    if (req.reconnect) {
      // set to tv
      tmrl_INFO_STREAM("TM_ROS: set " << hdr << " reconnect timeout " << t_o << "ms, timeval" << t_v << "ms");
    }
    else {
      // no reconnect
      client->set_reconnect_timeval(-1);
      tmrl_INFO_STREAM("TM_ROS: " << hdr << " set NOT reconnect");
    }
  }
  else {
    rb = false;
  }
  res.ok = rb;
  return rb;
}

bool TmrRosNode::write_item(tmr_msgs::WriteItemRequest &req, tmr_msgs::WriteItemResponse &res)
{
  bool rb = false;
  std::string content = req.item + "=" + req.value;
  rb = iface_.tmsvr.send_content(req.id, content);
  res.ok = rb;
  return rb;
}
bool TmrRosNode::ask_item(tmr_msgs::AskItemRequest &req, tmr_msgs::AskItemResponse &res)
{
  SvrMsg &pm = svr_pm_;
  bool rb = false;

  std::unique_lock<std::mutex> lck(svr_mtx_);
  svr_updated_ = false;
  lck.unlock();

  rb = iface_.tmsvr.send_content(req.id, req.item, tmrl::comm::TmsvrPacket::Mode::READ_STRING);

  lck.lock();
  if (rb && req.wait_time > 0.0) {
    if (!svr_updated_) {
      svr_cv_.wait_for(lck, std::chrono::duration<double>(req.wait_time));
    }
    if (!svr_updated_) {
      rb = false;
    }
    res.id = pm.svr_msg.id;
    res.value = pm.svr_msg.content;
  }
  svr_updated_ = false;

  res.ok = rb;
  return rb;
}

}
