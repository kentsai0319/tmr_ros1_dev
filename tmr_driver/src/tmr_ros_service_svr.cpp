#include "tmr_driver/tmr_ros_node.h"

namespace tmr_ros
{

////////////////////////////////
// Service for connection
////////////////////////////////

bool TmrRosNode::connect_tm(tmr_msgs::ConnectTMRequest &req, tmr_msgs::ConnectTMResponse &res)
{
  bool rb = true;
  int t_o = (int)(1000.0 * req.timeout);
  int t_v = (int)(1000.0 * req.timeval);
  switch (req.server) {
  case tmr_msgs::ConnectTMRequest::TMSVR:
    if (req.connect) {
      tmr_INFO_STREAM("TM_ROS: (re)connect(" << t_o <<") TMSVR");
      iface_.svr.halt();
      rb = iface_.svr.start(t_o);
    }
    if (req.reconnect) {
      pub_reconnect_timeout_ms_ = t_o;
      pub_reconnect_timeval_ms_ = t_v;
      tmr_INFO_STREAM("TM_ROS: set SVR reconnect timeout " << t_o << "ms, timeval" << t_v << "ms");
    }
    else {
      // no reconnect
      pub_reconnect_timeval_ms_ = -1;
      tmr_INFO_STREAM("TM_ROS: set SVR NOT reconnect");
    }
    break;
  case tmr_msgs::ConnectTMRequest::TMSCT:
    if (req.connect) {
      tmr_INFO_STREAM("TM_ROS: (re)connect(" << t_o << ") TMSCT");
      iface_.sct.halt();
      rb = iface_.sct.start(t_o);
    }
    if (req.reconnect) {
      sct_reconnect_timeout_ms_ = t_o;
      sct_reconnect_timeval_ms_ = t_v;
      tmr_INFO_STREAM("TM_ROS: set SCT reconnect timeout " << t_o << "ms, timeval" << t_v << "ms");
    }
    else {
      // no reconnect
      sct_reconnect_timeval_ms_ = -1;
      tmr_INFO_STREAM("TM_ROS: set SCT NOT reconnect");
    }
    break;
  }
  res.ok = rb;
  return rb;
}

////////////////////////////////
// Service for TMSVR
////////////////////////////////

bool TmrRosNode::write_item(tmr_msgs::WriteItemRequest &req, tmr_msgs::WriteItemResponse &res)
{
  bool rb = false;
  std::string content = req.item + "=" + req.value;
  rb = (iface_.svr.send_content_str(req.id, content) == iface_.RC_OK);
  res.ok = rb;
  return rb;
}
bool TmrRosNode::ask_item(tmr_msgs::AskItemRequest &req, tmr_msgs::AskItemResponse &res)
{
  PubMsg &pm = pm_;
  tmr::TmSvrData &data = iface_.svr.data;
  bool rb = false;

  svr_mtx_.lock();
  svr_updated_ = false;
  svr_mtx_.unlock();

  rb = (iface_.svr.send_content(req.id, tmr::TmSvrData::Mode::READ_STRING, req.item) == iface_.RC_OK);

  {
    boost::unique_lock<boost::mutex> lck(svr_mtx_);
    if (rb && req.wait_time > 0.0) {
      if (!svr_updated_) {
        svr_cv_.wait_for(lck, boost::chrono::duration<double>(req.wait_time));
      }
      if (!svr_updated_) {
        rb = false;
      }
      res.id = pm.svr_msg.id;
      res.value = pm.svr_msg.content;
    }
    svr_updated_ = false;
  }
  res.ok = rb;
  return rb;
}

}