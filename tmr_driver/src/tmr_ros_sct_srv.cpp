
#include "tmr_driver/tmr_ros_node.h"

// tmsct service part

namespace tmr_ros
{

bool TmrRosNode::send_script(tmr_msgs::SendScriptRequest &req, tmr_msgs::SendScriptResponse &res)
{
  bool rb = iface_.tmsct.send_script(req.id, req.script);
  res.ok = rb;
  return rb;
}
bool TmrRosNode::set_event(tmr_msgs::SetEventRequest &req, tmr_msgs::SetEventResponse &res)
{
  bool rb = false;
  switch (req.func) {
  case tmr_msgs::SetEventRequest::EXIT:
    rb = iface_.set_script_exit();
    break;
  case tmr_msgs::SetEventRequest::TAG:
    rb = iface_.set_tag((int)(req.arg0), (int)(req.arg1));
    break;
  case tmr_msgs::SetEventRequest::WAIT_TAG:
    rb = iface_.set_wait_tag((int)(req.arg0), (int)(req.arg1));
    break;
  case tmr_msgs::SetEventRequest::STOP:
    rb = iface_.set_stop();
    break;
  case tmr_msgs::SetEventRequest::PAUSE:
    rb = iface_.set_pause();
    break;
  case tmr_msgs::SetEventRequest::RESUME:
    rb = iface_.set_resume();
    break;
  }
  res.ok = rb;
  return rb;
}
bool TmrRosNode::set_io(tmr_msgs::SetIORequest &req, tmr_msgs::SetIOResponse &res)
{
  bool rb = iface_.set_io(tmrl::driver::IOModule(req.module), tmrl::driver::IOType(req.type), (int)(req.pin), req.state);
  res.ok = rb;
  return rb;
}
bool TmrRosNode::set_positions(tmr_msgs::SetPositionsRequest &req, tmr_msgs::SetPositionsResponse &res)
{
  bool rb = false;
  const size_t dof = tmrl::driver::RobotState::DOF;

  if (req.positions.size() != dof) {
    return rb;
  }
  switch (req.motion_type) {
  case tmr_msgs::SetPositionsRequest::PTP_J:
    rb = iface_.set_joint_pos_PTP(tmrl::to_arrayd<dof>(req.positions),
      (int)(req.velocity), req.acc_time, req.blend_percentage, req.fine_goal);
    break;
  case tmr_msgs::SetPositionsRequest::PTP_T:
    rb = iface_.set_tool_pose_PTP(tmrl::to_arrayd<dof>(req.positions),
      (int)(req.velocity), req.acc_time, req.blend_percentage, req.fine_goal);
    break;
  case tmr_msgs::SetPositionsRequest::LINE_T:
    rb = iface_.set_tool_pose_Line(tmrl::to_arrayd<dof>(req.positions),
      req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
    break;
  }
  res.ok = rb;
  return rb;
}
bool TmrRosNode::set_pvt(tmr_msgs::SetPvtPointRequest &req, tmr_msgs::SetPvtPointResponse &res)
{
  bool rb = false;
  tmrl::driver::PvtPoint point;
  point.positions = req.point.positions;
  point.velocities = req.point.velocities;
  point.time = req.point.time;

  switch (req.point.mode) {
  case tmr_msgs::PvtPoint::JOINT:
    rb = iface_.set_pvt_point(tmrl::driver::PvtMode::Joint, point);
    break;
  case tmr_msgs::PvtPoint::TOOL:
    rb = iface_.set_pvt_point(tmrl::driver::PvtMode::Tool, point);
    break;
  case tmr_msgs::PvtPoint::EXIT:
    rb = iface_.set_pvt_exit();
    break;
  case tmr_msgs::PvtPoint::ENTER_TOOL:
    rb = iface_.set_pvt_enter(tmrl::driver::PvtMode::Tool);
    break;
  case tmr_msgs::PvtPoint::ENTER_JOINT:
    rb = iface_.set_pvt_enter(tmrl::driver::PvtMode::Joint);
    break;
  }
  res.ok = rb;
  return rb;
}
bool TmrRosNode::set_trajectory(tmr_msgs::SetTrajectoryRequest &req, tmr_msgs::SetTrajectoryResponse &res)
{
  bool rb = false;
  tmrl::driver::PvtTraj traj;

  if (req.mode == tmr_msgs::SetTrajectoryRequest::JOINT) {
    traj.mode = tmrl::driver::PvtMode::Joint;
  }
  else {
    traj.mode = tmrl::driver::PvtMode::Tool;
  }
  double tt = 0.0;
  traj.points.resize(req.points.size());
  for (std::size_t i = 0; i < req.points.size(); ++i) {
    traj.points[i].positions = req.points[i].positions;
    traj.points[i].velocities = req.points[i].velocities;
    traj.points[i].time = req.points[i].time;
    tt += traj.points[i].time;
  }
  traj.total_time = tt;

  rb = iface_.set_pvt_traj(traj);
  res.ok = rb;
  return rb;
}

bool TmrRosNode::ask_sta(tmr_msgs::AskStaRequest &req, tmr_msgs::AskStaResponse &res)
{
  SctMsg &pm = sct_pm_;
  bool rb = false;

  std::unique_lock<std::mutex> lck(sta_mtx_);
  sta_updated_ = false;
  lck.unlock();

  rb = iface_.tmsct.send_sta_request(req.subcmd, req.subdata);

  lck.lock();
  if (rb && req.wait_time > 0.0) {
    if (!sta_updated_) {
      sta_cv_.wait_for(lck, std::chrono::duration<double>(req.wait_time));
    }
    if (!sta_updated_) {
      rb = false;
    }
    res.subcmd = pm.sta_msg.subcmd;
    res.subdata = pm.sta_msg.subdata;
  }
  sta_updated_ = false;

  res.ok = rb;
  return rb;
}

}