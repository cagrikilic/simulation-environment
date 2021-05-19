/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#include <cmd_mux/cmd_mux.h>
#include <cmd_mux/topic_handle.h>
#include <cmd_mux/cmd_mux_diagnostics.h>
#include <cmd_mux/cmd_mux_diagnostics_status.h>
#include <cmd_mux/utils.h>
#include <cmd_mux/xmlrpc_helpers.h>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::Twist& old_twist, const geometry_msgs::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x  < new_linear_x ) or
         (old_angular_z < new_angular_z);
}

namespace cmd_mux
{

CmdMux::CmdMux(int window_size)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  /// Get topics and locks:
  velocity_hs_ = std::make_shared< handle_container<VelocityTopicHandle> >();
  four_wheel_steer_hs_ = std::make_shared< handle_container<FourWheelSteeringTopicHandle> >();
  lock_hs_     = std::make_shared< handle_container<LockTopicHandle> >();
  getTopicHandles(nh, nh_priv, "topics_vel", *velocity_hs_);
  getTopicHandles(nh, nh_priv, "topics_4ws", *four_wheel_steer_hs_);
  getTopicHandles(nh, nh_priv, "locks" , *lock_hs_ );

  /// Publisher for output topic:
  cmd_twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
  cmd_4ws_pub_ = nh.advertise<four_wheel_steering_msgs::FourWheelSteering>("cmd_4ws_out", 1);

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>();
  status_      = std::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs     = lock_hs_;

  diagnostics_timer_ = nh.createTimer(ros::Duration(DIAGNOSTICS_PERIOD), &CmdMux::updateDiagnostics, this);
}

CmdMux::~CmdMux()
{}

void CmdMux::updateDiagnostics(const ros::TimerEvent& event)
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void CmdMux::publishTwist(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_twist_pub_.publish(*msg);
}

void CmdMux::publishFourWheelSteering(const four_wheel_steering_msgs::FourWheelSteeringConstPtr& msg)
{
  cmd_4ws_pub_.publish(*msg);
}

template<typename T>
void CmdMux::getTopicHandles(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, const std::string& param_name, std::list<T>& topic_hs)
{
  try
  {
    xh::Array output;
    xh::fetchParam(nh_priv, param_name, output);

    xh::Struct output_i;
    std::string name, topic;
    double timeout;
    int priority;
    for (int i = 0; i < output.size(); ++i)
    {
      xh::getArrayItem(output, i, output_i);

      xh::getStructMember(output_i, "name"    , name    );
      xh::getStructMember(output_i, "topic"   , topic   );
      xh::getStructMember(output_i, "timeout" , timeout );
      xh::getStructMember(output_i, "priority", priority);

      topic_hs.emplace_back(nh, name, topic, timeout, priority, this);
    }
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    ROS_FATAL_STREAM("Error parsing params: " << e.what());
  }
}

int CmdMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h.isLocked())
    {
      auto tmp = lock_h.getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  ROS_DEBUG_STREAM("Priority = " << static_cast<int>(priority));

  return priority;
}

const std::string CmdMux::getHighestPriority(void)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string cmd_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h.isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        cmd_name = velocity_h.getName();
      }
    }
  }

  for (const auto& four_wheel_steer_h : *four_wheel_steer_hs_)
  {
    if (not four_wheel_steer_h.isMasked(lock_priority))
    {
      const auto four_wheel_steer_priority = four_wheel_steer_h.getPriority();
      if (priority < four_wheel_steer_priority)
      {
        priority = four_wheel_steer_priority;
        cmd_name = four_wheel_steer_h.getName();
      }
    }
  }
  return cmd_name;
}

bool CmdMux::hasPriority(const VelocityTopicHandle& cmd)
{
  const std::string result = getHighestPriority();
  return cmd.getName() == result;
}

bool CmdMux::hasPriority(const FourWheelSteeringTopicHandle& cmd)
{
  const std::string result = getHighestPriority();
  return cmd.getName() == result;
}

} // namespace cmd_mux
