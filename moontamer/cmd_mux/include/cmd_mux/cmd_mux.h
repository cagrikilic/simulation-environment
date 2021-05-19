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

#ifndef CMD_MUX_H
#define CMD_MUX_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>

#include <list>

namespace cmd_mux
{

// Forwarding declarations:
class CmdMuxDiagnostics;
struct CmdMuxDiagnosticsStatus;
class VelocityTopicHandle;
class FourWheelSteeringTopicHandle;
class LockTopicHandle;

/**
 * @brief The CmdMux class implements a top-level twist multiplexer module
 * that priorize different velocity command topic inputs according to locks.
 */
class CmdMux
{
public:

  // use this type alias when the compiler supports this C++11 feat.
  template<typename T>
  using handle_container = std::list<T>;

  CmdMux(int window_size = 10);
  ~CmdMux();

  bool hasPriority(const VelocityTopicHandle& cmd);
  bool hasPriority(const FourWheelSteeringTopicHandle& cmd);

  void publishTwist(const geometry_msgs::TwistConstPtr& msg);
  void publishFourWheelSteering(const four_wheel_steering_msgs::FourWheelSteeringConstPtr& msg);

  void updateDiagnostics(const ros::TimerEvent& event);

protected:
  const std::string getHighestPriority(void);

  typedef CmdMuxDiagnostics       diagnostics_type;
  typedef CmdMuxDiagnosticsStatus status_type;

  ros::Timer diagnostics_timer_;

  static constexpr double DIAGNOSTICS_PERIOD = 1.0;

  /**
   * @brief velocity_hs_ Velocity topics' handles.
   * Note that if we use a vector, as a consequence of the re-allocation and
   * the fact that we have a subscriber inside with a pointer to 'this', we
   * must reserve the number of handles initially.
   */
  std::shared_ptr< handle_container<VelocityTopicHandle> > velocity_hs_;
  std::shared_ptr< handle_container<FourWheelSteeringTopicHandle> > four_wheel_steer_hs_;
  std::shared_ptr< handle_container<LockTopicHandle> > lock_hs_;

  ros::Publisher cmd_twist_pub_, cmd_4ws_pub_;

  geometry_msgs::Twist last_cmd_;

  template<typename T>
  void getTopicHandles(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, const std::string& param_name, std::list<T>& topic_hs);

  int getLockPriority();

  std::shared_ptr<diagnostics_type> diagnostics_;
  std::shared_ptr<status_type>      status_;
};

} // namespace cmd_mux

#endif // CMD_MUX_H
