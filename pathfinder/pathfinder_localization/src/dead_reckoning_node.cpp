/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WVU Interactive Robotics Laboratory nor
*     the names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#define MAX_ENCODER_COUNTS 2147483648
#define PI 3.14159265359

// Declare sensor data variables
int32_t fl_encoder_counts = 0;
int32_t fl_encoder_counts_prev = 0;
int32_t fl_encoder_feedback = 0;
int32_t fl_delta_counts = 0;
int32_t bl_encoder_counts = 0;
int32_t bl_encoder_counts_prev = 0;
int32_t bl_encoder_feedback = 0;
int32_t bl_delta_counts = 0;
int32_t fr_encoder_counts = 0;
int32_t fr_encoder_counts_prev = 0;
int32_t fr_encoder_feedback = 0;
int32_t fr_delta_counts = 0;
int32_t br_encoder_counts = 0;
int32_t br_encoder_counts_prev = 0;
int32_t br_encoder_feedback = 0;
int32_t br_delta_counts = 0;
double yaw_rate = 0.0;
double yaw_rate_prev = 0.0;
double delta_yaw = 0.0;
bool first_callback[5] = {true,true,true,true,true};
double imu_time;
double imu_time_prev;
double yaw = 0.0;
bool callbacks_run[5] = {false,false,false,false,false};

// Declare common functions
int32_t deltaCounts(int32_t counts, int32_t counts_prev);

// Declare sensor callback functions
void leftRoboteqCallback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg);
void rightRoboteqCallback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

int main(int argc, char** argv)
{
  // Declare variables that are read from params in 'pathfinder_localization/config/dead_reckoning.yaml'
  float loop_rate; // Hz
  double wheel_diameter; // m
  double encoder_counts_per_revolution;
  std::string left_roboteq_topic_name;
  std::string right_roboteq_topic_name;
  std::string imu_topic_name;
  std::string odometry_out_topic_name;
  std::string joint_state_out_topic_name;
  std::string odometry_frame_id;
  std::string odometry_child_frame_id;
  std::string joint_state_frame_id;

  // Initialize ROS
  std::string node_name = "pathfinder_dead_reckoning_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Read params from yaml file
  if(ros::param::get(node_name+"/loop_rate",loop_rate)==false)
  {
    ROS_FATAL("No parameter 'loop_rate' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/wheel_diameter",wheel_diameter)==false)
  {
    ROS_FATAL("No parameter 'wheel_diameter' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/encoder_counts_per_revolution",encoder_counts_per_revolution)==false)
  {
    ROS_FATAL("No parameter 'encoder_counts_per_revolution' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/left_roboteq_topic_name",left_roboteq_topic_name)==false)
  {
    ROS_FATAL("No parameter 'left_roboteq_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/right_roboteq_topic_name",right_roboteq_topic_name)==false)
  {
    ROS_FATAL("No parameter 'right_roboteq_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/imu_topic_name",imu_topic_name)==false)
  {
    ROS_FATAL("No parameter 'imu_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/odometry_out_topic_name",odometry_out_topic_name)==false)
  {
    ROS_FATAL("No parameter 'odometry_out_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/joint_state_out_topic_name",joint_state_out_topic_name)==false)
  {
    ROS_FATAL("No parameter 'joint_state_out_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/odometry_frame_id",odometry_frame_id)==false)
  {
    ROS_FATAL("No parameter 'odometry_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
  {
    ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name+"/joint_state_frame_id",joint_state_frame_id)==false)
  {
    ROS_FATAL("No parameter 'joint_state_frame_id' specified");
    ros::shutdown();
    exit(1);
  }

  // Initialize publishers and subscribers
  ros::Subscriber left_roboteq_sub = nh.subscribe(left_roboteq_topic_name, 1, leftRoboteqCallback); 
  ros::Subscriber right_roboteq_sub = nh.subscribe(right_roboteq_topic_name, 1, rightRoboteqCallback);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic_name, 1, imuCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odometry_out_topic_name, 1);
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_state_out_topic_name, 1);
  ros::Publisher joint_state_old_pub = nh.advertise<sensor_msgs::JointState>(joint_state_out_topic_name+"old", 1);

  // Initialize states
  nav_msgs::Odometry odom_msg; // Initializes to all zero, by default
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name.resize(4);
  joint_state_msg.name.at(0) = "front_left_wheel";
  joint_state_msg.name.at(1) = "front_right_wheel";
  joint_state_msg.name.at(2) = "back_left_wheel";
  joint_state_msg.name.at(3) = "back_right_wheel";
  joint_state_msg.position.resize(4,0.0);
  joint_state_msg.velocity.resize(4,0.0);
  joint_state_msg.effort.resize(4,0.0);
  sensor_msgs::JointState joint_state_old_msg;
  joint_state_old_msg.name.resize(4);
  joint_state_old_msg.name.at(0) = "front_left_wheel";
  joint_state_old_msg.name.at(1) = "front_right_wheel";
  joint_state_old_msg.name.at(2) = "back_left_wheel";
  joint_state_old_msg.name.at(3) = "back_right_wheel";
  joint_state_old_msg.position.resize(4,0.0);
  joint_state_old_msg.velocity.resize(4,0.0);
  joint_state_old_msg.effort.resize(4,0.0);

  // Loop and compute states by integrating yaw rate to get heading and averaging encoders to get distance
  ros::Rate rate(loop_rate);
  double time = ros::Time::now().toSec();
  double time_prev = ros::Time::now().toSec();
  double delta_time = 0.0;
  double first_loop = true;
  uint32_t seq = 0;
  while(ros::ok())
  {
    double delta_distance = 0.0;
    double delta_x = 0.0;
    double delta_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    time_prev = time;
    time = ros::Time::now().toSec();
    delta_time = time - time_prev;
    if(first_loop && !(callbacks_run[0] && callbacks_run[1] &&callbacks_run[2] && callbacks_run[3] && callbacks_run[4]))
    {
      delta_x = 0.0;
      delta_y = 0.0;
      velocity_x = 0.0;
      velocity_y = 0.0;
      first_loop = false;
      odom_msg.pose.pose.position.x = 0.0;
      odom_msg.pose.pose.position.y = 0.0;
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    }
    else
    {
      // Compute yaw and delta distances
      //ROS_INFO("delta_yaw = %f\ndelta_fl = %i\ndelta_bl = %i\ndelta_fr = %i\ndelta_br = %i",delta_yaw,fl_delta_counts,bl_delta_counts,fr_delta_counts,br_delta_counts);
      yaw += delta_yaw;
      delta_distance = ((double)(-fl_delta_counts - bl_delta_counts + fr_delta_counts + br_delta_counts))/4.0/encoder_counts_per_revolution*PI*wheel_diameter; // TEMPORARY: removed front left encoder because encoder does not work on pathfinder
      delta_x = delta_distance*cos(yaw);
      delta_y = delta_distance*sin(yaw);

      // DEBUGGING: print delta encoder counts
      //ROS_INFO("delta encoder [fl, fr, bl, br] = [%i, %i, %i, %i]",fl_delta_counts, fr_delta_counts, bl_delta_counts, br_delta_counts);
      //ROS_INFO("delta time = %f",delta_time);
      //if(fl_delta_counts==0)
      //  ROS_INFO("fl_delta_counts == 0");
      //if(fr_delta_counts==0)
      //  ROS_INFO("fr_delta_counts == 0");
      //if(bl_delta_counts==0)
      //  ROS_INFO("bl_delta_counts == 0");
      //if(br_delta_counts==0)
      //  ROS_INFO("br_delta_counts == 0");
     // DEBUGGING **********************

      // Compute joint states
      joint_state_msg.position.at(0) += fl_delta_counts/encoder_counts_per_revolution*2.0*PI;
      joint_state_msg.position.at(1) += fr_delta_counts/encoder_counts_per_revolution*2.0*PI;
      joint_state_msg.position.at(2) += bl_delta_counts/encoder_counts_per_revolution*2.0*PI;
      joint_state_msg.position.at(3) += br_delta_counts/encoder_counts_per_revolution*2.0*PI;

      //ROS_INFO("fl_encoder_feedback = %i",fl_encoder_feedback);
      //ROS_INFO("fr_encoder_feedback = %i",fr_encoder_feedback);
      //ROS_INFO("bl_encoder_feedback = %i",bl_encoder_feedback);
      //ROS_INFO("br_encoder_feedback = %i",br_encoder_feedback);
      joint_state_old_msg.velocity.at(0) = fl_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
      joint_state_msg.velocity.at(0) = ((double)fl_encoder_feedback)/1000.0*1000.0/60.0*2350.0/(1.0*500.0)*2.0*PI/65.5;
      joint_state_old_msg.velocity.at(1) = fr_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
      joint_state_msg.velocity.at(1) = ((double)fr_encoder_feedback)/1000.0*1000.0/60.0*2350.0/(1.0*500.0)*2.0*PI/65.5;
      joint_state_old_msg.velocity.at(2) = bl_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
      joint_state_msg.velocity.at(2) = ((double)bl_encoder_feedback)/1000.0*1000.0/60.0*2350.0/(1.0*500.0)*2.0*PI/65.5;
      joint_state_old_msg.velocity.at(3) = br_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
      joint_state_msg.velocity.at(3) = ((double)br_encoder_feedback)/1000.0*1000.0/60.0*2350.0/(1.0*500.0)*2.0*PI/65.5;
      //ROS_INFO("fl_ratio = %lf",joint_state_msg.velocity.at(0)/joint_state_old_msg.velocity.at(0));
      //ROS_INFO("fr_ratio = %lf",joint_state_msg.velocity.at(1)/joint_state_old_msg.velocity.at(1));
      //ROS_INFO("bl_ratio = %lf",joint_state_msg.velocity.at(2)/joint_state_old_msg.velocity.at(2));
      //ROS_INFO("br_ratio = %lf",joint_state_msg.velocity.at(3)/joint_state_old_msg.velocity.at(3));

//      joint_state_msg.velocity.at(0) = fl_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
//      joint_state_msg.velocity.at(1) = fr_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
//      joint_state_msg.velocity.at(2) = bl_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;
//      joint_state_msg.velocity.at(3) = br_delta_counts/encoder_counts_per_revolution*2.0*PI/delta_time;

      // Compute velocities
      double robot_body_vel = (-joint_state_msg.velocity.at(0) + joint_state_msg.velocity.at(1) - joint_state_msg.velocity.at(2) + joint_state_msg.velocity.at(3))*(wheel_diameter/2.0)/4.0;
      velocity_x = robot_body_vel*cos(yaw);
      velocity_y = robot_body_vel*sin(yaw);

      // Zero out deltas in case sensor callbacks do not occur before the next loop
      // Don't want to keep adding old data, only want the newest data
      fl_delta_counts = 0;
      bl_delta_counts = 0;
      fr_delta_counts = 0;
      br_delta_counts = 0;
      delta_yaw = 0.0;
    }
    // Pose
    // Add delta distances to odom message
    odom_msg.pose.pose.position.x += delta_x;
    odom_msg.pose.pose.position.y += delta_y;
    // Compute odom message quaternion from yaw angle
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    // Velocity (twist)
    odom_msg.twist.twist.linear.x = velocity_x;
    odom_msg.twist.twist.linear.y = velocity_y;
    odom_msg.twist.twist.angular.z = yaw_rate;
    // Header and frame info
    odom_msg.header.seq = seq;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = odometry_frame_id;
    odom_msg.child_frame_id = odometry_child_frame_id;
    odom_pub.publish(odom_msg);
    joint_state_msg.header.seq = seq;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = joint_state_frame_id;
    joint_state_pub.publish(joint_state_msg);
    joint_state_old_msg.header.seq = seq;
    joint_state_old_msg.header.stamp = ros::Time::now();
    joint_state_old_msg.header.frame_id = joint_state_frame_id;
    joint_state_old_pub.publish(joint_state_old_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

int32_t deltaCounts(int32_t counts, int32_t counts_prev)
{  
  int64_t temp_counts = (int64_t)counts;
  int64_t temp_counts_prev = (int64_t)counts_prev;
  int64_t temp_delta_counts = temp_counts - temp_counts_prev;
  if(temp_delta_counts > MAX_ENCODER_COUNTS) // Wrap around going backwards
  {
    temp_counts += 2*MAX_ENCODER_COUNTS;
    temp_delta_counts = temp_counts - temp_counts_prev;
  }
  else if(temp_delta_counts < -MAX_ENCODER_COUNTS) // Wrap around going forwards
  {
    temp_counts -= 2*MAX_ENCODER_COUNTS;
    temp_delta_counts = temp_counts - temp_counts_prev;
  }
  return (int32_t)temp_delta_counts;
}

void leftRoboteqCallback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg)
{
  // Front Left
  fl_encoder_counts_prev = fl_encoder_counts;
  fl_encoder_counts = msg->encoder_counter_absolute.at(2);
  fl_encoder_feedback = msg->feedback.at(0);
  if(first_callback[0])
  {
    //ROS_INFO("FL FIRST");
    fl_delta_counts = 0;
    first_callback[0] = false;
  }
  else
  {
    fl_delta_counts = deltaCounts(fl_encoder_counts, fl_encoder_counts_prev);
    callbacks_run[0] = true;
  }

  // Back Left
  bl_encoder_counts_prev = bl_encoder_counts;
  bl_encoder_counts = msg->encoder_counter_absolute.at(1);
  bl_encoder_feedback = msg->feedback.at(1);
  if(first_callback[1])
  {
    //ROS_INFO("BL FIRST");
    bl_delta_counts = 0;
    first_callback[1] = false;
  }
  else
  {
    bl_delta_counts = deltaCounts(bl_encoder_counts, bl_encoder_counts_prev);
    callbacks_run[1] = true;
  }
  //ROS_INFO("fl_counts = %i, fl_counts_prev = %i\nbl_counts = %i, bl_counts_prev = %i\n",fl_encoder_counts,fl_encoder_counts_prev,bl_encoder_counts,bl_encoder_counts_prev);
}

void rightRoboteqCallback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg)
{
  // Front Right
  fr_encoder_counts_prev = fr_encoder_counts;
  fr_encoder_counts = msg->encoder_counter_absolute.at(0);
  fr_encoder_feedback = msg->feedback.at(0);
  if(first_callback[2])
  {
    fr_delta_counts = 0;
    first_callback[2] = false;
  }
  else
  {
    fr_delta_counts = deltaCounts(fr_encoder_counts, fr_encoder_counts_prev);
    callbacks_run[2] = true;
  }

  // Back Right
  br_encoder_counts_prev = br_encoder_counts;
  br_encoder_counts = msg->encoder_counter_absolute.at(1);
  br_encoder_feedback = msg->feedback.at(1);
  if(first_callback[3])
  {
    br_delta_counts = 0;
    first_callback[3] = false;
  }
  else
  {
    br_delta_counts = deltaCounts(br_encoder_counts, br_encoder_counts_prev);
    callbacks_run[3] = true;
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  yaw_rate_prev = yaw_rate;
  yaw_rate = -msg->angular_velocity.z;
  imu_time_prev = imu_time;
  imu_time = ros::Time::now().toSec();
  if(first_callback[4])
  {
    delta_yaw = 0.0;
    first_callback[4] = false;
  }
  else
  {
    delta_yaw = ((yaw_rate + yaw_rate_prev)/2.0)*(imu_time - imu_time_prev);
    callbacks_run[4] = true;
  }
}
