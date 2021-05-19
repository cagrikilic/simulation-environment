/*
Ce noeud a pour but de récupérer les informations du topic Joint_States pour reconstruire les données odométriques du modèle bicyclette et de les publier dans un topic.
Ces données sont les braquages avant et arrière, les vitesses de braquage avant et arrière, la vitesse du véhicule, son accélération et son jerk.
*/

#include <ros/ros.h>
#include "four_wheel_steering_odometry.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_wheel_steering_odometry_node");
  ros::NodeHandle nb;

  OdometryJointMessage ojm(nb);
  ojm.setVehicleParam();

  ros::Subscriber sub = nb.subscribe("joint_states", 1, &OdometryJointMessage::odomJointCallback, &ojm);
  ros::Publisher odom_msgs = nb.advertise<four_wheel_steering_msgs::FourWheelSteeringStamped>("odom_steer",1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    four_wheel_steering_msgs::FourWheelSteeringStamped msg;
    ojm.odomMessage(msg);
    odom_msgs.publish(msg);

    loop_rate.sleep();
    ros::spinOnce();
  }

}
