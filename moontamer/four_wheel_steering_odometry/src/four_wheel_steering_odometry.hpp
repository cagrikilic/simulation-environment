/*
Ce noeud a pour but de récupérer les informations du topic Joint_States pour reconstruire les données odométriques du modèle bicyclette et de les publier dans un topic.
Ces données sont les braquages avant et arrière, les vitesses de braquage avant et arrière, la vitesse du véhicule, son accélération et son jerk.
Ce fichier .hpp sert à la création d'une classe propre à ce noeud : OdometryJointMessage.
*/

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <four_wheel_steering_msgs/FourWheelSteeringStamped.h>
#include <sensor_msgs/JointState.h>
#include <vector>

class OdometryJointMessage
{
public:
  OdometryJointMessage(ros::NodeHandle& nb);

  // Méthode d'un callback pour récupérer les données du topic mais qui aussi reconstruit les variables du modèle bicyclette
  void odomJointCallback (const sensor_msgs::JointState::ConstPtr& val);

  // Méthode pour récupérer les données géométriques du robot
  void setVehicleParam ();

  //Méthode qui remplit le message qui sera envoyé sur le topic
  void odomMessage (four_wheel_steering_msgs::FourWheelSteeringStamped &msg);

private:
  /*
   * get a param from the nearest vehicle_geomtry namespace
   * @in param_name : param to look for in namespace
   * @out param_value : value of the param
   * @return true if succeded false otherwise
   * */
  bool getVehicleGeometryParam(const std::string& param_name, double &param_value);

  // Param of the vehicle geometry
  double track_;
  double wheel_base_;
  double wheel_radius_;

  four_wheel_steering_msgs::FourWheelSteering fws_msg_;

  /// TODO use also FourWheelSteering class for previous values
  double speedprevious_;
//  double thetap_; // remain unused for the moment
  double accelerationprevious_;
  double previous_front_left_steering_angle_;
  double previous_front_right_steering_angle_;
  double previous_rear_left_steering_angle_;
  double previous_rear_right_steering_angle_;

  ros::Time previous;
  ros::NodeHandle nh_;

};
