/*
Ce noeud a pour but de récupérer les informations du topic Joint_States pour reconstruire les données odométriques du modèle bicyclette et de les publier dans un topic.
Ces données sont les braquages avant et arrière, les vitesses de braquage avant et arrière, la vitesse du véhicule, son accélération et son jerk.
Ce fichier .cpp permet de décrire les méthodes utilisées dans le noeud.
*/

#include "four_wheel_steering_odometry.hpp"

#include <exception>

OdometryJointMessage::OdometryJointMessage(ros::NodeHandle &nb):
  nh_(nb),
  track_(0.0), wheel_base_(0.0), wheel_radius_(0.0),
  speedprevious_(0.0), accelerationprevious_(0.0),
  previous_front_left_steering_angle_(0.0), previous_front_right_steering_angle_(0.0),
  previous_rear_left_steering_angle_(0.0), previous_rear_right_steering_angle_(0.0)

{
}

void OdometryJointMessage::odomJointCallback (const sensor_msgs::JointState::ConstPtr& val)
{
  /// Création des variables qui seront utilisées pour calculer nos données
  double front_left_steering_angle=val->position[0];
  double front_right_steering_angle=val->position[2];
  double rear_left_steering_angle=val->position[4];
  double rear_right_steering_angle=val->position[6];

  double front_left_wheel_velocity=val->velocity[1];
  double front_right_wheel_velocity=val->velocity[3];
  double rear_left_wheel_velocity=val->velocity[5];
  double rear_right_wheel_velocity=val->velocity[7];

  /// création de variables temporelles pour les dérivées qui seront calculer
  ros::Duration ros_diff_time = ros::Time::now() - previous;
  double diff_time = ros_diff_time.toSec();

  /// Calcul du front et rear steering angle
  fws_msg_.front_steering_angle=2/((1/tan(front_left_steering_angle))+(1/tan(front_right_steering_angle)));
  fws_msg_.rear_steering_angle=2/((1/tan(rear_left_steering_angle))+(1/tan(rear_right_steering_angle)));

  /// Calcul de la vitesse, de l'accélération et du jerk

  // Formule exacte mais utilisant thetap qui n'est pas implémenté pour l'instant
//  speed_=sqrt(pow((front_left_wheel_velocity_*wheel_radius_),2)-pow((wheel_base_*thetap_/2),2))+thetap_*track_/2;

  fws_msg_.speed=((front_left_wheel_velocity+front_right_wheel_velocity+rear_left_wheel_velocity+rear_right_wheel_velocity)/4)*wheel_radius_;
  fws_msg_.acceleration=(fws_msg_.speed-speedprevious_)/diff_time;
  fws_msg_.jerk=(fws_msg_.acceleration-accelerationprevious_)/diff_time;

  //mise en mémoire des valeurs
  speedprevious_=fws_msg_.speed;
  accelerationprevious_=fws_msg_.acceleration;
  previous = ros::Time::now();

  /// Calcul du front et rear steering velocity

  // Méthode avec les informations du topic joint_states // Ne marche pas pour l'instant
//  front_steering_angle_velocity_=sqrt(pow(thetap_*wheel_base_/2,2)+pow(speed_,2))/wheel_radius_; // 2 formules au cas où 2 rayons de roues différents (mais variables à changer)
//  rear_steering_angle_velocity_=sqrt(pow(thetap_*wheel_base_/2,2)+pow(speed_,2))/wheel_radius_;

  // Méthode avec le taux de variation des angles de braquages
  double front_left_steering_velocity= (front_left_steering_angle-previous_front_left_steering_angle_)/diff_time;
  double front_right_steering_velocity= (front_right_steering_angle-previous_front_right_steering_angle_)/diff_time;
  double rear_left_steering_velocity= (rear_left_steering_angle-previous_rear_left_steering_angle_)/diff_time;
  double rear_right_steering_velocity= (rear_right_steering_angle-previous_rear_right_steering_angle_)/diff_time;

  fws_msg_.front_steering_angle_velocity=(front_left_steering_velocity+front_right_steering_velocity)/2;
  fws_msg_.rear_steering_angle_velocity=(rear_left_steering_velocity+rear_right_steering_velocity)/2;

  previous_front_left_steering_angle_=front_left_steering_angle;
  previous_front_right_steering_angle_=front_right_steering_angle;
  previous_rear_left_steering_angle_=rear_left_steering_angle;
  previous_rear_right_steering_angle_=rear_right_steering_angle;

   // Pour tester les valeurs seuls dans le terminal avec ROS
//  double valeur_test=front_left_steering_velocity;
//  ROS_INFO_STREAM ("valeur test"<<valeur_test); // pour faire des tests sur les valeurs.
}

bool OdometryJointMessage::getVehicleGeometryParam(const std::string& param_name, double& param_value)
{
  bool success = true;
  std::string param_path = "vehicle_controller/"+param_name;
  if(param_value < 0.0001)
  {
    if(success &= nh_.hasParam(param_path))
    {
      success &= nh_.getParam(param_path, param_value);
      ROS_INFO_STREAM(param_name<<" "<<param_value);
    }
  }
  if(success == false)
    throw std::runtime_error("Failed to get param "+param_name+" ; Will retry every second");
  return success;
}

void OdometryJointMessage::setVehicleParam()
{
  bool success = false;
  ros::Rate r(1);
  while (ros::ok() && success == false)
  {
    success = true;
    try
    {
      getVehicleGeometryParam("track", track_);
      getVehicleGeometryParam("wheel_base", wheel_base_);
      getVehicleGeometryParam("wheel_radius", wheel_radius_);
    }
    catch(std::runtime_error e)
    {
      ROS_WARN_STREAM(e.what());
      success = false;
      r.sleep();
    }
  }
}

void OdometryJointMessage::odomMessage (four_wheel_steering_msgs::FourWheelSteeringStamped &msg)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/base_link";
  msg.data = fws_msg_;
}

