#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>

namespace gazebo
{
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense()
{
}

GazeboRosRealsense::~GazeboRosRealsense()
{
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  std::string camera_prefix;
  std::string node_name;
  if(_sdf->HasElement("cameraPrefix"))
  {
    camera_prefix = _sdf->GetElement("cameraPrefix")->Get<std::string>();
    node_name = camera_prefix + "/realsense";
    printf("has cameraPrefix, %s\n",camera_prefix.c_str());
  }
  else
  {
    camera_prefix = "";
    node_name = "realsense";
    printf("does not have cameraPrefix\n");
  }
  depth_camera_name_ = camera_prefix + DEPTH_CAMERA_NAME;
  color_camera_name_ = camera_prefix + COLOR_CAMERA_NAME;
  ired1_camera_name_ = camera_prefix + IRED1_CAMERA_NAME;
  ired2_camera_name_ = camera_prefix + IRED2_CAMERA_NAME;
  camera_frame_id_ = camera_prefix + "_realsense";

  RealSensePlugin::Load(_model, _sdf);
  printf("after RealSensePlugin::Load\n");

  this->rosnode_ = new ros::NodeHandle(node_name);

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(*this->rosnode_, node_name));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  this->color_pub_ = this->itnode_->advertiseCamera("camera/color/image_raw", 2);
  this->ir1_pub_ = this->itnode_->advertiseCamera("camera/ir/image_raw", 2);
  this->ir2_pub_ = this->itnode_->advertiseCamera("camera/ir2/image_raw", 2);
  this->depth_pub_ = this->itnode_->advertiseCamera("camera/depth/image_raw", 2);
}

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub)
{
  common::Time current_time = this->world->GetSimTime();

  // identify camera
  std::string camera_id = extractCameraName(cam->Name());
  const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
    {color_camera_name_, &(this->color_pub_)},
    {ired1_camera_name_, &(this->ir1_pub_)},
    {ired2_camera_name_, &(this->ir2_pub_)},
  };
  const auto image_pub = camera_publishers.at(camera_id);

  // copy data into image
  this->image_msg_.header.frame_id = camera_frame_id_;
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  const std::map<std::string, std::string> supported_image_encodings = {
    {"L_INT8", sensor_msgs::image_encodings::MONO8},
    {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
  };
  const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_,
    pixel_format,
    cam->ImageHeight(), cam->ImageWidth(),
    cam->ImageDepth() * cam->ImageWidth(),
    reinterpret_cast<const void*>(cam->ImageData()));

  // identify camera rendering
  const std::map<std::string, rendering::CameraPtr> cameras = {
    {color_camera_name_, this->colorCam},
    {ired1_camera_name_, this->ired1Cam},
    {ired2_camera_name_, this->ired2Cam},
  };

  // publish to ROS
  auto camera_info_msg = cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
  image_pub->publish(this->image_msg_, camera_info_msg);
}

void GazeboRosRealsense::OnNewDepthFrame()
{
  // get current time
  common::Time current_time = this->world->GetSimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id = camera_frame_id_;
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_,
    pixel_format,
    this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
    2 * this->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->depthMap.data()));

  // publish to ROS
  auto depth_info_msg = cameraInfo(this->depth_msg_, this->depthCam->HFOV().Radian());
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
}

std::string GazeboRosRealsense::extractCameraName(const std::string& name)
{
    if (name.find(color_camera_name_) != std::string::npos) return color_camera_name_;
    if (name.find(ired1_camera_name_) != std::string::npos) return ired1_camera_name_;
    if (name.find(ired2_camera_name_) != std::string::npos) return ired2_camera_name_;

    ROS_ERROR("Unknown camera name");
    return color_camera_name_;
}

sensor_msgs::CameraInfo GazeboRosRealsense::cameraInfo(const sensor_msgs::Image& image, float horizontal_fov)
{
    sensor_msgs::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

    info_msg.K[0] = focal;
    info_msg.K[4] = focal;
    info_msg.K[2] = info_msg.width * 0.5;
    info_msg.K[5] = info_msg.height * 0.5;
    info_msg.K[8] = 1.;

    info_msg.P[0] = info_msg.K[0];
    info_msg.P[5] = info_msg.K[4];
    info_msg.P[2] = info_msg.K[2];
    info_msg.P[6] = info_msg.K[5];
    info_msg.P[10] = info_msg.K[8];

    return info_msg;
}

}
