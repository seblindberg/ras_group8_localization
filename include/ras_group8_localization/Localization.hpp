#pragma once

#include <ros/ros.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_localization {

class Localization
{
public:
  Localization(ros::NodeHandle& node_handle);
  virtual ~Localization();

private:
  bool readParameters();
  void topicCallback(const phidgets::motor_encoder& msg);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber subscriber_;
  
  /* Parameters
   */
  std::string subscriber_topic_;
};

} /* namespace */