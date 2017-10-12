#include <ros/ros.h>
#include <ras_group8_localization/Localization.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_localization");
  ros::NodeHandle node_handle("~");

  ras_group8_localization::Localization main_object(node_handle);

  ros::spin();
  return 0;
}