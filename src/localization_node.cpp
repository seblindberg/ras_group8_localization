#include <ros/ros.h>
#include <ras_group8_localization/Localization.hpp>

using namespace ras_group8_localization;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_localization");
  ros::NodeHandle node_handle("~");

  Localization main_object = Localization::load(node_handle);
  
  main_object.run(5.0);
  ros::spin();
  
  return 0;
}