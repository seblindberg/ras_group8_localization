#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ras_group8_localization/Localization.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <ras_group8_util/BMP.hpp>

using namespace ras_group8_localization;

void
  poseCallback(const nav_msgs::Odometry& msg);
  
static void
  load_test_grid(nav_msgs::OccupancyGrid *grid, const char *fname);
  
static volatile bool pose_is_valid;
static nav_msgs::Odometry pose;

void
poseCallback(const nav_msgs::Odometry& msg)
{
  pose = msg;
  pose_is_valid = true;
}

TEST(Localization, test_localiation)
{
  const double x0 = 0.5;
  const double y0 = 0.3;
  const double theta0 = 0.1;
  
  ros::NodeHandle node_handle;
  ros::Publisher  map_publisher   =
    node_handle.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Publisher  odom_publisher  =
    node_handle.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber pose_subscriber = node_handle.subscribe("/ras_group8_localization/pose", 1, &poseCallback);
  
  /* Setup and publish the map */
  nav_msgs::OccupancyGrid map;
  load_test_grid(&map, "/home/ras/catkin/maze_test.bmp");
  map.header.frame_id = "map";
  map_publisher.publish(map);
    
  /* Setup and publish the odometry */
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = x0;
  odom.pose.pose.position.y = y0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta0);
  odom_publisher.publish(odom);
  
  pose_is_valid = false;
  do {
    ros::spinOnce();
  } while (!pose_is_valid);
    
  EXPECT_FLOAT_EQ(x0, pose.pose.pose.position.x);
  EXPECT_FLOAT_EQ(y0, pose.pose.pose.position.y);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_localization");
  ros::NodeHandle nh;
  
  srand((int)time(0));
  return RUN_ALL_TESTS();
}

void
load_test_grid(nav_msgs::OccupancyGrid *grid, const char *fname)
{
  FILE* f = fopen(fname, "rb");
  int res;

  if (f == NULL) {
    ROS_ERROR("Failed to open target file");
    return;
  }

  if (res = ras_group8_util::BMP::read(grid, f)) {
    ROS_ERROR("Failed to read from file (%i)", res);
  }

  if (f != NULL) fclose(f);
}