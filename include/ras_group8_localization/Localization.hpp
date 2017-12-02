#pragma once

#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_group8_localization/ParticleFilter.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

namespace ras_group8_localization {
  
#define RAS_GROUP8_LOCALIZATION_PUBLISH_STATE 1

class Localization
{
public:
  Localization(ros::NodeHandle& node_handle,
               const std::string& pose_topic,
               const std::string& map_topic,
               const std::string& odom_topic,
               const std::string& laser_topic,
               const std::string& child_frame_id,
               int num_particles,
               double target_map_resolution);
  
  virtual ~Localization();
  
  void
    run(double update_rate);
  
  static Localization
    load(ros::NodeHandle& node_handle);

private:
  void
    mapCallback(const nav_msgs::OccupancyGrid& map);
  
  void
    odometryCallback(const nav_msgs::Odometry& odometry);

  void
    laserCallback(const sensor_msgs::LaserScan& laser_scan);
    
  void
    update(const ros::TimerEvent& timer_event);
    
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  void
    publishParticles();
#endif

  /* ROS Objects
   */
  ros::NodeHandle&   node_handle_;
  ros::Publisher     pose_publisher_;
  
  ros::Subscriber    map_subscriber_;
  ros::Subscriber    odom_subscriber_;
  ros::Subscriber    laser_subscriber_;
  
  /* Map TF Transform
   */
  tf::TransformBroadcaster frame_broadcaster_;
  geometry_msgs::TransformStamped map_transform_;
  
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  ros::Publisher     state_particles_publisher_;
  ros::Publisher     map_publisher_;
  geometry_msgs::PoseArray particles_msg_;
#endif
  
  nav_msgs::Odometry odometry_prev_;
  
  /* Store a copy of the world map (may be down sampled) */
  nav_msgs::OccupancyGrid map_;
  
  nav_msgs::Odometry pose_msg_;
  
  ParticleFilter     pf_;
  bool               pf_initialized_;
  bool               map_initialized_;
  
  const double       target_map_resolution_;
  
  ros::Timer         update_timer_;
};

} /* namespace */