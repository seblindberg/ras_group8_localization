#include <ras_group8_localization/Localization.hpp>
#include <ras_group8_map/Grid.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>

#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
#include <geometry_msgs/PoseArray.h>
#endif


namespace ras_group8_localization {
  
static double
  poseToHeading(const geometry_msgs::Pose& pose);

/* Localization
 *
 * The particle filter positions the laser frame within the map frame.
 */
Localization::Localization(ros::NodeHandle& node_handle,
                           const std::string& pose_topic,
                           const std::string& map_topic,
                           const std::string& odom_topic,
                           const std::string& laser_topic,
                           int num_particles,
                           double target_map_resolution)
    : node_handle_(node_handle),
      pf_initialized_(false),
      map_initialized_(false),
      pf_(num_particles, 0.1, 0.1, 0.1, 0.1),
      target_map_resolution_(target_map_resolution)
{
  pose_publisher_ =
    node_handle_.advertise<nav_msgs::Odometry>(pose_topic, 1, true);
    
  map_subscriber_ =
    node_handle_.subscribe(map_topic, 1,
                           &Localization::mapCallback, this);
    
  odom_subscriber_ =
    node_handle_.subscribe(odom_topic, 10,
                           &Localization::odometryCallback, this);
  
  laser_subscriber_ =
    node_handle_.subscribe(laser_topic, 10,
                           &Localization::laserCallback, this);
  
  /* Prepare the pose message */
  pose_msg_.header.frame_id = "map";
  pose_msg_.header.seq = 0;
                                            
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  state_particles_publisher_ =
    node_handle_.advertise<geometry_msgs::PoseArray>("particles", 1);
    
  particles_msg_.header.seq = 0;
  //particles_msg_.header.frame_id = "map";
  particles_msg_.poses.resize(num_particles);
#endif
  
  ROS_INFO("Successfully launched node.");
}

Localization::~Localization()
{
}

void
Localization::run(double update_rate)
{
  /* Attatch the update timer */
  update_timer_ =
    node_handle_.createTimer(ros::Duration(1.0 / update_rate),
                             &Localization::update, this);
}

/* Map Callback
 *
 */
void
Localization::mapCallback(const nav_msgs::OccupancyGrid& map)
{
  const int map_scaling_factor =
    floor(target_map_resolution_ / map.info.resolution);
  
  ROS_INFO("Setting the map");
  ROS_INFO("map_scaling_factor = %i", map_scaling_factor);
  
  /* Trying to ensure we get at least the target_map_resolution_ */
  if (map_scaling_factor > 1) {
    map_ = ras_group8_map::Grid::downsample(map, map_scaling_factor);
  } else {
    map_ = map;
  }
    
  /* Assume the same reference frame as the map */
  pose_msg_.header.frame_id = map.header.frame_id;
  particles_msg_.header.frame_id = map.header.frame_id;
  
  map_initialized_ = true;
}

void
Localization::odometryCallback(const nav_msgs::Odometry& odometry)
{
  if (!pf_initialized_) {
    /* TODO: Transform odometry to laser frame */
    const double x     = odometry.pose.pose.position.x;
    const double y     = odometry.pose.pose.position.y;
    const double theta = poseToHeading(odometry.pose.pose);
    
    /* Initialize the particles to the position given by the
       odometry */
    ROS_INFO("Initialize at (%f, %f, %f)", x, y, theta);
    pf_.initialize(x, y, theta);
    pf_initialized_ = true;
  } else {
    const double dt =
      (odometry.header.stamp - odometry_prev_.header.stamp).toSec();
    const double v = odometry.twist.twist.linear.x;
    const double w = odometry.twist.twist.angular.z;
    
    /* Only move particles when we are actually moving */
    if (v > 0 || w > 0) {
      /* Move the particles as suggested by the odometry */
      pf_.move(v, w, dt);
    }
  }
  
  odometry_prev_ = odometry;
}

void
Localization::laserCallback(const sensor_msgs::LaserScan& laser_scan)
{
  if (!pf_initialized_ || !map_initialized_) {
    return;
  }
  
  /* TODO: Move the laser scan to the odometry frame */
  pf_.weigh(map_, laser_scan);
  pf_.resample();
}

/* Update
 *
 * Notify the outside world of our updated pose in the odometry frame, relative
 * to the map frame.
 */
void
Localization::update(const ros::TimerEvent& timer_event)
{
  const ros::Time now = ros::Time::now();
  
  /* Estimate our pose using the best 90 % */
  geometry_msgs::PoseWithCovariance pose_cov = pf_.estimatePose(0.1);
  
  /* TODO: transform from the laser frame to the odometry frame */
  
  /* Copy the pose */
  pose_msg_.pose = pose_cov;
  
  pose_msg_.header.seq ++;
  pose_msg_.header.stamp = now;
  
  pose_publisher_.publish(pose_msg_);
  
  publishParticles();
}

void
Localization::publishParticles()
{
  pf_.particlePoses(&particles_msg_);
  
  particles_msg_.header.seq ++;
  particles_msg_.header.stamp = ros::Time::now();
  
  state_particles_publisher_.publish(particles_msg_);
}

Localization
Localization::load(ros::NodeHandle& n)
{
  const std::string pose_topic =
    n.param("pose_topic", std::string("pose"));
    
  const std::string map_topic =
    n.param("map_topic", std::string("map"));
    
  const std::string odom_topic =
    n.param("odom_topic", std::string("odom"));
    
  const std::string laser_topic =
    n.param("laser_topic", std::string("laser"));
  
  Localization localization(n, pose_topic,
                               map_topic,
                               odom_topic,
                               laser_topic,
                               100,
                               0.1);
  
  return localization;
}

double poseToHeading(const geometry_msgs::Pose& pose)
{
  double yaw;
  double tmp;

  /* Convert the orientation to a tf Quaternion */
  tf::Quaternion q(pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w);

  tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);

  return yaw;
}


} /* namespace */