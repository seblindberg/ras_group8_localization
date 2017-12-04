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
                           const std::string& child_frame_id,
                           int num_particles,
                           double target_map_resolution,
                           double lidar_angle_offset)
    : node_handle_(node_handle),
      pf_initialized_(false),
      map_initialized_(false),
      pf_(num_particles, 0.01, 0.02, 0.02, 0.03, lidar_angle_offset),
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
  // pose_msg_.header.frame_id = frame_id; /* Default frame id */
  pose_msg_.header.seq = 0;
  
  /* Prepare the map transform */
  map_transform_.header.seq = 0;
  map_transform_.child_frame_id  = child_frame_id;
  //map_transform_.child_frame_id = child_frame_id;
  map_transform_.transform.translation.z = 0.0;
                                            
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  state_particles_publisher_ =
    node_handle_.advertise<geometry_msgs::PoseArray>("particles", 1);
    
  particles_msg_.header.seq = 0;
  particles_msg_.poses.resize(num_particles);
  
  map_publisher_ =
    node_handle_.advertise<nav_msgs::OccupancyGrid>("downscaled_map", 1, true);
#endif

  /* Check if an initial position is set */
  double x_0;
  double y_0;
  double theta_0;
  
  if (node_handle_.getParam("initial_position/x", x_0)
   && node_handle_.getParam("initial_position/y", y_0)
   && node_handle_.getParam("initial_position/theta", theta_0)) {
    ROS_INFO("Initializing particle filter at (%f, %f, %f)", x_0, y_0, theta_0);
    
    pf_.initialize(x_0, y_0, theta_0);
    pf_initialized_ = true;
  }
  
  
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
  
  /* DEBUG
     Allow the map to spawn a random distribution of particles */
  if (!pf_initialized_) {
    pf_.initialize(map);
    pf_initialized_ = true;
  }
  
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  map_publisher_.publish(map_);
#endif
    
  /* Assume the same reference frame as the map */
  pose_msg_.header.frame_id      = map.header.frame_id;
  map_transform_.header.frame_id = map.header.frame_id;
  //map_transform_.child_frame_id  = map.header.frame_id;
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  particles_msg_.header.frame_id = map.header.frame_id;
#endif

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
    pose_msg_.pose = odometry.pose;
  } else {
    const double dt =
      (odometry.header.stamp - odometry_prev_.header.stamp).toSec();
    const double v = odometry.twist.twist.linear.x;
    const double w = odometry.twist.twist.angular.z;
    
    /* Only move particles when we are actually moving */
    if (v > 0 || w > 0) {
      /* Move the particles as suggested by the odometry */
      pf_.move(v, w, dt);
      
      const double pose_theta = poseToHeading(pose_msg_.pose.pose);
      
      /* Also move the pose */
      const double dtheta = w * dt;
      
      const double dx = cos(pose_theta) * (v * dt);
      const double dy = sin(pose_theta) * (v * dt);
      
      pose_msg_.pose.pose.position.x += dx;
      pose_msg_.pose.pose.position.y += dy;
      
      pose_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(pose_theta + dtheta);
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
  
  ROS_INFO("Laser Update");
  
  // int max_reinit = 3;
  // for (;;) {
  //   if (pf_.weigh(map_, laser_scan) > 0) {
  //     break;
  //   }
  //
  //   if (--max_reinit == 0) {
  //     return; /* Give up */
  //     //break;
  //   } else if (max_reinit == 1) {
  //     //pf_.initialize(map_);
  //   } else {
  //     /* Add some system noise */
  //     ROS_INFO("Add noise");
  //     pf_.addSystemNoise();
  //   }
  // };
  //
  // /* Add a step that reinitializes the filter */
  //
  // pf_.resample();
  
  int resamples = 10;
  do {
    if (pf_.weigh(map_, laser_scan) == 0) {
      pf_.addSystemNoise();
      return;
    }
    
    pf_.resample();
  } while (--resamples);
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
  
  /* Estimate our pose */
  geometry_msgs::PoseWithCovariance pose_cov = pf_.estimatePose(0.5);
  
  /* TODO: transform from the laser frame to the odometry frame */
  
  /* Copy the pose
     Do a sanity check on the distance between the two first */
  {
    const double dx = pose_cov.pose.position.x - pose_msg_.pose.pose.position.x;
    const double dy = pose_cov.pose.position.y - pose_msg_.pose.pose.position.y;
    const double d  = sqrtf(dx*dx + dy*dy);
    
    if (d < 0.3) {
      pose_msg_.pose = pose_cov;
    }
  }
  
  pose_msg_.header.seq ++;
  pose_msg_.header.stamp = now;
  
  /* TODO: Find the source of this bug */
  if (pose_msg_.pose.pose.position.x > 10
   || pose_msg_.pose.pose.position.y > 10) {
    return;
  }
  
  pose_publisher_.publish(pose_msg_);
    
  /* Publish the frame */
  map_transform_.header.seq ++;
  map_transform_.header.stamp = now;
    
  map_transform_.transform.translation.x = pose_cov.pose.position.x;
  map_transform_.transform.translation.y = pose_cov.pose.position.y;
  map_transform_.transform.rotation      = pose_cov.pose.orientation;
  
  ROS_INFO("Publishing transform %s -> %s", map_transform_.header.frame_id.c_str(), map_transform_.child_frame_id.c_str());
  frame_broadcaster_.sendTransform(map_transform_);
  
#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
  publishParticles();
#endif
}

#if RAS_GROUP8_LOCALIZATION_PUBLISH_STATE
void
Localization::publishParticles()
{
  pf_.particlePoses(&particles_msg_);
  
  particles_msg_.header.seq ++;
  particles_msg_.header.stamp = ros::Time::now();
  
  state_particles_publisher_.publish(particles_msg_);
}
#endif

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
    
  const std::string child_frame_id =
    n.param("frame_id", std::string("base_link"));
    
  const int num_particles =
    n.param("num_particles", 100);
    
  const double target_map_resolution =
    n.param("target_map_resolution", 0.05);
    
  const double lidar_angle_offset =
    n.param("lidar_angle_offset", 0);
  
  Localization localization(n, pose_topic,
                               map_topic,
                               odom_topic,
                               laser_topic,
                               child_frame_id,
                               num_particles,
                               target_map_resolution,
                               lidar_angle_offset);
  
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