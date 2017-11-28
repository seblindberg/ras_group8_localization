#pragma once

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h> /* TODO: make optional */

#include <random>

namespace ras_group8_localization {

typedef struct {
  double x;
  double y;
  double theta;
  double w;
} Particle;

class ParticleFilter
{
public:
  ParticleFilter(int num_particle, double v_var, double w_var, double xy_var,
                 double theta_var, int seed = 0);

  virtual ~ParticleFilter();
  
  void
    initialize(nav_msgs::OccupancyGrid& grid);
    
  void
    initialize(double x, double y, double theta);
    
  void
    inspect();
  
  void
    move(double v, double w, double dt);
    
  double
    weigh(const nav_msgs::OccupancyGrid& map,
          const sensor_msgs::LaserScan &laser_scan);
  
  void
    resample();
    
  Particle
    averageParticle(const double p_threshold = 0.0);
    
  geometry_msgs::PoseWithCovariance
    estimatePose(const double p_threshold = 0.0);
    
  void
    particlePoses(geometry_msgs::PoseArray *pose_array);
  
private:
  
  void
    normalizeParticles();
  
  std::vector<Particle> particle_store_a_;
  std::vector<Particle> particle_store_b_;
  
  std::vector<Particle> *particles_;
  std::vector<Particle> *particles_resampled_;
  
  std::default_random_engine generator_;
  
  /* Random noise distributions */
  std::normal_distribution<double> distribution_v_;
  std::normal_distribution<double> distribution_w_;
  std::normal_distribution<double> distribution_xy_;
  std::normal_distribution<double> distribution_theta_;
};

} /* namespace */