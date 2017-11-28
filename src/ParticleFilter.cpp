#include <ras_group8_localization/ParticleFilter.hpp>
#include <math.h>
#include <ros/ros.h>
#include <ras_group8_map/Grid.hpp>
#include <tf/transform_datatypes.h>

namespace ras_group8_localization {
  
static inline void
  moveParticle(Particle *particle, const double v, const double w,
               const double dt);

static inline bool
  particleIsBlocked(Particle& particle);

static inline double
  gridLikelihood(const nav_msgs::OccupancyGrid &grid, double x, double y);
  
static inline double
  gridLikelihood(const nav_msgs::OccupancyGrid &grid, int x, int y);
  
 
ParticleFilter::ParticleFilter(int num_particle, double v_var, double w_var,
                               double xy_var, double theta_var, int seed)
  : generator_(seed),
    distribution_v_(0.0, v_var),
    distribution_w_(0.0, w_var),
    distribution_xy_(0.0, xy_var),
    distribution_theta_(0.0, theta_var),
    particles_(&particle_store_a_),
    particles_resampled_(&particle_store_b_)
{
  particle_store_a_.resize(num_particle);
  particle_store_b_.resize(num_particle);
}

ParticleFilter::~ParticleFilter()
{
}

void
ParticleFilter::initialize(nav_msgs::OccupancyGrid &grid)
{
  const int num_particles = particles_->size();
  
  const double x_max = grid.info.width * grid.info.resolution;
  const double y_max = grid.info.height * grid.info.resolution;
  
  const double rand_x_factor     = x_max / RAND_MAX;
  const double rand_y_factor     = y_max / RAND_MAX;
  const double rand_theta_factor = (2*M_PI) / RAND_MAX;
  const double w_uniform = (double) RAND_MAX / num_particles;
  
  /* TODO: Do not place particles on walls */
  for (int i = 0; i < num_particles; i ++) {
    Particle *p = &(*particles_)[i];
    p->x     = (double) rand() * rand_x_factor;
    p->y     = (double) rand() * rand_y_factor;
    p->theta = (double) rand() * rand_theta_factor;
    p->w     = w_uniform;
  }
}

void
ParticleFilter::initialize(double x, double y, double theta)
{
  const int num_particles = particles_->size();
  const double w_uniform = (double) RAND_MAX / num_particles;
  
  for (int i = 0; i < num_particles; i ++) {
    Particle *p = &(*particles_)[i];
    
    /* Generate some system noise */
    const double x_noise     = distribution_xy_(generator_);
    const double y_noise     = distribution_xy_(generator_);
    const double theta_noise = distribution_theta_(generator_);
    
    p->x     = x + x_noise;
    p->y     = y + y_noise;
    p->theta = theta + theta_noise;
    p->w     = w_uniform;
  }
}

void
ParticleFilter::inspect()
{
  const int num_particles = particles_->size();
  
  printf("[\n");
  for (int i = 0; i < num_particles; i ++) {
    const Particle *p = &(*particles_)[i];
    printf("[%f, %f, %f, %f];\n", p->x, p->y, p->theta, p->w);
  }
  printf("]\n");
}

/* Move
 *
 * Uses additive gaussian noise as system model noise.
 */
void
ParticleFilter::move(double v, double w, double dt)
{
  const int num_particles = particles_->size();
  /* Move each particle */
  for (int i = 0; i < num_particles; i ++) {
    Particle *p = &(*particles_)[i];
    /* Add noise to the linear and angular velocity */
    const double v_noise = distribution_v_(generator_);
    const double w_noise = distribution_w_(generator_);
    
    moveParticle(p, v + v_noise, w + w_noise, dt);
  }
}

/* Weigh
 *
 * Take each particle, place it in the map and sum over the range measurements.
 */
double
ParticleFilter::weigh(const nav_msgs::OccupancyGrid& map,
                      const sensor_msgs::LaserScan &laser_scan)
{
  const int num_particles = particles_->size();
  const int num_ranges   = laser_scan.ranges.size();
  const double range_min = laser_scan.range_min;
  const double range_max = laser_scan.range_max;
  const double angle_increment = laser_scan.angle_increment;
  double total_likelihood = 0.0;
  
  for (int i = 0; i < num_particles; i ++) {
    Particle *p = &(*particles_)[i];
    double range_angle = p->theta + laser_scan.angle_min;
    double l = 0.0; /* Particle likelihood */
    
    /* Check if we are in an occupied cell */
    if (gridLikelihood(map, p->x, p->y) > 50) {
      // ROS_INFO("(%f, %f): Inside wall", p->x, p->y);
    } else {
    
      /* Check the laser data */
      for (int j = 0; j < num_ranges; j ++) {
        const double r = laser_scan.ranges[j];
        /* Calculate the transform of each of the laser scan */
        const double x = p->x + r * cos(range_angle);
        const double y = p->y + r * sin(range_angle);
        
        /* Fetch the probability value from the map */
        l += gridLikelihood(map, x, y);
        
        range_angle += angle_increment;
      }
    }
    
    /* Assign the particle likelihood as the weight */
    p->w = l;
    total_likelihood += l;
  }
  
  /* Normalize particles */
  const double w_factor = RAND_MAX / total_likelihood;
  for (int i = 0; i < num_particles; i ++) {
    (*particles_)[i].w *= w_factor;
  }
  
  return total_likelihood;
}

/* Normalize Particles
 *
 * Rescales the weights of all the particles to sum to RAND_MAX.
 */
void
ParticleFilter::normalizeParticles()
{
  const int num_particles = particles_->size();
  double w_sum = 0.0;
  for (int i = 0; i < num_particles; i ++) {
    w_sum += (*particles_)[i].w;
  }
  
  const double w_factor = RAND_MAX / w_sum;
  for (int i = 0; i < num_particles; i ++) {
    (*particles_)[i].w *= w_factor;
  }
}

/* Resample
 *
 * Perform systematic resampling of the particles based on their weights.
 */
void
ParticleFilter::resample()
{
  const int num_particles = particles_->size();
  const double r0 = (double) rand() / num_particles;
  const double m_scaling_factor = (double) RAND_MAX / num_particles;
  const double w_uniform = (double) RAND_MAX / num_particles;
  
  /* Cumulative sum over the particle wheights */
  double cumulative_weight = (*particles_)[0].w;
  /* Particle to resample from */
  int particle_i  = 0;
  
  for (int m = 0; m < num_particles; m ++) {
    Particle *p_dest = &(*particles_resampled_)[m];
    
    /* Find the next particle to sample from */
    while (cumulative_weight < m_scaling_factor * m + r0) {
      ++particle_i;
      cumulative_weight += (*particles_)[particle_i].w;
    }
    
    Particle *p_src = &(*particles_)[particle_i];
    
    /* Get the relative importance of the selected particle.
       unimportance < 1 means p_src is above average. Use
       this to scale the applied noise */
    double unimportance  = w_uniform / p_src->w;
    
    /* Cap the noise scaling factor at 2 */
    if (unimportance > 2) {
      unimportance = 2;
    }
    
    /* Resample particle m from particle_i
       Add system noise */
    const double x_noise     = distribution_xy_(generator_)    * unimportance;
    const double y_noise     = distribution_xy_(generator_)    * unimportance;
    const double theta_noise = distribution_theta_(generator_) * unimportance;
    
    p_dest->x     = p_src->x + x_noise;
    p_dest->y     = p_src->y + y_noise;
    p_dest->theta = p_src->theta + theta_noise;
    p_dest->w     = w_uniform;
    
    // ROS_INFO("Resample %u -> %u", particle_i, m);
  }
  
  std::swap(particles_resampled_, particles_);
}

/* Average Particle
 *
 * The particles must be normalized prior to calling this method.
 */
Particle
ParticleFilter::averageParticle(const double p_threshold)
{
  const int num_particles  = particles_->size();
  const double w_threshold = p_threshold * ((double) RAND_MAX / num_particles);
  int num_particles_avg    = num_particles;

  Particle average;
  double average_theta_x = 0.0;
  double average_theta_y = 0.0;

  double scale_factor = 0.0;

  average.x     = 0.0;
  average.y     = 0.0;
  average.theta = 0;
  average.w     = 0.0;

  for (int i = 0; i < num_particles; i ++) {
    const Particle *p = &(*particles_)[i];
    if (p->w < w_threshold) {
      --num_particles_avg;
      continue;
    }

    average.x     += p->x * p->w;
    average.y     += p->y * p->w;
    average.w     += p->w;

    average_theta_x += cos(p->theta) * p->w;
    average_theta_y += sin(p->theta) * p->w;

    scale_factor += p->w;
  }

  if (num_particles_avg > 0) {
    const double scale_factor_w = 1.0 / num_particles_avg;
    scale_factor = 1.0 / scale_factor;

    average.x     *= scale_factor;
    average.y     *= scale_factor;
    average.w     *= scale_factor_w;

    average_theta_x *= scale_factor;
    average_theta_y *= scale_factor;

    average.theta = atan2(average_theta_y, average_theta_x);
  }

  return average;
}

void
ParticleFilter::particlePoses(geometry_msgs::PoseArray *pose_array)
{
  const int num_particles  = particles_->size();
  
  if (pose_array->poses.size() != num_particles) {
    pose_array->poses.resize(num_particles);
  }
  
  for (int i = 0; i < num_particles; i ++) {
    const Particle *p = &(*particles_)[i];
    geometry_msgs::Pose *pose = &pose_array->poses[i];
    
    pose->position.x  = p->x;
    pose->position.y  = p->y;
    pose->position.z  = 0;
    pose->orientation = tf::createQuaternionMsgFromYaw(p->theta);
  }
}

geometry_msgs::PoseWithCovariance
ParticleFilter::estimatePose(const double p_threshold)
{
  geometry_msgs::PoseWithCovariance pose;
  const Particle mu     = averageParticle(p_threshold);
  
  /* Fill in the pose */
  pose.pose.position.x  = mu.x;
  pose.pose.position.y  = mu.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(mu.theta);
  
  const int num_particles  = particles_->size();
  const double w_threshold = p_threshold * ((double) RAND_MAX / num_particles);
  
  double var_xx = 0.0;
  double var_xy = 0.0;
  double var_yy = 0.0;
  double var_xtheta = 0.0;
  double var_ytheta = 0.0;
  double var_thetatheta = 0.0;
  double scale_factor = 0.0;
  
  for (int i = 0; i < num_particles; i ++) {
    const Particle *p = &(*particles_)[i];
    if (p->w < w_threshold) {
      continue;
    }
    
    const double dx = p->x - mu.x;
    const double dy = p->y - mu.y;

    const double dtheta = fmod(p->theta - mu.theta, 2*M_PI);

    const double dy_w     = dy     * p->w;
    const double dtheta_w = dtheta * p->w;

    var_xx         += dx     * dx * p->w;
    var_xy         += dx     * dy_w;
    var_yy         += dy     * dy_w;
    var_xtheta     += dx     * dtheta_w;
    var_ytheta     += dy     * dtheta_w;
    var_thetatheta += dtheta * dtheta_w;
    
    scale_factor   += p->w;
  }
  
  if (scale_factor > 0) {
    scale_factor = 1.0 / scale_factor;
    
    var_xx         *= scale_factor;
    var_xy         *= scale_factor;
    var_yy         *= scale_factor;
    var_xtheta     *= scale_factor;
    var_ytheta     *= scale_factor;
    var_thetatheta *= scale_factor;
  }
  
  /* Row 0 (x) */
  pose.covariance[0] = var_xx;
  pose.covariance[1] = var_xy;
  pose.covariance[2] = 0.0; /* var_zx */
  pose.covariance[3] = 0.0;
  pose.covariance[4] = 0.0;
  pose.covariance[5] = var_xtheta;
  
  /* Row 1 (y) */
  pose.covariance[6]  = var_xy;
  pose.covariance[7]  = var_yy;
  pose.covariance[8]  = 0.0;
  pose.covariance[9]  = 0.0;
  pose.covariance[10] = 0.0;
  pose.covariance[11] = var_ytheta;
  
  /* Row 2 (z) */
  pose.covariance[12] = 0.0;
  pose.covariance[13] = 0.0;
  pose.covariance[14] = 0.0;
  pose.covariance[15] = 0.0;
  pose.covariance[16] = 0.0;
  pose.covariance[17] = 0.0;
  
  /* Row 3 (roll) */
  pose.covariance[18] = 0.0;
  pose.covariance[19] = 0.0;
  pose.covariance[20] = 0.0;
  pose.covariance[21] = 0.0;
  pose.covariance[22] = 0.0;
  pose.covariance[23] = 0.0;
  
  /* Row 4 (pitch) */
  pose.covariance[24] = 0.0;
  pose.covariance[25] = 0.0;
  pose.covariance[26] = 0.0;
  pose.covariance[27] = 0.0;
  pose.covariance[28] = 0.0;
  pose.covariance[29] = 0.0;
  
  /* Row 5 (yaw) */
  pose.covariance[30] = var_xtheta;
  pose.covariance[31] = var_ytheta;
  pose.covariance[32] = 0.0;
  pose.covariance[33] = 0.0;
  pose.covariance[34] = 0.0;
  pose.covariance[35] = var_thetatheta;
  
  return pose;
}

/* Move Particle
 *
 * Implements the particle motion model
 */
void
moveParticle(Particle *particle, const double v, const double w,
             const double dt)
{
  const double dtheta = w * dt;
  const double dx = cos(particle->theta) * (v * dt);
  const double dy = sin(particle->theta) * (v * dt);
  
  particle->x     += dx;
  particle->y     += dy;
  particle->theta += dtheta;
}

/* Particle is Blocked
 *
 */
bool
particleIsBlocked(Particle& particle)
{
  return false;
}

double
gridLikelihood(const nav_msgs::OccupancyGrid& grid, int c, int r)
{
  if (c < 0 || c >= grid.info.width) {
    return 0;
  }
  
  if (r < 0 || r >= grid.info.height) {
    return 0;
  }
  
  return grid.data[r * grid.info.width + c];
}

double
gridLikelihood(const nav_msgs::OccupancyGrid& grid, double x, double y)
{
  /* Translate the x and y coordinates into row and column */
  const double r_real = y / grid.info.resolution - 0.5;
  const double c_real = x / grid.info.resolution - 0.5;
  
  /* If we floor the real part we get the cell to the left, while the ceil of gives the one to the right. The fractional part gives the ratio between them. */
  
  /*     +---+---+
   * r_a |   |   | above
   *     +---+---+
   * r_b |   |   | below
   *     +---+---+
   *      c_l c_r
   */
  
  /* Check the four adjacent cells */
  const int c_l = floor(c_real);
  const int c_r = ceil(c_real);
  const int r_b = floor(r_real);
  const int r_a = ceil(r_real);
  
  const double c_frac = c_real - c_l;
  const double r_frac = r_real - r_b;
  
  const double l = (1 - c_frac) * (1 - r_frac) * gridLikelihood(grid, c_l, r_b)
                 + (    c_frac) * (1 - r_frac) * gridLikelihood(grid, c_r, r_b)
                 + (1 - c_frac) * (    r_frac) * gridLikelihood(grid, c_l, r_a)
                 + (    c_frac) * (    r_frac) * gridLikelihood(grid, c_r, r_a);
                 
  return l;
}
  
}