#include <gtest/gtest.h>
#include <ras_group8_localization/ParticleFilter.hpp>
#include <ras_group8_map/Grid.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

using namespace ras_group8_localization;

void fill_grid(nav_msgs::OccupancyGrid *grid)
{
  grid->info.width      = 10; /* [cell] */
  grid->info.height     = 10;
  grid->info.resolution = 0.1; /* [m/cell] */
  grid->data.resize(100);
  
  /* Draw on the grid */
  /* (1,0) | +  (1,1)
   * (0,0) +--> (0,1)
   */
  for (int i = 0; i < 10; i ++) {
    grid->data[i]        = 100;
    grid->data[i + 90]   = 100;
    grid->data[10*i]     = 100;
    grid->data[10*i + 9] = 100;
  }
   
  for (int i = 4; i < 8; i ++) {
    grid->data[10*i + 2] = 100;
  }
}

void simulate_scan(sensor_msgs::LaserScan *scan)
{
  /* Simulate a laser scan */
  static const double ranges[31] =
    {0.5897, 0.6436, 0.5336, 0.4714, 0.4369, 0.4216, 0.4223, 0.4392,
     0.2286, 0.1759, 0.1484, 0.1328, 0.1242, 0.1204, 0.1206, 0.1248,
     0.1339, 0.1503, 0.4982, 0.4714, 0.4369, 0.4216, 0.4223, 0.4392,
     0.4758, 0.5414, 0.6576, 0.5842, 0.5411, 0.5220, 0.5229};
    
  scan->angle_min = 0.0;
  scan->angle_max = 2*M_PI - M_PI/16;
  scan->angle_increment = M_PI/16;
  
  scan->range_min = 0.0;
  scan->range_max = 2.0;
  scan->ranges.assign(&ranges[0], &ranges[30]);
}

TEST(ParticleFilter, test_update)
{
  ParticleFilter pf(10, 0.1, 0.1, 0.1, 0.1, 0);
  pf.initialize(0.5, 0.5, 0);
  
  pf.inspect();
  
  pf.move(0.1, 0.1, 0.1);
  pf.move(0.1, 0.1, 0.1);
  pf.move(0.1, 0.1, 0.1);
  pf.move(0.1, 0.1, 0.1);
  pf.move(0.1, 0.1, 0.1);
  
  pf.inspect();
  
  Particle p = pf.averageParticle(0.0);
  printf("[%f, %f, %f, %f],\n", p.x, p.y, p.theta, p.w);
}

TEST(ParticleFilter, test_weigh)
{
  /* Setup an occupancy grid */
  nav_msgs::OccupancyGrid grid;
  fill_grid(&grid);
    
  sensor_msgs::LaserScan scan;
  simulate_scan(&scan);
    
  ParticleFilter pf(100, 0.1, 0.1, 0.1, 0.1, 0);
  /* Initialize uniformly on the grid */
  pf.initialize(grid);
  pf.inspect();
  ROS_INFO("-1");
  for (int i = 0; i < 100; i ++) {
    pf.weigh(grid, scan);
    pf.resample();
    ROS_INFO("%i", i);
  }
  
  pf.weigh(grid, scan);
  pf.inspect();
  
  ROS_INFO("100");
  
  for (int i = 0; i < 100; i ++) {
     printf("%i,", grid.data[i]);
   }
  
  /* Get the average position of the best particles */
  Particle ap = pf.averageParticle(0.05);
  printf("[%f, %f, %f, %f];\n", ap.x, ap.y, ap.theta, ap.w);
}

TEST(ParticleFilter, test_estimate_pose)
{
  /* Setup an occupancy grid */
  nav_msgs::OccupancyGrid grid;
  fill_grid(&grid);
  
  /* Setup a laser scan */
  sensor_msgs::LaserScan scan;
  simulate_scan(&scan);
    
  ParticleFilter pf(100, 0.0, 0.0, 0.1, 0.1, 0);
  /* Initialize far from [0.5 0.4 0.5] */
  pf.initialize(0.8, 0.8, 2*M_PI - 0.3);
  pf.weigh(grid, scan);
  
  geometry_msgs::PoseWithCovariance pose =
    pf.estimatePose(0.05);
  
  printf("[%f, %f];\n", pose.pose.position.x, pose.pose.position.y);
  //
  // for (int i = 0; i < 6; i++) {
  //   printf("[%f, %f, %f, %f, %f, %f];\n",
  //     pose.covariance[i*6 + 0],
  //     pose.covariance[i*6 + 1],
  //     pose.covariance[i*6 + 2],
  //     pose.covariance[i*6 + 3],
  //     pose.covariance[i*6 + 4],
  //     pose.covariance[i*6 + 5]);
  // }
  
  /* Allow the filter to converge some */
  for (int i = 0; i < 100; i ++) {
    const double l = pf.weigh(grid, scan);
    pf.resample();
    
    //printf("%f, ", l);
  }
  
  pf.weigh(grid, scan);
  //pf.inspect();
  
  pose = pf.estimatePose(0.05);
  
  printf("[%f, %f];\n", pose.pose.position.x, pose.pose.position.y);
  
  // for (int i = 0; i < 6; i++) {
  //   printf("[%f, %f, %f, %f, %f, %f];\n",
  //     pose.covariance[i*6 + 0],
  //     pose.covariance[i*6 + 1],
  //     pose.covariance[i*6 + 2],
  //     pose.covariance[i*6 + 3],
  //     pose.covariance[i*6 + 4],
  //     pose.covariance[i*6 + 5]);
  // }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}