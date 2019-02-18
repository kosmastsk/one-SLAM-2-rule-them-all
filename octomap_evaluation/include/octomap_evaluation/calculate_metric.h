/*   */

#ifndef CALCULATE_METRIC_HEADER
#define CALCULATE_METRIC_HEADER

// C++ headers
#include <iostream>
#include <cmath>

// ROS headers
#include <ros/package.h>
#include <ros/ros.h>

// Point Cloud library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace calculate_metric
{
class Metric
{
protected:
  // Variables
  ros::NodeHandle _nh;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _groundTruth;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _slamMap;

  double _metric;

  int loadPointClouds(char* argv[]);
  double calculateMetric();

  double bruteForceNearestNeighbor(pcl::PointXYZRGB points, std::string distNorm);

  double calculateDistance(pcl::PointXYZRGB slamPoints, pcl::PointXYZRGB groundTruthPoints, std::string distNorm);

public:
  Metric();
  Metric(char* argv[]);
  ~Metric();
};

}  // namespace calculate_metric

#endif
