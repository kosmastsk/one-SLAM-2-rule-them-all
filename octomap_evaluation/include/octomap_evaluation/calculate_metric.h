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

  pcl::PointCloud<pcl::PointXYZ>::Ptr _groundTruth;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _slamMap;

  double _mse;
  double _q;

  int loadPointClouds(char* argv[], pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr slamMap);
  double calculateMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth, pcl::PointCloud<pcl::PointXYZ>::Ptr slamMap);
  double calculateQ(pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth, double mse);

  double bruteForceNearestNeighbor(pcl::PointXYZ points, std::string distNorm,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth);

  double calculateDistance(pcl::PointXYZ slamPoints, pcl::PointXYZ groundTruthPoints, std::string distNorm);

public:
  Metric();
  Metric(char* argv[]);
  ~Metric();
};

}  // namespace calculate_metric

#endif
