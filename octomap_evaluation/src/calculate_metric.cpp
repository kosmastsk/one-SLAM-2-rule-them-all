/* Implementation of the Metric calculation between two point clouds */

#include "octomap_evaluation/calculate_metric.h"

namespace calculate_metric
{
/******************************/
/*        Constructor         */
/******************************/

Metric::Metric()
{
  ROS_INFO("Metric empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Metric::Metric(char* argv[])
{
  _groundTruth = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  _slamMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (loadPointClouds(argv) == -1)
  {
    ROS_INFO("Could not load point clouds. Node shutting down...\n");
    ros::shutdown();
  }

  _mse = calculateMSE();
  ROS_INFO("MSE: %f \n", _mse);

  _q = calculateQ();
  ROS_INFO("Q: %f \n", _q);

  ros::shutdown();  // Job is done, node can shutdown now
}

/******************************/
/*        Destructor          */
/******************************/

Metric::~Metric()
{
  ROS_INFO("Class Metric has been destroyed\n");
}

/******************************/
/*      loadPointClouds       */
/******************************/

int Metric::loadPointClouds(char* argv[])
{
  std::string ground_truth_name = argv[1];
  std::string slam_map_name = argv[2];

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(ground_truth_name, *_groundTruth) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", ground_truth_name.c_str());
    return (-1);
  }
  ROS_INFO("Loaded %d data points from %s\n", _groundTruth->width * _groundTruth->height, ground_truth_name.c_str());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(slam_map_name, *_slamMap) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", slam_map_name.c_str());
    return (-1);
  }
  ROS_INFO("Loaded %d data points from %s\n", _slamMap->width * _slamMap->height, slam_map_name.c_str());

  return 1;
}

/******************************/
/*       calculateMSE         */
/******************************/

double Metric::calculateMSE()
{
  double result = 0;
  std::string distNorm = "Euclidean";

  ROS_INFO("Entered calculateMetric() \n");

  // Nearest Neighbor close point method
  for (int i = 0; i < _slamMap->width; i++)
  {
    double dist = bruteForceNearestNeighbor(_slamMap->points[i], distNorm);
    result += dist * dist;
  }
  result = result / _slamMap->width;

  return result;
}

/******************************/
/*         calculateQ         */
/******************************/

double Metric::calculateQ()
{
  ROS_INFO("Entered calculateQ() \n");
  double quality;
  float k = 3;  // can be changed

  // In order to create a normalized MSE we use an exponential, the width and height of the ground truth point cloud and
  // a constant
  quality =
      exp(-(k / (_groundTruth->width * _groundTruth->width + _groundTruth->height * _groundTruth->height)) * _mse);

  return quality;
}

/******************************/
/*  bruteForceNearestNeighbor */
/******************************/

double Metric::bruteForceNearestNeighbor(pcl::PointXYZRGB points, std::string distNorm)
{
  double minDst = calculateDistance(points, _groundTruth->points[0], distNorm);
  double dst;

  /* #pragma omp parallel for */  // Uncomment this for using OpenMP
  for (int i = 0; i < _groundTruth->width; i++)
  {
    dst = calculateDistance(points, _groundTruth->points[i], distNorm);
    if (dst < minDst)
      minDst = dst;
  }
  return minDst;
}

/******************************/
/*      calculateDistance     */
/******************************/

double Metric::calculateDistance(pcl::PointXYZRGB slamPoints, pcl::PointXYZRGB groundTruthPoints, std::string distNorm)
{
  double dst;
  double diff_x = slamPoints.x - groundTruthPoints.x;
  double diff_y = slamPoints.y - groundTruthPoints.y;
  double diff_z = slamPoints.z - groundTruthPoints.z;
  if (distNorm == "Euclidean")
  {
    dst = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  }

  return dst;
}

}  // namespace calculate_metric
