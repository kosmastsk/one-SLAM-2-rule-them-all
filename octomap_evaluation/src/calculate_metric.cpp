/* Implementation of the Metric calculation between two point clouds */

#include "octomap_evaluation/calculate_metric.h"

namespace calculate_metric
{
/******************************/
/*        Constructor         */
/******************************/

Metric::Metric()
{
  ROS_DEBUG("Metric empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Metric::Metric(char* argv[])
{
  _groundTruth = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  _slamMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (loadPointClouds(argv, _groundTruth, _slamMap) == -1)
  {
    ROS_INFO("Could not load point clouds. Node shutting down...\n");
    ros::shutdown();
  }

  _mse = calculateMSE(_groundTruth, _slamMap);
  ROS_INFO("MSE: %f \n", _mse);

  _q = calculateQ(_groundTruth, _mse);
  ROS_INFO("Q: %f \n", _q);

  ros::shutdown();  // Job is done, node can shutdown now
}

/******************************/
/*        Destructor          */
/******************************/

Metric::~Metric()
{
  ROS_DEBUG("Class Metric has been destroyed\n");
}

/******************************/
/*      loadPointClouds       */
/******************************/

int Metric::loadPointClouds(char* argv[], pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr slamMap)
{
  std::string ground_truth_name = argv[1];
  std::string slam_map_name = argv[2];

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(ground_truth_name, *groundTruth) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", ground_truth_name.c_str());
    return (-1);
  }
  ROS_INFO("Loaded %d data points from %s\n", groundTruth->width * groundTruth->height, ground_truth_name.c_str());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(slam_map_name, *slamMap) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", slam_map_name.c_str());
    return (-1);
  }
  ROS_INFO("Loaded %d data points from %s\n", slamMap->width * slamMap->height, slam_map_name.c_str());

  return 1;
}

/******************************/
/*       calculateMSE         */
/******************************/

double Metric::calculateMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr slamMap)
{
  double result = 0;
  std::string distNorm = "Euclidean";

  ROS_INFO("Entered calculateMetric() \n");

  // Nearest Neighbor close point method
  for (int i = 0; i < slamMap->width; i++)
  {
    double dist = bruteForceNearestNeighbor(slamMap->points[i], distNorm, groundTruth);
    result += dist * dist;
  }
  result = result / slamMap->width;

  return result;
}

/******************************/
/*         calculateQ         */
/******************************/

double Metric::calculateQ(pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth, double mse)
{
  ROS_INFO("Entered calculateQ() \n");
  double quality;
  float k = 3;  // can be changed

  // In order to create a normalized MSE we use an exponential, the width and height of the ground truth point cloud and
  // a constant
  quality = exp(-(k / (groundTruth->width * groundTruth->width + groundTruth->height * groundTruth->height)) * mse);

  return quality;
}

/******************************/
/*  bruteForceNearestNeighbor */
/******************************/

double Metric::bruteForceNearestNeighbor(pcl::PointXYZ points, std::string distNorm,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth)
{
  double minDst = calculateDistance(points, groundTruth->points[0], distNorm);
  double dst;

#pragma omp parallel for  // Uncomment this for using OpenMP
  for (int i = 0; i < groundTruth->width; i++)
  {
    dst = calculateDistance(points, groundTruth->points[i], distNorm);
    if (dst < minDst)
      minDst = dst;
  }
  return minDst;
}

/******************************/
/*      calculateDistance     */
/******************************/

double Metric::calculateDistance(pcl::PointXYZ slamPoints, pcl::PointXYZ groundTruthPoints, std::string distNorm)
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
