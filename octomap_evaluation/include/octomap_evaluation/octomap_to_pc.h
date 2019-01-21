/*   */

#ifndef OCTOMAP_TO_PC_HEADER
#define OCTOMAP_TO_PC_HEADER

// C++ headers
#include <iostream>
#include <stdio.h>
#include <string.h>

// ROS headers
#include <ros/package.h>
#include <ros/ros.h>

// Octomap headers
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

// Point Cloud headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace octomap_to_pc
{
class Converter
{
private:
  // Variables
  ros::NodeHandle _nh;
  std::string _filename;

  // Methods
  // Different ones depending if we provide Binary or Colored octomap
  pcl::PointCloud<pcl::PointXYZRGB> octomapToPointCloud(octomap::ColorOcTree* octree, std::string filename);

  pcl::PointCloud<pcl::PointXYZ> octomapToPointCloud(octomap::OcTree* octree, std::string filename);

  void savePointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filename);
  void savePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string filename);

public:
  Converter();
  Converter(char* argv[]);
  ~Converter();
};
}  // namespace octomap_to_pc

#endif
