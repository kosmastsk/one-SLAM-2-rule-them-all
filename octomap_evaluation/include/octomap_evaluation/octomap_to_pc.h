/*   */

#ifndef OCTOMAP_TO_PC_HEADER
#define OCTOMAP_TO_PC_HEADER

// C++ headers
#include <iostream>
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

namespace octomap_to_pc {
class Converter {
private:
  // Variables
  ros::NodeHandle _nh;
  std::string _filename;
  octomap::ColorOcTree *_octree;
  pcl::PointCloud<pcl::PointXYZRGB> _cloud;

  // Methods
  octomap::ColorOcTree *readOctomap(std::string filename);
  pcl::PointCloud<pcl::PointXYZRGB>
  octomapToPointCloud(octomap::ColorOcTree *octree);

public:
  Converter();
  Converter(char *argv[]);
  ~Converter();
};
} // namespace octomap_to_pc

#endif
