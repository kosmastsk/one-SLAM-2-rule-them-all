/*   */

#ifndef OCTOMAP_TO_PC_HEADER
#define OCTOMAP_TO_PC_HEADER

// C++ headers
#include <stdio.h>
#include <string.h>

// ROS headers
#include <ros/ros.h>

// Octomap headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

// Point Cloud headers


namespace octomap_to_pc
{
class Converter
{
private:
  // Variables
  ros::NodeHandle _nh;
  std::string _filename;
  octomap::ColorOcTree* _octree;

  // Methods
  octomap::ColorOcTree* readOctomap(std::string filename);
  void octomapToPointCloud(octomap::ColorOcTree* octree);

public:
  Converter();
  Converter(int argc, char *argv[]);
  ~Converter();

};
}


#endif
