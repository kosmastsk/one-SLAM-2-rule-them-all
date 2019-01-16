/* Implementation of the Octomap to Point cloud methods */

#include "octomap_evaluation/octomap_to_pc.h"
#include <math.h>

namespace octomap_to_pc {
/******************************/
/*        Constructor         */
/******************************/
Converter::Converter() {
  ROS_INFO("Converter empty object created");
  _filename = "indoors.ot";
}

/******************************/
/* Constructor with arguments */
/******************************/
Converter::Converter(char *argv[]) {
  // Get the octomap filename from the user
  std::string path = ros::package::getPath("octomap_evaluation");
  _filename = path.c_str() + std::string("/maps/") + argv[1];

  _octree = readOctomap(_filename);

  _cloud = octomapToPointCloud(_octree, argv[1]);
}

/******************************/
/*        Destructor          */
/******************************/
Converter::~Converter() { ROS_INFO("Class converter has been destroyed\n"); }

/******************************/
/*        readOctomap         */
/******************************/
octomap::ColorOcTree *Converter::readOctomap(std::string filename) {
  octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(filename);
  octomap::ColorOcTree *octree = dynamic_cast<octomap::ColorOcTree *>(tree);

  return octree;
}

/******************************/
/*     octomapToPointCloud    */
/******************************/
pcl::PointCloud<pcl::PointXYZRGB>
Converter::octomapToPointCloud(octomap::ColorOcTree *octree, char *filename) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  size_t leafs = octree->getNumLeafNodes();

  cloud.width = leafs;
  cloud.height = 1;
  cloud.is_dense = false; // True if no points are invalid(NaN or Inf).

  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;
  for (octomap::ColorOcTree::leaf_iterator it = octree->begin_leafs(),
                                           end = octree->end_leafs();
       it != end; ++it) {

    // Provide the coordinate values from octomap
    cloud.points[i].x = it.getCoordinate().x();
    cloud.points[i].y = it.getCoordinate().y();
    cloud.points[i].z = it.getCoordinate().z();

    // Color
    cloud.points[i].r = it->getColor().r;
    cloud.points[i].g = it->getColor().g;
    cloud.points[i].b = it->getColor().b;

    i++;
  }
  ROS_INFO("%d\n", i);

  std::string path = ros::package::getPath("octomap_evaluation");
  path = path.c_str() + std::string("/maps/") + filename + std::string(".pcd");

  pcl::io::savePCDFileASCII(path, cloud);
  std::cerr << "Saved " << cloud.points.size() << " date points to " << path
            << std::endl;

  return cloud;
}

} // namespace octomap_to_pc
