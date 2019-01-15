/* Implementation of the Octomap to Point cloud methods */

#include "octomap_evaluation/octomap_to_pc.h"

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
Converter::Converter(int argc, char *argv[]) {
  // Get the octomap filename from the user
  std::string path = ros::package::getPath("octomap_evaluation");
  _filename = path.c_str() + std::string("/maps/") + argv[1];

  _octree = readOctomap(_filename);

  _pointCloud = octomapToPointCloud(_octree);
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
Converter::octomapToPointCloud(octomap::ColorOcTree *octree) {
  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

  // pointCloud.width = 5;
  // pointCloud.height = 1;
  // pointCloud.is_dense = false;
  // pointCloud.points.resize(pointCloud.width * pointCloud.height);
  ROS_INFO("size: %d", pointCloud.points.size());
  int i = 0;
  for (octomap::ColorOcTree::leaf_iterator it = octree->begin_leafs(),
                                           end = octree->end_leafs();
       it != end; ++it) {
    pointCloud.points[i].x = 0.95; // it.getCoordinate().x();
    /*pointCloud.points[i].y = 0;    // it.getCoordinate().y();
    pointCloud.points[i].z = 0;    // it.getCoordinate().z();
    pointCloud.points[i].r = 0;
    pointCloud.points[i].g = 0;
    pointCloud.points[i].b = 0;
    */ ++i;
  }

  ROS_INFO("size: %d", pointCloud.points.size());

  pcl::io::savePCDFileASCII("output.pcd", pointCloud);
  std::cerr << "Saved" << pointCloud.points.size()
            << "date points to output.pcd" << std::endl;
  for (size_t i = 0; i < pointCloud.points.size(); ++i)
    std::cerr << "    " << pointCloud.points[i].x << " "
              << pointCloud.points[i].y << " " << pointCloud.points[i].z
              << std::endl;

  return pointCloud;
}

} // namespace octomap_to_pc
