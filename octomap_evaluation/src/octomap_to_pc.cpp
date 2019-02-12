/* Implementation of the Octomap to Point cloud methods */

#include "octomap_evaluation/octomap_to_pc.h"
#include <math.h>

namespace octomap_to_pc
{
/******************************/
/*        Constructor         */
/******************************/

Converter::Converter()
{
  ROS_INFO("Converter empty object created");
  //_filename = "indoors.ot";
}

/******************************/
/* Constructor with arguments */
/******************************/

Converter::Converter(char* argv[])
{
  // Get the octomap filename from the user
  _filename = argv[1];

  /*** Read Octomap ***/
  // Change function called depending on the suffix
  // Having an .ot suffix does not ensure that it contains a ColorOcTree. So we have to check the id inside
  if (_filename.substr(_filename.length() - 2) == "ot")
  {
    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(_filename);

    // Check if the id in the file really contains a ColorOcTree or is just an Octree
    if (tree->getTreeType() == std::string("OcTree"))
    {
      octomap::OcTree* _octree = dynamic_cast<octomap::OcTree*>(tree);
      if (!_octree)
      {
        ROS_ERROR("Could not read OcTree in file\n");
      }
      pcl::PointCloud<pcl::PointXYZ> _cloud = octomapToPointCloud(_octree, _filename);
    }
    else if (tree->getTreeType() == std::string("ColorOcTree"))
    {
      octomap::ColorOcTree* _octree = dynamic_cast<octomap::ColorOcTree*>(tree);
      if (!_octree)
      {
        ROS_ERROR("Could not read ColorOcTree in file, currently there are no other types supported in .ot");
      }
      pcl::PointCloud<pcl::PointXYZRGB> _cloud = octomapToPointCloud(_octree, argv[1]);
    }
  }
  /* Binary */
  else if (_filename.substr(_filename.length() - 2) == "bt")
  {
    octomap::OcTree* _octree = new octomap::OcTree(_filename);
    if (!_octree)
    {
      ROS_ERROR("Could not read OcTree in file\n");
    }
    pcl::PointCloud<pcl::PointXYZ> _cloud = octomapToPointCloud(_octree, _filename);
  }

  ros::shutdown();  // Job is done, node can shutdown now
}

/******************************/
/*        Destructor          */
/******************************/

Converter::~Converter()
{
  ROS_INFO("Class converter has been destroyed\n");
}

/******************************/
/*     octomapToPointCloud    */
/*         with colour        */
/******************************/

pcl::PointCloud<pcl::PointXYZRGB> Converter::octomapToPointCloud(octomap::ColorOcTree* givenOctree,
                                                                 std::string filename)
{
  ROS_INFO("Conversion to Point Cloud in progress ....\n");
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  size_t leafs = givenOctree->getNumLeafNodes();

  cloud.width = leafs;
  cloud.height = 1;
  cloud.is_dense = false;  // True if no points are invalid(NaN or Inf).
  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;
  for (octomap::ColorOcTree::leaf_iterator it = givenOctree->begin_leafs(), end = givenOctree->end_leafs(); it != end;
       ++it)
  {
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

  savePointCloud(cloud, filename);

  return cloud;
}

/******************************/
/*     octomapToPointCloud    */
/*       without colour       */
/******************************/

pcl::PointCloud<pcl::PointXYZ> Converter::octomapToPointCloud(octomap::OcTree* octree, std::string filename)
{
  ROS_INFO("Conversion to Point Cloud in progress ....\n");
  pcl::PointCloud<pcl::PointXYZ> cloud;

  size_t leafs = octree->getNumLeafNodes();

  cloud.width = leafs;
  cloud.height = 1;
  cloud.is_dense = false;  // True if no points are invalid(NaN or Inf).

  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;
  for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
  {
    // Provide the coordinate values from octomap
    cloud.points[i].x = it.getCoordinate().x();
    cloud.points[i].y = it.getCoordinate().y();
    cloud.points[i].z = it.getCoordinate().z();

    i++;
  }

  savePointCloud(cloud, filename);

  return cloud;
}

/******************************/
/*       savePointCloud       */
/*         with colour        */
/******************************/

void Converter::savePointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filename)
{
  // Edit the filename first, to remove path
  // https://stackoverflow.com/questions/8520560/get-a-file-name-from-a-path
  // Remove directory if present.
  // Do this before extension removal incase directory has a period character.
  const size_t last_slash_idx = filename.find_last_of("\\/");
  if (std::string::npos != last_slash_idx)
  {
    filename.erase(0, last_slash_idx + 1);
  }

  std::string path = ros::package::getPath("octomap_evaluation");
  path = path.c_str() + std::string("/maps/") + filename + std::string(".pcd");
  pcl::io::savePCDFileASCII(path, cloud);
  ROS_INFO("Saved %lu date points to %s\n", cloud.points.size(), path.c_str());
}

/******************************/
/*       savePointCloud       */
/*       without colour       */
/******************************/

void Converter::savePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string filename)
{
  // Edit the filename first, to remove path
  // https://stackoverflow.com/questions/8520560/get-a-file-name-from-a-path
  // Remove directory if present.
  // Do this before extension removal incase directory has a period character.
  const size_t last_slash_idx = filename.find_last_of("\\/");
  if (std::string::npos != last_slash_idx)
  {
    filename.erase(0, last_slash_idx + 1);
  }

  std::string path = ros::package::getPath("octomap_evaluation");
  path = path.c_str() + std::string("/maps/") + filename + std::string(".pcd");
  pcl::io::savePCDFileASCII(path, cloud);
  ROS_INFO("Saved %lu date points to %s\n", cloud.points.size(), path.c_str());
}

}  // namespace octomap_to_pc
