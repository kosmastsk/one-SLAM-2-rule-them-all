/* Implementation of the Octomap to Point cloud methods */

#include "octomap_evaluation/octomap_to_pc.h"

namespace octomap_to_pc
{
  /******************************/
  /*        Constructor         */
  /******************************/
  Converter::Converter()
  {
    ROS_INFO("Converter empty object created");
    _filename = "indoors.ot";
  }

  /******************************/
  /* Constructor with arguments */
  /******************************/
  Converter::Converter(int argc, char *argv[])
  {
    // Get the octomap filename from the user
    _filename = argv[1];
    // Create the full path. Consider we are running the node, when pwd = ~/catkin_ws
    _filename = "/home/nikos/catkin_ws/src/octomap_evaluation/maps/" + _filename;

    _octree = readOctomap(_filename);

    octomapToPointCloud(_octree);
}


  /******************************/
  /*        Destructor          */
  /******************************/
  Converter::~Converter()
  {
    ROS_INFO("Class converter has been destroyed\n");
  }

  /******************************/
  /*        readOctomap         */
  /******************************/
  octomap::ColorOcTree* Converter::readOctomap(std::string filename)
  {
    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
    octomap::ColorOcTree* octree = dynamic_cast<octomap::ColorOcTree*>(tree);

    return octree;
  }

  /******************************/
  /*     octomapToPointCloud    */
  /******************************/
  void Converter::octomapToPointCloud(octomap::ColorOcTree* octree)
  {
	  pcl::PointCloud<pcl::PointXYZ> cloud;
	  cloud.is_dense=false;
	  int i=0;
	  for(octomap::ColorOcTree::leaf_iterator it=octree->begin_leafs(), end=octree->end_leafs(); it!=end ; ++it)
	  {
		  cloud.points[i].x=it.getCoordinate().x();
		  cloud.points[i].y=it.getCoordinate().y();
		  cloud.points[i].z=it.getCoordinate().z();
		  i++;
	  }
	  pcl::io::savePCDFileASCII ("output.pcd",cloud);
	  std::cerr <<"Saved"<<cloud.points.size()<<"date points to output.pcd" <<std::endl;
	  return ;
  }

}
