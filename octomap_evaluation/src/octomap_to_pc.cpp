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
  //std::string path = ros::package::getPath("octomap_evaluation");
  //_filename = path.c_str() + std::string("/maps/") + argv[1];
  //std::cout << argv[0] << '\n';
  //std::cout << argv[1] << '\n';
	std::string _filename=argv[1];
	/*** Read Octomap ***/
	if(_filename.substr( _filename.length()-2) == "ot")
		{
		  octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(_filename);
		  octomap::ColorOcTree *_octree = dynamic_cast<octomap::ColorOcTree *>(tree);
		  _cloud = octomapToPointCloud(_octree, argv[1]);
		}else if (_filename.substr( _filename.length()-2) == "bt")
		{
			using namespace octomap;
			octomap::OcTree *_octree = new OcTree(_filename);
			_cloud = octomapToPointCloud(_octree, argv[1]);
		}

}

/******************************/
/*        Destructor          */
/******************************/
Converter::~Converter() { ROS_INFO("Class converter has been destroyed\n"); }


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

  //std::string path = ros::package::getPath("octomap_evaluation");
  //path = path.c_str() + std::string("/maps/") + filename + std::string(".pcd");
  pcl::io::savePCDFileASCII("output.pcd", cloud);
  //std::cerr << "Saved " << cloud.points.size() << " date points to " << path
   //         << std::endl;

  return cloud;
}
/*************************/
/*octomaptopointcloud non color*/
pcl::PointCloud<pcl::PointXYZRGB>
Converter::octomapToPointCloud(octomap::OcTree *octree, char *filename) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  size_t leafs = octree->getNumLeafNodes();

  cloud.width = leafs;
  cloud.height = 1;
  cloud.is_dense = false; // True if no points are invalid(NaN or Inf).

  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;
  for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
                                           end = octree->end_leafs();
       it != end; ++it) {

    // Provide the coordinate values from octomap
    cloud.points[i].x = it.getCoordinate().x();
    cloud.points[i].y = it.getCoordinate().y();
    cloud.points[i].z = it.getCoordinate().z();
    // Color
    cloud.points[i].r = 0 ;
    cloud.points[i].g = 132 ;
    cloud.points[i].b = 132 ;
    i++;
  }
  ROS_INFO("%d\n", i);

  //std::string path = ros::package::getPath("octomap_evaluation");
  //path = path.c_str() + std::string("/maps/") + filename + std::string(".pcd");

  pcl::io::savePCDFileASCII("output.pcd", cloud);
  //std::cerr << "Saved " << cloud.points.size() << " date points to " << path
   //         << std::endl;

  return cloud;
}

} // namespace octomap_to_pc
