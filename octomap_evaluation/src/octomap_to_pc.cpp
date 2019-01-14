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
    // Get the octomap filename from the user. Octomaps are saved in the octomap_evaluation/maps directory
    _filename = argv[1];
    // Create the full path. Consider we are running the node, when pwd = ~/catkin_ws
    _filename = "src/one-SLAM-2-rule-them-all/octomap_evaluation/maps/" + _filename;

    _octree = readOctomap(_filename);

    std::string outputFilename = "edit.bt";
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

}
