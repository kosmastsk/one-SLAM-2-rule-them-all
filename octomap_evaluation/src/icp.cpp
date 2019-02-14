/* Implementation of the ICP registration between two point clouds */

#include "octomap_evaluation/icp.h"

namespace icp
{
/******************************/
/*        Constructor         */
/******************************/

Icp::Icp()
{
  ROS_INFO("Icp empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Icp::Icp(char* argv[])
{
  // Get the octomap filename from the user
  _params = argv[1];
  _reference = argv[2];
  _reading = argv[3];

  // Load point clouds
  const PointMatcher<float>::DataPoints ref(PointMatcherIO<float>::loadPCD(_reference));
  const PointMatcher<float>::DataPoints data(PointMatcherIO<float>::loadPCD(_reading));

  // Load a data filters configuration for ICP using yaml file
  std::ifstream ifs(_params);
  if (!ifs.good())
  {
    std::cerr << "Cannot open config file " << argv[1] << std::endl;
  }

  // Construct the ICP algorithm from the YAML file
  _icp.PointMatcher<float>::ICPChainBase::loadFromYaml(ifs);

  // Compute the transformation to express data in ref
  PointMatcher<float>::TransformationParameters T = _icp(data, ref);

  // Transform data to express it in ref
  PointMatcher<float>::DataPoints data_out(data);
  _icp.transformations.apply(data_out, T);

  // Create a filename that consists of both filenames

  std::string path = ros::package::getPath("octomap_evaluation");
  path = path.c_str() + std::string("/maps/") + extractFilename(_reference) + std::string("_") +
         extractFilename(_reading) + std::string("_") + std::string("icp.pcd");

  // Save files to see the results
  PointMatcherIO<float>::savePCD(data_out, path);

  std::cout << "Final transformation:" << std::endl << T << std::endl;
  std::cout << "[OUTPUT FILE]: " << path << std::endl;

  // To preview the outputs use the following command
  // pcl_viewer -multiview 1 indoors.ot.pcd indoors_v2.ot.pcd result_icp.pcd
  //                             OR
  // pcl_viewer indoors.ot.pcd indoors_v2.ot.pcd result_icp.pcd

  ros::shutdown();  // Job is done, node can shutdown now
}

/******************************/
/*        Destructor          */
/******************************/

Icp::~Icp()
{
  ROS_INFO("Class Icp has been destroyed\n");
}

std::string Icp::extractFilename(std::string full_path)
{
  // https://stackoverflow.com/questions/8520560/get-a-file-name-from-a-path
  // Remove directory if present.
  // Do this before extension removal incase directory has a period character.
  const size_t last_slash_idx = full_path.find_last_of("\\/");
  if (std::string::npos != last_slash_idx)
  {
    full_path.erase(0, last_slash_idx + 1);
  }
  // Remove extension if present.
  const size_t period_idx = full_path.rfind('.');
  if (std::string::npos != period_idx)
  {
    full_path.erase(period_idx);
  }

  return full_path;
}

}  // namespace icp
