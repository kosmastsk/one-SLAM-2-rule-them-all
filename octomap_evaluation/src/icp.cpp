/* Implementation of the ICP registration between two point clouds */

#include "octomap_evaluation/icp.h"

namespace icp
{
/******************************/
/*        Constructor         */
/******************************/

Transformation::Transformation()
{
  ROS_INFO("Icp empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Transformation::Transformation(char* argv[])
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

  // Save files to see the results
  std::string path = ros::package::getPath("octomap_evaluation");
  path = path.c_str() + std::string("/maps/") + std::string("result_icp.pcd");
  PointMatcherIO<float>::savePCD(data_out, path);

  std::cout << "Final transformation:" << std::endl << T << std::endl;

  // To preview the outputs use the following command
  // pcl_viewer -multiview 1 indoors.ot.pcd indoors_v2.ot.pcd result_icp.pcd
}

/******************************/
/*        Destructor          */
/******************************/

Transformation::~Transformation()
{
  ROS_INFO("Class Icp has been destroyed\n");
}

}  // namespace icp
