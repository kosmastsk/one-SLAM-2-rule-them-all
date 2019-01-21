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
  _reference = argv[1];
  _reading = argv[2];

  // Load point clouds
  const PointMatcher<float>::DataPoints ref(PointMatcherIO<float>::loadPCD(_reference));
  const PointMatcher<float>::DataPoints data(PointMatcherIO<float>::loadPCD(_reading));

  // See the implementation of setDefault() to create a custom ICP algorithm
  _icp.setDefault();

  // Compute the transformation to express data in ref
  PointMatcher<float>::TransformationParameters T = _icp(data, ref);

  // Transform data to express it in ref
  PointMatcher<float>::DataPoints data_out(data);
  _icp.transformations.apply(data_out, T);

  // Safe files to see the results
  PointMatcherIO<float>::savePCD(data_out, "icp_result.pcd");
  std::cout << "Final transformation:" << std::endl << T << std::endl;
}

/******************************/
/*        Destructor          */
/******************************/

Icp::~Icp()
{
  ROS_INFO("Class Icp has been destroyed\n");
}

}  // namespace icp
