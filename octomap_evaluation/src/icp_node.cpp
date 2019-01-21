/* This ROS node performs registration between two point clouds using
 * libpointmatcher
 */

#include "octomap_evaluation/icp.h"

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_node");

  if (argc != 3)
  {
    ROS_INFO("\n[USAGE] rosrun octomap_evaluation icp_node <reference_point_cloud.pcd> <reading_point_cloud.csv>\n");
    return 1;
  }
  else
  {
    icp::Icp icp(argv);
    ros::spin();
  }

  return 0;
}
