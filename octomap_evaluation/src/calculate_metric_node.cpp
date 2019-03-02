/* This ROS node performs registration between two point clouds using
 * libpointmatcher
 */

#include "octomap_evaluation/calculate_metric.h"

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "calculate_metric");

  if (argc != 3)
  {
    ROS_INFO("\n[USAGE] rosrun octomap_evaluation calculate_metric <ground_truth_map.pcd> <slam_map.pcd>\n");
    return 1;
  }
  else
  {
    calculate_metric::Metric metric(argv);
    ros::spin();
  }

  return 0;
}
