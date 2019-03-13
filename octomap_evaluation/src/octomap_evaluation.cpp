#include "octomap_evaluation/octomap_to_pc.h"
#include "octomap_evaluation/icp.h"
#include "octomap_evaluation/calculate_metric.h"

// simple timing benchmark output
#define _BENCH_TIME 0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_evaluation_node");

  if (argc != 3)
  {
    ROS_INFO("\n[USAGE] rosrun octomap_evaluation octomap_evaluation <ground_truth_map.ot|bt> <slam_map.ot|bt>\n");
    return 1;
  }

#if defined(_BENCH_TIME)
  ros::WallTime startTime = ros::WallTime::now();
#endif

  std::string ground_truth_map = argv[1];
  std::string slam_map = argv[2];

// TODO all of the following - use already implemented functions
// Load octomaps as files

// Subsample them

// Convert them to Point Clouds

// Apply the ICP to the 2nd map

// Calculate the metric between the two Point Clouds

#if defined(_BENCH_TIME)
  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Octomap evaluation took " << dt << " seconds.");
#endif

  return 0;
}
