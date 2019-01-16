/* This ROS node implements the conversion of an octomap to Point Cloud format
 */

#include "octomap_evaluation/octomap_to_pc.h"

/* Main function */
int main(int argc, char **argv) {
  ros::init(argc, argv, "octomap_to_pc");

  if (argc != 2) {
    ROS_INFO("\n[USAGE] rosrun octomap_evaluation octomap_to_pc_node "
             "<filename.ot>\n");
    return 1;
  } else {
    octomap_to_pc::Converter object(argv);
  }

  ros::spin();

  return 0;
}
