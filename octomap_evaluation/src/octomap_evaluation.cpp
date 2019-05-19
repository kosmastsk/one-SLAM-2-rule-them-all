#include "octomap_evaluation/octomap_to_pc.h"
#include "octomap_evaluation/icp.h"
#include "octomap_evaluation/calculate_metric.h"

// (Un)comment the following lines, depending your needs

// simple timing benchmark output
#define _BENCH_TIME

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_evaluation_node");
  ros::NodeHandle nh;

  // Create objects from the classes that containc functionality
  octomap_to_pc::Converter conv;

  // FIXME  error: expected type-specified

  if (argc != 4)
  {
    ROS_INFO("\n[USAGE] rosrun octomap_evaluation octomap_evaluation <ground_truth_map.ot|bt> <slam_map.ot|bt> "
             "<icp_config_file.yaml>\n");
    return 1;
  }

#if defined(_BENCH_TIME)
  ros::WallTime startTime = ros::WallTime::now();
#endif

  std::string ground_truth_map = argv[1];
  std::string slam_map = argv[2];
  std::string params = argv[3];

  // TODO all of the following - use already implemented functions
  // Load octomaps as files

  // If the octomap contains color, or is not binary convert it to binary.
  // We do now need color, to evaluate them
  if (ground_truth_map.substr(ground_truth_map.length() - 2) == "ot" || slam_map.substr(slam_map.length() - 2) == "ot")
  {
    ROS_WARN("Please convert your files to .bt with convert_octree and try again\n");
    ROS_WARN("$ convert_octree <input_filename> <output_filename>\n");
  }

  // GROUND TRUTH MAP
  ROS_DEBUG("Handling the ground truth map\n");
  octomap::OcTree* gnd_octree = new octomap::OcTree(ground_truth_map);
  if (!gnd_octree)
  {
    ROS_ERROR("Could not read OcTree in file %s\n", ground_truth_map.c_str());
  }
  // Convert them to Point Clouds
  pcl::PointCloud<pcl::PointXYZ> gnd_cloud = conv.octomapToPointCloud(gnd_octree);

  // Create pointer pointing to cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr gnd_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(gnd_cloud));

  // Subsample them
  gnd_cloud = conv.subsampleCloud(gnd_cloudPtr);

  // Save PointCloud file
  conv.savePointCloud(gnd_cloud, ground_truth_map);

  // SLAM MAP
  ROS_DEBUG("Handling the SLAM provided map\n");
  octomap::OcTree* slam_octree = new octomap::OcTree(slam_map);
  if (!slam_octree)
  {
    ROS_ERROR("Could not read OcTree in file %s\n", slam_map.c_str());
  }
  // Convert them to Point Clouds
  pcl::PointCloud<pcl::PointXYZ> slam_cloud = conv.octomapToPointCloud(slam_octree);

  // Create pointer pointing to cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr slam_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(slam_cloud));

  // Subsample them
  slam_cloud = conv.subsampleCloud(slam_cloudPtr);

  // Save PointCloud file
  conv.savePointCloud(slam_cloud, slam_map);

  // After the conversion the paths are the same, with .pcd in the end
  ground_truth_map += std::string(".pcd");
  slam_map += std::string(".pcd");

  // Apply the ICP to the 2nd map
  // Load point clouds as DataPoints
  const PointMatcher<float>::DataPoints ref(PointMatcherIO<float>::loadPCD(ground_truth_map));
  const PointMatcher<float>::DataPoints data(PointMatcherIO<float>::loadPCD(slam_map));

  // Load a data filters configuration for ICP using yaml file
  std::ifstream ifs(params);
  if (!ifs.good())
  {
    std::cerr << "Cannot open config file " << argv[1] << std::endl;
  }

  // Construct the ICP algorithm from the YAML file
  PointMatcher<float>::ICP icp;
  icp.PointMatcher<float>::ICPChainBase::loadFromYaml(ifs);

  // Compute the transformation to express data in ref
  PointMatcher<float>::TransformationParameters T = icp(data, ref);

  // Transform data to express it in ref
  PointMatcher<float>::DataPoints data_out(data);
  icp.transformations.apply(data_out, T);

  // Save files to see the results
  PointMatcherIO<float>::savePCD(data_out, slam_map);

  std::cout << "Final transformation from ICP:" << std::endl << T << std::endl;

  // Calculate the metric between the two Point Clouds
  char** paths;          // pointer for const char* type
  paths = new char*[3];  // When argv is provided in the Metric constructor, the first place in argv is the name of the
                         // launcher filename, so we need to provide places [1] and [2], to work properly

  paths[1] = new char[100];
  paths[2] = new char[100];
  std::strcpy(paths[1], (ground_truth_map).c_str());
  std::strcpy(paths[2], (slam_map).c_str());

  // Metric constructor implements of the functionality for calculating it, so we just need an object of it
  calculate_metric::Metric metric(paths);

#if defined(_BENCH_TIME)
  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Octomap evaluation took " << dt << " seconds.");
#endif

  bool delete_point_clouds;
  nh.param<bool>("/delete_point_clouds", delete_point_clouds, false);
  if (delete_point_clouds)
  {
    ROS_INFO("Deleting Point Clouds ....\n");
    remove(ground_truth_map.c_str());
    remove(slam_map.c_str());
  }

  // Free up memory
  delete[] paths;

  return 0;
}
