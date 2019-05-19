# octomap_evaluation

## Overview

OctoMap evaluation package has as input two OctoMap files, received from SLAM algorithms, in order to extract some evaluation metrics. First, the octomap files are converted to Point Cloud files, both of them are subsampled and then the [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/) library is used to apply an ICP transformation between the two point clouds and align them. Finallly, MSE and a normalized metric is calculated to evaluate an octomap with regards to the ground truth.

**Keywords:** octomap, evaluation, point cloud, icp, mse

The octomap_evaluation package has been tested under ROS Kinetic and Ubuntu 16.04.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org)
- Point Cloud Library

		sudo apt-get install ros-kinetic-pcl-ros
- libpointmatcher  
    Instructions on how to install [here](https://github.com/ethz-asl/libpointmatcher)
- OctoMap  
    Instructions on how to install [here](https://github.com/OctoMap/octomap/wiki/Compilation-and-Installation-of-OctoMap)
- octomap_ros

		sudo apt-get install ros-kinetic-octomap-ros
- OpenMP (optional)

		sudo apt install libomp-dev
#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone git@github.com:kosmastsk/one-SLAM-2-rule-them-all.git
	cd ..
	catkin build octomap_evaluation

## Usage

Run the main node with

	roslaunch octomap_evaluation octomap_evaluation.launch
This node accepts only binary octrees that represent an octomap (`filename.bt`). In case your octomap is in `.ot` format, convert it by running:

	convert_octree <input_filename>.bt <output_filename>.ot

The result, as well as other useful information, such as the subsampling factor, will be printed in the terminal. There is the option to keep or delete the PointCloud files after the evaluation, by setting it in the configuration file.

## Config files

octomap_evaluation/config

* **icp_config.yaml** Parameters for the registration of 3D shapes, used by the ICP algorithm

* **evaluation_parameters.yaml** Parameters related to the evaluation: voxel leaf size for the PointCloud subsampling, option to keep or delete PointCloud files after evaluation

## Launch files

* **octomap_to_point_cloud.launch:** Convert OctoMap to Point Cloud
     - **`octomap`** Octomap file path. Default: `$(find octomap_evaluation)/maps/indoors.ot`.
     - **`leaf_size`** Leaf size used for the subsampling of the PointCloud. Default: `0.02`.

* **point_cloud_icp.launch:** Apply ICP registration between two PointClouds
     - **`yaml_file`** Path to the .yaml file for ICP. Default: `$(find octomap_evaluation)/config/icp_config.yaml`
     - **`reference`** Reference Point Cloud file. Default: `$(find octomap_evaluation)/maps/indoors.bt.pcd`
     - **`reading`** Reading Point Cloud. Default: `$(find octomap_evaluation)/maps/indoors_v2.bt.pcd`

* **octomap_evaluation.launch:** Calculate the evaluation metric between two binary OctoMap files
     - **`reference`** Reference Point Cloud file. Default: `$(find octomap_evaluation)/maps/indoors.bt.pcd`
     - **`reading`** Reading Point Cloud. Default: `$(find octomap_evaluation)/maps/indoors_v2.bt.pcd`
     - **`yaml_file`** Path to the .yaml file for ICP. Default: `$(find octomap_evaluation)/config/icp_config.yaml`
     - **`leaf_size`** Leaf size used for the subsampling of the PointCloud. Default: `0.02`.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/one-SLAM-2-rule-them-all/issues).
