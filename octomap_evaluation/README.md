# octomap_evaluation

## Overview

Octomap evaluation package has as input two octomap files, received from SLAM algorithms, in order to extract some evaluation metrics. First, the octomap files are converted to Point Cloud files and then the [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/) library is used to apply an ICP transformation between the two point clouds and align them. Finallly, MSE and a normalized metric is used to compare the two octomaps.

**Keywords:** octomap, evaluation, point cloud, icp, mse

The octomap_evaluation package has been tested under ROS Kinetic and Ubuntu 16.04.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org)
- Point Cloud Library

		sudo apt-get install ros-kinetic-pcl-ros
- libpointmatcher
    Instructions on how to install [here](https://github.com/ethz-asl/libpointmatcher).
- Octomap
    Instructions on how to install [here](https://github.com/OctoMap/octomap/wiki/Compilation-and-Installation-of-OctoMap).-
- octomap_ros

		sudo apt-get install ros-kinetic-octomap-ros

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone git@github.com:kosmastsk/one-SLAM-2-rule-them-all.git
	cd one-SLAM-2-rule-them-all
	git checkout octomap-evaluation
	cd ../../
	catkin build octomap_evaluation

## Config files

octomap_evaluation/config

* **icp_config.yaml** Parameters for the registration of 3D shapes, used by the ICP algorithm

## Launch files

* **octomap_to_point_cloud.launch:** Convert octomap to Point Cloud
     - **`octomap`** Octomap file path. Default: `$(find octomap_evaluation)/maps/indoors.ot`.

* **point_cloud_icp.launch:** Apply ICP registration
     - **`yaml_file`** Path to the .yaml file for ICP. Default: `$(find octomap_evaluation)/config/icp_config.yaml`
     - **`reference`** Reference Point Cloud file. Default: `$(find octomap_evaluation)/maps/indoors.ot.pcd`
     - **`reading`** Reading Point Cloud. Default: `$(find octomap_evaluation)/maps/indoors_v2.ot.pcd`
