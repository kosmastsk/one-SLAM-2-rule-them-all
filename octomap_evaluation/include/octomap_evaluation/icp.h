/*   */

#ifndef ICP_HEADER
#define ICP_HEADER

// C++ headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cassert>

// ROS headers
#include <ros/package.h>
#include <ros/ros.h>

// libpointmatcher
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"

// boost
#include "boost/filesystem.hpp"

namespace icp
{
class Icp
{
private:
  // Variables
  ros::NodeHandle _nh;
  std::string _reference;
  std::string _reading;

  PointMatcher<float>::ICP _icp;  // Create the default ICP algorithm

  // Methods

public:
  Icp();
  Icp(char* argv[]);
  ~Icp();
};

}  // namespace icp

#endif
