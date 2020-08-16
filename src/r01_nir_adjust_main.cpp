#include "ros/ros.h"
#include "r01_nir_adjust.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "r01_nir_adjust");
  r01_nir_adjust<pcl::PointXYZI>().spin();
  return 0;
}
