#include "ros/ros.h"
#include "nir_adjust.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nir_adjust");
  nir_adjust_().spin();
  return 0;
}
