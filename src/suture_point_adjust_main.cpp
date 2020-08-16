#include "ros/ros.h"
#include "suture_point_adjust.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suture_point_adjust");
  suture_point_adjust<pcl::PointXYZI>().spin();
  return 0;
}
