#include <ros/ros.h>
#include <gtest/gtest.h>
#include "nir_adjust.hpp"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nir_adjust_main_test");
  return RUN_ALL_TESTS();
}
