#include <ros/ros.h>
#include <gtest/gtest.h>
#include <mk_kernel/mkk.h>



class nir_adjust_test : public ::testing::Test {
  protected:
    ros::NodeHandle n_;
    ros::ServiceClient kernel_main_service_;
    mk_kernel::mkk test_call;
    // Setup
    mkKernelTest()
    { kernel_main_service_ = n_.serviceClient< mk_kernel::mkk > ("/kernel_main");
  }
};


TEST_F(mkKernelTest, callCleanTest)
{
   test_call.request.task_id = 3;
   if(kernel_main_service_.call(test_call))
   {
    EXPECT_EQ(test_call.response.error_, 0);
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nir_adjust_main_test");
  return RUN_ALL_TESTS();
}
