
#ifndef POINTCLOUD_ADJUST_BASE_HPP
#define POINTCLOUD_ADJUST_BASE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <vector>
#include <array>
#include <boost/make_shared.hpp>
#include <std_msgs/Bool.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>  
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h> // used for "pcl::fromROSMsg"


template<class T>
class pointcloud_adjust_base
{
  typedef pcl::PointCloud<T> PointCloud_type;
  
  private:
  ros::Publisher traj_pub_;
  ros::Subscriber sub_offset;
  void callback_offset(const std_msgs::Float64MultiArrayConstPtr& offset);
  
  protected: 
  ros::NodeHandle n;
  ros::Subscriber sub_pcl;

  bool receive= false;  //TODO: This might be an issue
  bool _group_exists = false;
  int marker_number_global = 20;
  double config_m0_x = 0.0, config_m0_y = 0.0, config_m0_z = 0.0;
  int number_marker = 0;  
  PointCloud_type pointcloud_in;
  PointCloud_type output_traj;
  std::vector<geometry_msgs::Twist> pfm0;
  virtual void callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud);
  
  public:
  pointcloud_adjust_base();
  void spin();
  // 2020/8/7 必須要是const..... ex: sensor_msgs::PointCloud2ConstPtr 如果用 sensor_msgs::PointCloud2Ptr 過不了的
  typedef boost::function<void (const sensor_msgs::PointCloud2ConstPtr& cloud)> pc_t;
  typedef boost::function<void (const std_msgs::Float64MultiArrayConstPtr& offset)> offest_t;
};

template<class T>
pointcloud_adjust_base<T>::pointcloud_adjust_base()
{
  traj_pub_ = n.advertise<PointCloud_type> ("/see_scope/overlay_filtered/cog",1);
    
  pc_t pt_sub_cb = boost::bind(&pointcloud_adjust_base::callback_pc, this, _1);
  sub_pcl = n.subscribe("/see_scope/overlay/cog", 1000, pt_sub_cb);

  offest_t offset_sub_callback = boost::bind(&pointcloud_adjust_base::callback_offset, this, _1);
  sub_offset = n.subscribe("/array", 1000, offset_sub_callback);

}

template <class T>
void pointcloud_adjust_base<T>::callback_offset(const std_msgs::Float64MultiArrayConstPtr& offset)
{
  number_marker = offset->layout.data_offset;
  config_m0_x = offset->data[0];
  config_m0_y = offset->data[1];
  config_m0_z = offset->data[2];

}

template <class T>
void pointcloud_adjust_base<T>::callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud) 
{
  ROS_INFO_STREAM("pointcloud_adjust_base says: Dude, I'm not doing anything.");
}

template <class T>
void pointcloud_adjust_base<T>::spin()
{ 
  ros::Rate loop_rate(10);
  int seq = 0;
  while (ros::ok())
  {
    if(receive)
    {
    std::cout << "receive= "<< receive << std::endl;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = seq++; // is this correct?
    header.frame_id = std::string("mono");
    output_traj.header = pcl_conversions::toPCL(header);
    traj_pub_.publish(output_traj);
    receive = false;
  }
    ros::spinOnce();
    loop_rate.sleep();
  }


}

#endif