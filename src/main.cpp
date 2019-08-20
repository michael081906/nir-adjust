#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <boost/make_shared.hpp>
#include <std_msgs/Bool.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h> // used for "pcl::fromROSMsg"



typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud pointcloud_in;
std::vector<geometry_msgs::Twist> pfm0;
std::vector<geometry_msgs::Twist> pfm1;
std::vector<geometry_msgs::Twist> pfm2;
ros::NodeHandle n;
ros::Publisher traj_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/2019_animal_study/nir_points",1);
pcl::PointCloud<pcl::PointXYZI> output_traj;
int seq = 0;

pcl::PointXYZI find_average(std::vector<geometry_msgs::Twist> pfm)
{
  double x_=0;
  double y_=0;
  double z_=0;
  pcl::PointXYZI final;
  for(int i=0;i<pfm.size();i++)
  {
    x_ = x_ + pfm[i].linear.x;
    y_ = y_ + pfm[i].linear.y;
    z_ = z_ + pfm[i].linear.z;
  }
  final.x = x_/pfm.size();
  final.y = y_/pfm.size();
  final.z = z_/pfm.size();
  std::cout << final.x << " , " << final.y << " , " << final.z << std::endl;
  return final;
}



void callback(const sensor_msgs::PointCloud2Ptr& cloud) {

  pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  pfm0.clear();
  pfm1.clear();
  pfm2.clear();
  output_traj.clear();
  
  int index_=0;
  for(int i=0;i<pointcloud_in.size();i++)
  {
    geometry_msgs::Twist temp_;
    temp_.linear.x = pointcloud_in[i].x;
    temp_.linear.y = pointcloud_in[i].y;
    temp_.linear.z = pointcloud_in[i].z;
    index_ = pointcloud_in[i].intensity;
    switch (index_) {
      case 0:
      pfm0.push_back(temp_);
      break;
      case 1:
      pfm1.push_back(temp_);
      break;
      case 2:
      pfm2.push_back(temp_);
      break;
    }// end switch
  }// end for
  pcl::PointXYZI temp_p;
  if(pfm0.size()>0)
  {
  temp_p = find_average(pfm0);
  temp_p.intensity=0;
  output_traj.push_back(temp_p);
}
if(pfm1.size()>0)
{
  temp_p = find_average(pfm1);
  temp_p.intensity=1;
  output_traj.push_back(temp_p);
}
if(pfm2.size()>0)
{
  temp_p = find_average(pfm2);
  temp_p.intensity=2;
  output_traj.push_back(temp_p);
}


  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.seq = seq++; // is this correct
  header.frame_id = std::string("world");
  output_traj.header = pcl_conversions::toPCL(header);
  traj_pub_.publish(output_traj);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nir_adjust");
  ros::Subscriber sub_pcl = n.subscribe("/see_scope/overlay/cog", 1, &callback);
  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
