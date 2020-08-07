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
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h> // used for "pcl::fromROSMsg"
#include <dynamic_reconfigure/server.h>
#include <nir_adjust/nir_adjustmentConfig.h>


int marker_number_global = 20;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud pointcloud_in;
std::vector<geometry_msgs::Twist> pfm0;
pcl::PointCloud<pcl::PointXYZI> output_traj_callback;

bool receive= false;
bool _group_exists = false;
ros::NodeHandle n;
double config_m0_x = 0.0, config_m0_y = 0.0, config_m0_z = 0.0;
int number_marker;



pcl::PointXYZI find_average(std::vector<geometry_msgs::Twist> pfm)
{
  double x_;
  double y_;
  double z_;
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
  
  output_traj_callback.clear();

  int index_=0;

  for(int intensity_ = 0; intensity_< marker_number_global; intensity_++)
  {
      _group_exists = false;
      pfm0.clear();
       
       //check if there is a markers
      for(int j=0;j< pointcloud_in.size();j++)
      {
        // check if there is such intensity exists
        if(intensity_ == pointcloud_in[j].intensity)
        {
          _group_exists = true;
          break; 
        }
      }
      if(_group_exists == false) break;
      
      for(int i=0;i< pointcloud_in.size();i++)
      {
        geometry_msgs::Twist temp_;
        temp_.linear.x = pointcloud_in[i].x;
        temp_.linear.y = pointcloud_in[i].y;
        temp_.linear.z = pointcloud_in[i].z;
        index_ = pointcloud_in[i].intensity;
        if(intensity_ == index_) pfm0.push_back(temp_);
        }// end for
        
        pcl::PointXYZI temp_p;
        if(pfm0.size()>0) 
        {
          temp_p = find_average(pfm0);
          if(intensity_== number_marker)
          {
            temp_p.x = temp_p.x + config_m0_x;
            temp_p.y = temp_p.y + config_m0_y;
            temp_p.z = temp_p.z + config_m0_z;
          }
          temp_p.intensity = intensity_;
          output_traj_callback.push_back(temp_p);
        }
       
  }

receive= true;

}

void callback_offset(const std_msgs::Float64MultiArray& offset)
{
  number_marker = offset.layout.data_offset;
  config_m0_x = offset.data[0];
  config_m0_y = offset.data[1];
  config_m0_z = offset.data[2];

}

/*void callback_d(nir_adjust::nir_adjustmentConfig &config, uint32_t level) {
 ROS_INFO("Reconfigure Request: %d %f %f %f", config.num_marker,
            config.x_offset, config.y_offset, config.z_offset);
            ros::NodeHandle nh;
            if(number_marker!= config.num_marker)
            {
              ROS_INFO_STREAM("Load the parameter here");
              nh.setParam("/nir_adjust/x_offset", offset_group_[config.num_marker].x_offset);
              nh.setParam("/nir_adjust/y_offset", offset_group_[config.num_marker].y_offset);
              nh.setParam("/nir_adjust/z_offset", offset_group_[config.num_marker].z_offset); 
              number_marker = config.num_marker;
            }
            else
            {
              ROS_INFO_STREAM("Rest the parameter here");
              number_marker = config.num_marker;
              config_m0_x = config.x_offset;
              config_m0_y = config.y_offset;
              config_m0_z = config.z_offset;         
              

            }
            
}
*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nir_adjust");

  ros::Publisher traj_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/see_scope/overlay_filtered/cog",1);
  ros::Subscriber sub_pcl = n.subscribe("/see_scope/overlay/cog", 1, &callback);
  ros::Subscriber sub_offset = n.subscribe("/array", 1, &callback_offset);
  //dynamic_reconfigure::Server<nir_adjust::nir_adjustmentConfig> server;
  //dynamic_reconfigure::Server<nir_adjust::nir_adjustmentConfig>::CallbackType f;
  //f = boost::bind(&callback_d, _1, _2);
  //server.setCallback(f);
  ros::Rate loop_rate(10);
  int seq = 0;
  pcl::PointCloud<pcl::PointXYZI> output_traj;

  while (ros::ok())
  {
    if(receive)
    {
    output_traj.clear();
    output_traj = output_traj_callback;
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

  return 0;
}
