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
class nir_adjust_
{
  typedef pcl::PointCloud<T> PointCloud_type;
  
  private:
  ros::NodeHandle n;
  ros::Publisher traj_pub_;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_offset;

  bool receive= false;
  bool _group_exists = false;
  int marker_number_global = 20;
  double config_m0_x = 0.0, config_m0_y = 0.0, config_m0_z = 0.0;
  int number_marker = 0;  

  PointCloud_type pointcloud_in;
  PointCloud_type output_traj_callback;
  PointCloud_type output_traj;
  std::vector<geometry_msgs::Twist> pfm0;
  T find_average(std::vector<geometry_msgs::Twist>& pfm);

  void callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void callback_offset(const std_msgs::Float64MultiArrayConstPtr& offset);

  public:
  nir_adjust_();
  void spin();
  // 2020/8/7 必須要是const..... ex: sensor_msgs::PointCloud2ConstPtr 如果用 sensor_msgs::PointCloud2Ptr 過不了的
  typedef boost::function<void (const sensor_msgs::PointCloud2ConstPtr& cloud)> pc_t;
  typedef boost::function<void (const std_msgs::Float64MultiArrayConstPtr& offset)> offest_t;
};

template<class T>
nir_adjust_<T>::nir_adjust_()
{
  traj_pub_ = n.advertise<PointCloud_type> ("/see_scope/overlay_filtered/cog",1);

  pc_t pt_sub_cb = boost::bind(&nir_adjust_::callback_pc, this, _1);
  sub_pcl = n.subscribe("/see_scope/overlay/cog", 1000, pt_sub_cb);

  offest_t offset_sub_callback = boost::bind(&nir_adjust_::callback_offset, this, _1);
  sub_offset = n.subscribe("/array", 1000, offset_sub_callback);

}

template<class T>
T nir_adjust_<T>::find_average(std::vector<geometry_msgs::Twist>& pfm)
{
  double x_;
  double y_;
  double z_;
  T final;
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

/*
 This function was written for adjusting pointcloud with type PointXYZI based on its intensity value. 
 
 The index you specify on the GUI is actually the intensity value.

 This means that you need to subscribe PointXYZI pointcloud in order to make this function work.

*/
template<class T>
void nir_adjust_<T>::callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud) {

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
        
        T temp_p;
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
  
  output_traj = output_traj_callback;
  receive= true;
}

template <class T>
void nir_adjust_<T>::callback_offset(const std_msgs::Float64MultiArrayConstPtr& offset)
{
  number_marker = offset->layout.data_offset;
  config_m0_x = offset->data[0];
  config_m0_y = offset->data[1];
  config_m0_z = offset->data[2];

}

template <class T>
void nir_adjust_<T>::spin()
{ 
  ros::Rate loop_rate(10);
  int seq = 0;
  while (ros::ok())
  {
    if(receive)
    {
    output_traj.clear();
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
