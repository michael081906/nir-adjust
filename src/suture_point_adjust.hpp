#ifndef SUTURE_POINT_ADJUST_HPP
#define SUTURE_POINT_ADJUST_HPP
#include "pointcloud_adjust_base.hpp"

template<class T>
class suture_point_adjust : public pointcloud_adjust_base<T> {
    typedef pointcloud_adjust_base<T> Base;
    private:
    void callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud);

    public:
    suture_point_adjust() {};

};

template<class T>
void suture_point_adjust<T>::callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  pcl::fromROSMsg(*cloud, Base::pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  ROS_INFO_STREAM("suture_point_adjust: You are in the happing adjusting for suturing");
  static T temp_p;
  temp_p = Base::pointcloud_in[Base::number_marker];
  temp_p.x = temp_p.x + Base::config_m0_x;
  temp_p.y = temp_p.y + Base::config_m0_y;
  temp_p.z = temp_p.z + Base::config_m0_z;
  Base::output_traj[Base::number_marker] = temp_p;
    
  Base::receive= true;
}

#endif