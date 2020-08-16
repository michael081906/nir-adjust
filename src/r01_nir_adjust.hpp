#ifndef R01_NIR_ADJUST_HPP
#define R01_NIR_ADJUST_HPP
#include "pointcloud_adjust_base.hpp"


template<class T>
class r01_nir_adjust : public pointcloud_adjust_base<T> {

  typedef pointcloud_adjust_base<T> Base;
  private:
  
  
  T find_average(std::vector<geometry_msgs::Twist>& pfm);
  void callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud);
  
  public:
  r01_nir_adjust();
  };

template<class T>
r01_nir_adjust<T>::r01_nir_adjust() {
  //typename Base::pc_t pt_sub_cb = boost::bind(&r01_nir_adjust::callback_pc, this, _1);
  //Base::sub_pcl = Base::n.subscribe("/see_scope/overlay/cog", 1000, pt_sub_cb);
}

/*
 This function was written for adjusting pointcloud with type PointXYZI based on its intensity value. 
 
 The index you specify on the GUI is actually the intensity value.

 This means that you need to subscribe PointXYZI pointcloud in order to make this function work.

*/
template<class T>
void r01_nir_adjust<T>::callback_pc(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  pcl::fromROSMsg(*cloud, Base::pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  
  Base::output_traj.clear();

  int index_=0;

  for(int intensity_ = 0; intensity_< Base::marker_number_global; intensity_++)
  {
      Base::_group_exists = false;
      Base::pfm0.clear();
       
       //check if there is a markers
      for(int j=0;j< Base::pointcloud_in.size();j++)
      {
        // check if there is such intensity exists
        if(intensity_ == Base::pointcloud_in[j].intensity)
        {
          Base::_group_exists = true;
          break; 
        }
      }
      if(Base::_group_exists == false) break;
      
      for(int i=0;i< Base::pointcloud_in.size();i++)
      {
        geometry_msgs::Twist temp_;
        temp_.linear.x = Base::pointcloud_in[i].x;
        temp_.linear.y = Base::pointcloud_in[i].y;
        temp_.linear.z = Base::pointcloud_in[i].z;
        index_ = Base::pointcloud_in[i].intensity;
        if(intensity_ == index_) Base::pfm0.push_back(temp_);
        }// end for
        
        T temp_p;
        if(Base::pfm0.size()>0) 
        {
          temp_p = find_average(Base::pfm0);
          if(intensity_== Base::number_marker)
          {
            temp_p.x = temp_p.x + Base::config_m0_x;
            temp_p.y = temp_p.y + Base::config_m0_y;
            temp_p.z = temp_p.z + Base::config_m0_z;
          }
          temp_p.intensity = intensity_;
          Base::output_traj.push_back(temp_p);
        }
       
  }
  
  Base::receive= true;
}

template<class T>
T r01_nir_adjust<T>::find_average(std::vector<geometry_msgs::Twist>& pfm)
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

#endif