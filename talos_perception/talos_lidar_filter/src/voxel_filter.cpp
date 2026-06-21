/*******************************************************************************
 * voxel_filter
 *
 * Voxel Grid (voksel ızgara) filtresi:
 * Gelen 3B nokta bulutunu, belirlenen kenar uzunluklarına (leaf size) sahip
 * voksel hücrelerine böler. Aynı hücreye düşen birden fazla nokta, bu
 * noktaların ağırlık merkezi (centroid) ile tek bir temsilci noktaya indirilir:
 *
 *   C = (1/n) * Σ P_i
 *
 * Böylece çevrenin özellikleri korunurken nokta sayısı azalır ve sonraki
 * işlem yükü (kümeleme, engel tespiti vb.) hafifler.
 *
 * Filtre, pass_filter çıktısı /cart/points_filtered üzerinden gelen,
 * Z ekseninde zaten sınırlandırılmış buluta uygulanır.
 ******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class VoxelFilter
{
public:
  explicit VoxelFilter(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
  {
    // Parametreler (rosrun ile _param:=deger veya launch ile değiştirilebilir)
    pnh.param<std::string>("input_topic",  input_topic_,  "/cart/points_filtered");
    pnh.param<std::string>("output_topic", output_topic_, "/cart/points_voxel");
    // Voksel kenar uzunlukları (metre)
    pnh.param<double>("leaf_x", leaf_x_, 0.1);
    pnh.param<double>("leaf_y", leaf_y_, 0.1);
    pnh.param<double>("leaf_z", leaf_z_, 0.1);

    pcl_sub_ = nh_.subscribe(input_topic_, 1, &VoxelFilter::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    ROS_INFO("voxel_filter: '%s' -> '%s' | leaf = [%.3f, %.3f, %.3f]",
             input_topic_.c_str(), output_topic_.c_str(),
             leaf_x_, leaf_y_, leaf_z_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher  pcl_pub_;

  std::string input_topic_, output_topic_;
  double leaf_x_, leaf_y_, leaf_z_;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // PCLPointCloud2 ile çalış: intensity/ring gibi tüm alanlar korunur.
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_x_, leaf_y_, leaf_z_);
    voxel.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);
    output.header = cloud_msg->header;  // frame_id ve timestamp'i koru
    pcl_pub_.publish(output);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  VoxelFilter voxel_filter(nh, pnh);

  ros::spin();
  return 0;
}
