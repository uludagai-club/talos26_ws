/*******************************************************************************
 * pass_filter
 *
 * Pass-through (geçiş) filtresi:
 * Gelen 3B nokta bulutunu Z ekseni boyunca belirli bir aralığa (ROI) göre
 * kırpar. Z_min altındaki zemin yansımaları ve Z_max üstündeki araç tavanı /
 * üst geçiş gibi sürüşü etkilemeyen noktalar temizlenir. Böylece sonraki
 * işlem yükü (kümeleme, engel tespiti vb.) azalır.
 *
 *   P_ROI = { p_i ∈ P | Z_min <= z_i <= Z_max }
 *
 * Filtre, /cart/center_laser/scan (velodyne frame) üzerinden gelen ham LiDAR
 * bulutuna uygulanır. Bu frame'de LiDAR orijindedir; LiDAR chassis üzerinde
 * z=0.92'de monteli olduğundan zemin ~z=-0.92 civarına düşer. Z sınırlarını
 * (z_min/z_max) buna göre ayarlayın.
 ******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>

class PassFilter
{
public:
  explicit PassFilter(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
  {
    // Parametreler (launch / rosparam ile değiştirilebilir)
    pnh.param<std::string>("input_topic",  input_topic_,  "/cart/center_laser/scan");
    pnh.param<std::string>("output_topic", output_topic_, "/cart/points_filtered");
    pnh.param<std::string>("filter_field", filter_field_, "z");
    pnh.param<double>("z_min", z_min_, -0.7);  // zemin yansımalarını kes (velodyne frame)
    pnh.param<double>("z_max", z_max_, 1.0);   // araç tavanı / üst geçiş üstünü kes
    pnh.param<bool>("negative", negative_, false);

    pcl_sub_ = nh_.subscribe(input_topic_, 1, &PassFilter::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    ROS_INFO("pass_filter: '%s' -> '%s' | %s in [%.2f, %.2f]",
             input_topic_.c_str(), output_topic_.c_str(),
             filter_field_.c_str(), z_min_, z_max_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher  pcl_pub_;

  std::string input_topic_, output_topic_, filter_field_;
  double z_min_, z_max_;
  bool   negative_;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // PCLPointCloud2 ile çalış: intensity/ring gibi tüm alanlar korunur.
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(filter_field_);
    pass.setFilterLimits(z_min_, z_max_);
    pass.setNegative(negative_);
    pass.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);
    output.header = cloud_msg->header;  // frame_id ve timestamp'i koru
    pcl_pub_.publish(output);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pass_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PassFilter pass_filter(nh, pnh);

  ros::spin();
  return 0;
}
