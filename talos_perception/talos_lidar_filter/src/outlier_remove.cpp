/*******************************************************************************
 * outlier_remove  (Statistical Outlier Removal - SOR)
 *
 * Sensör hataları veya çevresel partiküller nedeniyle oluşan yanlış (gürültü)
 * noktaları, noktalar arası mesafelerin Normal (Gauss) dağılımı varsayımıyla
 * istatistiksel olarak temizler.
 *
 * Adımlar:
 *  1) Lokal komşuluk: her p_i için en yakın k komşuya ortalama Öklid mesafesi
 *         d_i = (1/k) Σ_{j=1..k} ||p_i - p_ij||_2
 *  2) Küresel dağılım: tüm d_i'lerden ortalama (μ) ve standart sapma (σ)
 *         μ = (1/N) Σ d_i ,   σ = sqrt( (1/N) Σ (d_i - μ)^2 )
 *  3) Eşik ve filtreleme: tolerans çarpanı α ile
 *         T_max = μ + α·σ
 *     d_i > T_max olan noktalar (Gauss dağılımına uymayan) gürültü kabul
 *     edilip kaldırılır.
 *
 * PCL'de: mean_k = k, stddev_mul_thresh = α.
 * Filtre, ham LiDAR bulutu /cart/center_laser/scan üzerinden gelen veriye
 * uygulanır (test amaçlı; ham bulut yoğun olduğundan SOR maliyeti yüksektir).
 ******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

class OutlierRemove
{
public:
  explicit OutlierRemove(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
  {
    // Parametreler (rosrun ile _param:=deger veya launch ile değiştirilebilir)
    pnh.param<std::string>("input_topic",  input_topic_,  "/cart/center_laser/scan");
    pnh.param<std::string>("output_topic", output_topic_, "/cart/points_clean");
    pnh.param<int>("mean_k", mean_k_, 50);          // k: komşu sayısı
    pnh.param<double>("stddev_mul", stddev_mul_, 1.0); // α: std sapma çarpanı

    pcl_sub_ = nh_.subscribe(input_topic_, 1, &OutlierRemove::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    ROS_INFO("outlier_remove: '%s' -> '%s' | mean_k(k)=%d  stddev_mul(alpha)=%.2f",
             input_topic_.c_str(), output_topic_.c_str(), mean_k_, stddev_mul_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher  pcl_pub_;

  std::string input_topic_, output_topic_;
  int    mean_k_;
  double stddev_mul_;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // PCLPointCloud2 ile çalış: intensity/ring gibi tüm alanlar korunur.
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_mul_);
    sor.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);
    output.header = cloud_msg->header;  // frame_id ve timestamp'i koru
    pcl_pub_.publish(output);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "outlier_remove");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OutlierRemove outlier_remove(nh, pnh);

  ros::spin();
  return 0;
}
