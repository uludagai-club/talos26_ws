/*******************************************************************************
 * ground_removal  (Patchwork++ Zemin Ayiklama)
 *
 * LiDAR verilerinde zemin-engel temasi, nesnelerin dogru ayrismasini
 * zorlastirir. Egimli/suspansiyon sarsintili zeminlerde sabit bir Z esigi
 * ve hatta TEK bir global RANSAC duzlemi yetersiz kalir: gercek zemin genis
 * alanda tek duzlem degildir (egim, rampa, kaldirim).
 *
 * Bu node zemini Patchwork++ ile ayiklar. Patchwork++, sensor cevresini
 * esmerkezli halka/sektorlere boler (Concentric Zone Model), her hucreye PCA
 * ile YEREL bir duzlem oturtur (R-GPF), egim/yukseklik esiklerini sahaya gore
 * adapte eder (A-GLE) ve kare-arasi "ground revert" (TGR) ile titremeyi
 * bastirir. Boylece eski RANSAC node'undaki elle yazilan iki mekanizma:
 *   - plane_smooth (EMA zamansal yumusatma)  -> enable_TGR
 *   - use_perpendicular / eps_angle          -> uprightness_thr
 * yerlesik ve daha saglam karsiliklariyla gelir.
 *
 * Bu node yalnizca I/O ve ego (arac) kirpmasindan sorumludur; algoritma
 * tamamen vendor'lanmis PatchWorkpp<PointT> sinifindadir. PatchWorkpp kendi
 * parametrelerini (~sensor_height, ~czm/*, ~th_dist ...) ozel namespace'ten
 * okur; bkz. config/ground_removal.yaml.
 *
 * Akis:
 *   input_topic (varsayilan /cart/points_voxel)
 *     -> ego (arac) yaricap kirpmasi
 *     -> PatchWorkpp::estimate_ground
 *     -> obstacle_topic (/cart/points_noground)  + ground_topic (/cart/points_ground)
 ******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <patchworkpp/patchworkpp.hpp>

#include <memory>

using PointT = pcl::PointXYZI;   // intensity korunur (Patchwork++ RNR icin gerekir)
using CloudT = pcl::PointCloud<PointT>;

class GroundRemoval
{
public:
  explicit GroundRemoval(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
  {
    pnh.param<std::string>("input_topic",    input_topic_,    "/cart/points_voxel");
    pnh.param<std::string>("obstacle_topic", obstacle_topic_, "/cart/points_noground");
    pnh.param<std::string>("ground_topic",   ground_topic_,   "/cart/points_ground");

    // Ego (arac) yaricapi: LiDAR'a XY'de bu mesafeden yakin noktalar (aracin
    // kendi direkleri/govdesi) Patchwork++'a hic verilmeden tamamen atilir;
    // boylece kendi araci engel olarak raporlanmaz. (Patchwork++'in min_r'si
    // bu noktalari segmentasyon disi birakir ama silmeyebilir; bu yuzden burada
    // acikca kirpiyoruz.)
    pnh.param<double>("min_range", min_range_, 2.5);
    // Azami menzil (0 = sinirsiz): cok uzak gurultulu noktalari kirpar.
    pnh.param<double>("max_range", max_range_, 0.0);

    // Patchwork++ cekirdegi: tum algoritma parametrelerini ozel namespace'ten
    // (sensor_height, num_iter, th_dist, czm/* ...) kendisi okur.
    pwpp_ = std::make_unique<PatchWorkpp<PointT>>(&pnh);

    sub_      = nh_.subscribe(input_topic_, 1, &GroundRemoval::cloudCb, this);
    pub_obst_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic_, 1);
    pub_grnd_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_, 1);

    ROS_INFO("ground_removal (Patchwork++): '%s' -> engel:'%s' / zemin:'%s' | "
             "ego min_range=%.2f max_range=%.2f",
             input_topic_.c_str(), obstacle_topic_.c_str(), ground_topic_.c_str(),
             min_range_, max_range_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_obst_, pub_grnd_;

  std::string input_topic_, obstacle_topic_, ground_topic_;
  double min_range_, max_range_;

  std::unique_ptr<PatchWorkpp<PointT>> pwpp_;

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);
    if (raw->empty())
      return;

    // --- 1) Ego (arac) yaricap kirpmasi ---
    CloudT cloud;
    cloud.reserve(raw->size());
    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;
    for (const auto& p : raw->points)
    {
      const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
      if (r2 < min_r2) continue;
      if (max_range_ > 0.0 && r2 > max_r2) continue;
      cloud.push_back(p);
    }
    if (cloud.empty())
      return;
    cloud.header = raw->header;   // frame_id (viz icin) korunur

    // --- 2) Patchwork++ ile zemin/engel ayrimi ---
    CloudT ground, nonground;
    double time_taken = 0.0;
    pwpp_->estimate_ground(cloud, ground, nonground, time_taken);

    ROS_INFO_THROTTLE(2.0,
      "ground_removal (Patchwork++): giris=%zu zemin=%zu engel=%zu (%.1f ms)",
      cloud.size(), ground.size(), nonground.size(), time_taken * 1000.0);

    publish(nonground, pub_obst_, msg->header);
    publish(ground,    pub_grnd_, msg->header);
  }

  void publish(const CloudT& cloud, ros::Publisher& pub,
               const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(cloud, out);
    out.header = header;   // frame_id ve timestamp korunur
    pub.publish(out);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_removal");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GroundRemoval ground_removal(nh, pnh);

  ros::spin();
  return 0;
}
