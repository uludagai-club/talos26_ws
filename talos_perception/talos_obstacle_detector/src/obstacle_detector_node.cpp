// =============================================================================
//  talos_obstacle_detector  -  DBSCAN kumeleme + PCA tabanli OBB + zamansal takip
// -----------------------------------------------------------------------------
//  Akis:
//    /cart/points_noground (Patchwork++ zemin ayiklamasi ciktisi, engel adaylari)
//      -> DBSCAN (epsilon, MinPts) ile gurultu eleme + kumeleme
//      -> her kume icin PCA ile Oriented Bounding Box (konum/boyut/yonelim)
//      -> tracker: kareler arasi esleme + EMA yumusatma + histerezis
//      -> /obstacles (jsk BoundingBoxArray), /obstacles/poses, /obstacles/markers,
//         /obstacles/clusters (renkli debug bulutu)
//
//  Tum hesap girdi bulutunun frame'inde (velodyne) yapilir: TF interpolasyon
//  hatasi olmadan en yuksek geometrik dogruluk.
//
//  Tracker neden var: kare kare DBSCAN+PCA kutulari "git gel" yapar
//  (seyrek LiDAR'da kume yanip soner; PCA baskin ekseni ~90 derece atlar).
//  Takip + yumusatma bunlari giderir, kutulari kararli kilar.
// =============================================================================

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

namespace
{
// aci farki [-pi, pi]
inline double angDiff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
inline Eigen::Quaternionf yawQuat(double yaw)
{
  return Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(yaw),
                                              Eigen::Vector3f::UnitZ()));
}
}  // namespace

// tek karede uretilen ham olcum
struct Detection
{
  Eigen::Vector3f   pos;
  Eigen::Vector3f   dim;
  Eigen::Quaternionf quat;
  double            yaw;   // vertical_box modunda gecerli
  int               npts;
};

// zaman icinde takip edilen, yumusatilmis engel
struct Track
{
  int               id;
  Eigen::Vector3f   pos;
  Eigen::Vector3f   dim;
  Eigen::Quaternionf quat;
  double            yaw;
  int               npts;
  int               hits   = 0;
  int               misses = 0;
  bool              confirmed = false;
};

class ObstacleDetector
{
public:
  ObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, "/cart/points_noground");

    // --- DBSCAN parametreleri (dokumandaki epsilon ve MinPts) ---
    pnh.param("eps", eps_, 0.5);
    pnh.param("min_pts", min_pts_, 5);

    // --- kume kabul filtreleri ---
    pnh.param("min_cluster_size", min_cluster_size_, 10);
    pnh.param("max_cluster_size", max_cluster_size_, 25000);
    pnh.param("max_extent_xy", max_extent_xy_, 15.0);
    pnh.param("max_height", max_height_, 6.0);

    // OBB modeli: true -> Z dik, yaw PCA'dan; false -> tam 3B PCA
    pnh.param("vertical_box", vertical_box_, true);

    // --- tracker (zamansal kararlilik) ---
    pnh.param("track_enable", track_enable_, true);
    pnh.param("assoc_dist", assoc_dist_, 1.5);   // esleme kapisi (m)
    pnh.param("pos_alpha", pos_alpha_, 0.5);     // pozisyon EMA katsayisi
    pnh.param("dim_alpha", dim_alpha_, 0.3);     // boyut EMA katsayisi
    pnh.param("yaw_alpha", yaw_alpha_, 0.3);     // yonelim EMA katsayisi
    pnh.param("min_hits", min_hits_, 2);         // onaylanma icin gereken toplam kare
    pnh.param("max_misses", max_misses_, 5);     // kaybolunca canli tutma karesi

    boxes_pub_   = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacles", 1);
    poses_pub_   = nh.advertise<geometry_msgs::PoseArray>("/obstacles/poses", 1);
    marker_pub_  = nh.advertise<visualization_msgs::MarkerArray>("/obstacles/markers", 1);
    cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacles/clusters", 1);

    sub_ = nh.subscribe(input_topic_, 5, &ObstacleDetector::cloudCb, this);

    ROS_INFO("obstacle_detector: input=%s eps=%.2f min_pts=%d vertical_box=%s track=%s",
             input_topic_.c_str(), eps_, min_pts_,
             vertical_box_ ? "true" : "false", track_enable_ ? "true" : "false");
  }

private:
  // --- DBSCAN: etiketleri doldurur (>=0 kume id, NOISE gurultu) -------------
  static constexpr int UNCLASSIFIED = -2;
  static constexpr int NOISE        = -1;

  int dbscan(const CloudT::Ptr& cloud, std::vector<int>& labels)
  {
    const int n = static_cast<int>(cloud->size());
    labels.assign(n, UNCLASSIFIED);

    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(cloud);

    std::vector<int>   idx;
    std::vector<float> dist;
    int cid = 0;

    for (int i = 0; i < n; ++i)
    {
      if (labels[i] != UNCLASSIFIED) continue;

      tree.radiusSearch(cloud->points[i], eps_, idx, dist);
      if (static_cast<int>(idx.size()) < min_pts_)
      {
        labels[i] = NOISE;
        continue;
      }

      labels[i] = cid;
      std::vector<int> seeds(idx.begin(), idx.end());
      for (size_t s = 0; s < seeds.size(); ++s)
      {
        const int q = seeds[s];
        if (labels[q] == NOISE) labels[q] = cid;
        if (labels[q] != UNCLASSIFIED) continue;
        labels[q] = cid;

        std::vector<int> idx2; std::vector<float> dist2;
        tree.radiusSearch(cloud->points[q], eps_, idx2, dist2);
        if (static_cast<int>(idx2.size()) >= min_pts_)
          seeds.insert(seeds.end(), idx2.begin(), idx2.end());
      }
      ++cid;
    }
    return cid;
  }

  // --- PCA ile OBB: kume noktalarindan konum/boyut/yonelim ------------------
  bool computeOBB(const CloudT::Ptr& c, Detection& det)
  {
    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*c, centroid4);
    const Eigen::Vector3f centroid = centroid4.head<3>();

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    double yaw = 0.0;

    if (vertical_box_)
    {
      Eigen::Matrix2f cov2 = Eigen::Matrix2f::Zero();
      for (const auto& p : c->points)
      {
        const float dx = p.x - centroid.x();
        const float dy = p.y - centroid.y();
        cov2(0, 0) += dx * dx; cov2(0, 1) += dx * dy;
        cov2(1, 0) += dx * dy; cov2(1, 1) += dy * dy;
      }
      cov2 /= static_cast<float>(c->size());

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov2);
      const Eigen::Vector2f axis = es.eigenvectors().col(1);  // en buyuk ozdeger
      yaw = std::atan2(axis.y(), axis.x());
      const float cy = std::cos(yaw), sy = std::sin(yaw);
      R << cy, -sy, 0.0f,
           sy,  cy, 0.0f,
           0.0f, 0.0f, 1.0f;
    }
    else
    {
      Eigen::Matrix3f cov;
      pcl::computeCovarianceMatrixNormalized(*c, centroid4, cov);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
      R = es.eigenvectors();
      R.col(2) = R.col(0).cross(R.col(1));
    }

    Eigen::Vector3f lo( std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max());
    Eigen::Vector3f hi(-std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max());
    const Eigen::Matrix3f Rt = R.transpose();
    for (const auto& p : c->points)
    {
      const Eigen::Vector3f local = Rt * (Eigen::Vector3f(p.x, p.y, p.z) - centroid);
      lo = lo.cwiseMin(local);
      hi = hi.cwiseMax(local);
    }

    const Eigen::Vector3f dim        = hi - lo;
    const Eigen::Vector3f local_cntr = 0.5f * (lo + hi);

    if (dim.x() > max_extent_xy_ || dim.y() > max_extent_xy_ || dim.z() > max_height_)
      return false;

    det.pos  = centroid + R * local_cntr;
    det.dim  = dim.cwiseMax(0.05f);
    det.quat = Eigen::Quaternionf(R);
    det.yaw  = yaw;
    return true;
  }

  // --- tracker: olcumleri mevcut izlere esle, yumusat, histerezis -----------
  void updateTracks(std::vector<Detection>& dets)
  {
    const int N = static_cast<int>(tracks_.size());
    const int M = static_cast<int>(dets.size());

    std::vector<char> det_used(M, 0);
    std::vector<char> trk_matched(N, 0);

    // tum gecerli (iz, olcum) ciftlerini mesafeye gore sirala -> kararli greedy
    struct Pair { float d; int t; int m; };
    std::vector<Pair> pairs;
    for (int t = 0; t < N; ++t)
      for (int m = 0; m < M; ++m)
      {
        const float dx = tracks_[t].pos.x() - dets[m].pos.x();
        const float dy = tracks_[t].pos.y() - dets[m].pos.y();
        const float d  = std::sqrt(dx * dx + dy * dy);
        if (d <= assoc_dist_) pairs.push_back({d, t, m});
      }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.d < b.d; });

    for (const auto& p : pairs)
    {
      if (trk_matched[p.t] || det_used[p.m]) continue;
      trk_matched[p.t] = 1;
      det_used[p.m]    = 1;
      updateMatched(tracks_[p.t], dets[p.m]);
    }

    // eslesmeyen izler: kacirildi (hits sifirLANMAZ; toplam birikim korunur ki
    // arada bir kare cakilan gercek engel onayini kaybetmesin)
    for (int t = 0; t < N; ++t)
      if (!trk_matched[t])
        ++tracks_[t].misses;

    // eslesmeyen olcumler: yeni iz
    for (int m = 0; m < M; ++m)
      if (!det_used[m])
      {
        Track tr;
        tr.id   = next_id_++;
        tr.pos  = dets[m].pos;
        tr.dim  = dets[m].dim;
        tr.quat = dets[m].quat;
        tr.yaw  = dets[m].yaw;
        tr.npts = dets[m].npts;
        tr.hits = 1;
        tracks_.push_back(tr);
      }

    // bayatlamis izleri sil
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                    [this](const Track& t) { return t.misses > max_misses_; }),
                  tracks_.end());
  }

  void updateMatched(Track& tr, Detection& det)
  {
    tr.misses = 0;
    ++tr.hits;
    if (tr.hits >= min_hits_) tr.confirmed = true;

    const float pa = static_cast<float>(pos_alpha_);
    const float da = static_cast<float>(dim_alpha_);

    // pozisyon EMA
    tr.pos = (1.0f - pa) * tr.pos + pa * det.pos;

    if (vertical_box_)
    {
      // yaw'i izin yaw'ina en yakin 90 derece esitine cek (eksen sicramasi onlenir);
      // 90 derecelik kayma boyut x/y takasini gerektirir
      double bestY = det.yaw;
      bool   swap  = false;
      double bestd = 1e9;
      for (int k = -2; k <= 2; ++k)
      {
        const double y = det.yaw + k * M_PI / 2.0;
        const double d = std::fabs(angDiff(y, tr.yaw));
        if (d < bestd) { bestd = d; bestY = y; swap = (std::abs(k) % 2 == 1); }
      }
      if (swap) std::swap(det.dim.x(), det.dim.y());

      tr.yaw  = tr.yaw + yaw_alpha_ * angDiff(bestY, tr.yaw);
      tr.quat = yawQuat(tr.yaw);
    }
    else
    {
      tr.quat = tr.quat.slerp(static_cast<float>(yaw_alpha_), det.quat);
    }

    // boyut EMA
    tr.dim  = (1.0f - da) * tr.dim + da * det.dim;
    tr.npts = det.npts;
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    CloudT::Ptr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);

    CloudT::Ptr clean(new CloudT());
    clean->reserve(cloud->size());
    for (const auto& p : cloud->points)
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        clean->push_back(p);

    const std_msgs::Header hdr = msg->header;

    pcl::PointCloud<pcl::PointXYZRGB> colored;
    std::vector<Detection> dets;

    if (!clean->empty())
    {
      std::vector<int> labels;
      const int ncl = dbscan(clean, labels);

      std::vector<std::vector<int>> groups(ncl);
      for (int i = 0; i < static_cast<int>(labels.size()); ++i)
        if (labels[i] >= 0) groups[labels[i]].push_back(i);

      int cidx = 0;
      for (const auto& g : groups)
      {
        const int sz = static_cast<int>(g.size());
        if (sz < min_cluster_size_ || sz > max_cluster_size_) continue;

        CloudT::Ptr c(new CloudT());
        c->reserve(sz);
        for (int i : g) c->push_back(clean->points[i]);

        Detection det;
        if (!computeOBB(c, det)) continue;
        det.npts = sz;
        dets.push_back(det);

        const float r  = (cidx * 73  % 255) / 255.0f;
        const float gg = (cidx * 151 % 255) / 255.0f;
        const float b  = (cidx * 223 % 255) / 255.0f;
        for (const auto& p : c->points)
        {
          pcl::PointXYZRGB cp;
          cp.x = p.x; cp.y = p.y; cp.z = p.z;
          cp.r = static_cast<uint8_t>(r * 255);
          cp.g = static_cast<uint8_t>(gg * 255);
          cp.b = static_cast<uint8_t>(b * 255);
          colored.push_back(cp);
        }
        ++cidx;
      }
    }

    // ---- takip ----
    jsk_recognition_msgs::BoundingBoxArray boxes;
    geometry_msgs::PoseArray poses;
    visualization_msgs::MarkerArray markers;
    boxes.header = hdr;
    poses.header = hdr;

    auto fillOutputs = [&](int id, const Eigen::Vector3f& pos,
                           const Eigen::Vector3f& dim,
                           const Eigen::Quaternionf& q, int npts)
    {
      jsk_recognition_msgs::BoundingBox box;
      box.header = hdr;
      box.pose.position.x = pos.x();
      box.pose.position.y = pos.y();
      box.pose.position.z = pos.z();
      box.pose.orientation.x = q.x();
      box.pose.orientation.y = q.y();
      box.pose.orientation.z = q.z();
      box.pose.orientation.w = q.w();
      box.dimensions.x = std::max(dim.x(), 0.05f);
      box.dimensions.y = std::max(dim.y(), 0.05f);
      box.dimensions.z = std::max(dim.z(), 0.05f);
      box.label = id;
      box.value = static_cast<float>(npts);
      boxes.boxes.push_back(box);

      poses.poses.push_back(box.pose);

      visualization_msgs::Marker mk;
      mk.header = hdr;
      mk.ns = "obstacle_ids";
      mk.id = id;
      mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose = box.pose;
      mk.pose.position.z += 0.5 * box.dimensions.z + 0.3;
      mk.scale.z = 0.5;
      mk.color.r = 1.0; mk.color.g = 1.0; mk.color.b = 0.0; mk.color.a = 1.0;
      mk.lifetime = ros::Duration(0.3);
      mk.text = "obs " + std::to_string(id) + " (" + std::to_string(npts) + ")";
      markers.markers.push_back(mk);
    };

    int published = 0;
    if (track_enable_)
    {
      updateTracks(dets);
      for (const auto& tr : tracks_)
      {
        // Gosterme kurali:
        //  - bu karede tespit edildiyse (misses==0) DAIMA goster
        //    -> hicbir gercek engel kaybolmaz (tum kutular gelir)
        //  - bu karede kaybolduysa yalniz ONAYLI iz gosterilir (grace icinde)
        //    -> kurulu engelin kisa kopmalari koprulenir, 1 karelik gurultu gosterilmez
        const bool detected_now = (tr.misses == 0);
        if (!detected_now && !tr.confirmed) continue;
        fillOutputs(tr.id, tr.pos, tr.dim, tr.quat, tr.npts);
        ++published;
      }
    }
    else
    {
      int id = 0;
      for (const auto& d : dets)
      {
        fillOutputs(id++, d.pos, d.dim, d.quat, d.npts);
        ++published;
      }
    }

    boxes_pub_.publish(boxes);
    poses_pub_.publish(poses);
    marker_pub_.publish(markers);

    sensor_msgs::PointCloud2 cmsg;
    pcl::toROSMsg(colored, cmsg);
    cmsg.header = hdr;
    cluster_pub_.publish(cmsg);

    ROS_INFO_THROTTLE(2.0,
        "obstacle_detector: %d nokta -> %zu olcum -> %d kararli engel (iz=%zu)",
        static_cast<int>(clean->size()), dets.size(), published, tracks_.size());
  }

  // params
  std::string input_topic_;
  double eps_, max_extent_xy_, max_height_;
  int    min_pts_, min_cluster_size_, max_cluster_size_;
  bool   vertical_box_;

  // tracker params
  bool   track_enable_;
  double assoc_dist_, pos_alpha_, dim_alpha_, yaw_alpha_;
  int    min_hits_, max_misses_;

  // tracker state
  std::vector<Track> tracks_;
  int next_id_ = 0;

  // ros
  ros::Subscriber sub_;
  ros::Publisher  boxes_pub_, poses_pub_, marker_pub_, cluster_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ObstacleDetector node(nh, pnh);
  ros::spin();
  return 0;
}
