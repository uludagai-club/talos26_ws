#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <cmath>

class ObstacleDetector
{
public:
  ObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_listener_(tf_buffer_)
  {
    pnh.param<std::string>("map_topic", map_topic_, "/map_cloud");
    pnh.param<std::string>("input_topic", input_topic_, "/cart/center_laser/scan");
    pnh.param<std::string>("output_frame", output_frame_, "map");
    pnh.param("novel_threshold", novel_threshold_, 0.30);
    pnh.param("cluster_tolerance", cluster_tolerance_, 0.40);
    pnh.param("min_cluster_size", min_cluster_size_, 8);
    pnh.param("max_cluster_size", max_cluster_size_, 25000);
    pnh.param("max_extent_xy", max_extent_xy_, 5.0);
    pnh.param("map_voxel_leaf", map_voxel_leaf_, 0.10);
    pnh.param("input_voxel_leaf", input_voxel_leaf_, 0.15);
    pnh.param("tf_timeout", tf_timeout_, 0.10);
    pnh.param("min_range", min_range_, 2.5);

    boxes_pub_    = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacles", 1);
    poses_pub_    = nh.advertise<geometry_msgs::PoseArray>("/obstacles/poses", 1);
    cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("/obstacles/cloud", 1);
    marker_pub_   = nh.advertise<visualization_msgs::MarkerArray>("/obstacles/markers", 1);
    extremes_pub_ = nh.advertise<geometry_msgs::PoseArray>("/obstacles/x_extremes", 1);

    map_sub_   = nh.subscribe(map_topic_, 1, &ObstacleDetector::mapCallback, this);
    input_sub_ = nh.subscribe(input_topic_, 5, &ObstacleDetector::inputCallback, this);

    ROS_INFO("obstacle_detector: waiting for map on %s and clouds on %s (min_range=%.2f)",
             map_topic_.c_str(), input_topic_.c_str(), min_range_);
  }

private:
  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    if (map_voxel_leaf_ > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(map_voxel_leaf_, map_voxel_leaf_, map_voxel_leaf_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
      vg.filter(*ds);
      cloud = ds;
    }

    map_cloud_ = cloud;
    map_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    map_kdtree_->setInputCloud(map_cloud_);
    map_frame_ = msg->header.frame_id;
    map_ready_ = true;

    ROS_INFO("obstacle_detector: map ready (%zu points, frame=%s)",
             map_cloud_->size(), map_frame_.c_str());
  }

  void inputCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!map_ready_) return;

    // 1) radius filter in sensor frame (sensor origin = (0,0,0) in cloud frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *raw);
      const double r2 = min_range_ * min_range_;
      filtered->reserve(raw->size());
      for (const auto& p : raw->points)
      {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.x * p.x + p.y * p.y + p.z * p.z < r2) continue;
        filtered->push_back(p);
      }
    }
    if (filtered->empty()) return;

    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered, filtered_msg);
    filtered_msg.header = msg->header;

    // 2) transform to output frame
    sensor_msgs::PointCloud2 cloud_out;
    try
    {
      auto tf = tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id,
                                           msg->header.stamp, ros::Duration(tf_timeout_));
      tf2::doTransform(filtered_msg, cloud_out, tf);
    }
    catch (tf2::TransformException&)
    {
      try
      {
        auto tf = tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id,
                                             ros::Time(0), ros::Duration(tf_timeout_));
        tf2::doTransform(filtered_msg, cloud_out, tf);
      }
      catch (tf2::TransformException& e2)
      {
        ROS_WARN_THROTTLE(2.0, "obstacle_detector: TF lookup failed: %s", e2.what());
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_out, *cloud);

    if (input_voxel_leaf_ > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(input_voxel_leaf_, input_voxel_leaf_, input_voxel_leaf_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
      vg.filter(*ds);
      cloud = ds;
    }

    // 3) KD-tree diff against map
    pcl::PointCloud<pcl::PointXYZ>::Ptr novel(new pcl::PointCloud<pcl::PointXYZ>());
    novel->reserve(cloud->size());
    const double thr_sq = novel_threshold_ * novel_threshold_;
    std::vector<int> idx(1);
    std::vector<float> dist_sq(1);
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (const auto& p : cloud->points)
      {
        if (map_kdtree_->nearestKSearch(p, 1, idx, dist_sq) > 0)
        {
          if (dist_sq[0] > thr_sq) novel->push_back(p);
        }
        else
        {
          novel->push_back(p);
        }
      }
    }

    sensor_msgs::PointCloud2 diff_msg;
    pcl::toROSMsg(*novel, diff_msg);
    diff_msg.header.frame_id = output_frame_;
    diff_msg.header.stamp = msg->header.stamp;
    cloud_pub_.publish(diff_msg);

    if (novel->empty()) return;

    // 4) Euclidean clustering on novel cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(novel);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(novel);
    ec.extract(cluster_indices);

    jsk_recognition_msgs::BoundingBoxArray boxes;
    geometry_msgs::PoseArray poses;
    geometry_msgs::PoseArray extremes;
    visualization_msgs::MarkerArray markers;

    boxes.header.frame_id = output_frame_;
    boxes.header.stamp = msg->header.stamp;
    poses.header   = boxes.header;
    extremes.header = boxes.header;

    int label = 0;
    for (const auto& ci : cluster_indices)
    {
      // gather cluster XYZ
      const size_t N = ci.indices.size();
      Eigen::MatrixXd P(N, 3);
      for (size_t i = 0; i < N; ++i)
      {
        const auto& p = novel->points[ci.indices[i]];
        P(i, 0) = p.x; P(i, 1) = p.y; P(i, 2) = p.z;
      }

      // hybrid OBB: PCA on XY (yaw-only), Z world-aligned
      const double cx = P.col(0).mean();
      const double cy = P.col(1).mean();
      Eigen::MatrixXd C(N, 2);
      C.col(0) = P.col(0).array() - cx;
      C.col(1) = P.col(1).array() - cy;

      Eigen::Matrix2d cov = (C.transpose() * C) / static_cast<double>(N);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
      Eigen::Vector2d ev = es.eigenvectors().col(1);          // largest-eigenvalue axis
      double yaw = std::atan2(ev.y(), ev.x());

      const double c = std::cos(yaw), s = std::sin(yaw);
      // local x = (dx*c + dy*s); local y = (-dx*s + dy*c)
      Eigen::VectorXd lx = C.col(0) * c + C.col(1) * s;
      Eigen::VectorXd ly = -C.col(0) * s + C.col(1) * c;

      double lx_min, lx_max, ly_min, ly_max;
      Eigen::Index lx_min_i, lx_max_i;
      lx_min = lx.minCoeff(&lx_min_i);
      lx_max = lx.maxCoeff(&lx_max_i);
      ly_min = ly.minCoeff();
      ly_max = ly.maxCoeff();
      const double z_min = P.col(2).minCoeff();
      const double z_max = P.col(2).maxCoeff();

      const double dim_x = lx_max - lx_min;
      const double dim_y = ly_max - ly_min;
      const double dim_z = z_max - z_min;
      if (dim_x > max_extent_xy_ || dim_y > max_extent_xy_) continue;

      // OBB center: midpoint of local extents transformed back to world
      const double mid_lx = 0.5 * (lx_max + lx_min);
      const double mid_ly = 0.5 * (ly_max + ly_min);
      const double box_cx = cx + mid_lx * c - mid_ly * s;
      const double box_cy = cy + mid_lx * s + mid_ly * c;
      const double box_cz = 0.5 * (z_min + z_max);

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      jsk_recognition_msgs::BoundingBox box;
      box.header = boxes.header;
      box.pose.position.x = box_cx;
      box.pose.position.y = box_cy;
      box.pose.position.z = box_cz;
      box.pose.orientation.x = q.x();
      box.pose.orientation.y = q.y();
      box.pose.orientation.z = q.z();
      box.pose.orientation.w = q.w();
      box.dimensions.x = std::max(dim_x, 0.05);
      box.dimensions.y = std::max(dim_y, 0.05);
      box.dimensions.z = std::max(dim_z, 0.05);
      box.label = label;
      box.value = static_cast<float>(N);
      boxes.boxes.push_back(box);

      geometry_msgs::Pose pose;
      pose.position = box.pose.position;
      pose.orientation.w = 1.0;
      poses.poses.push_back(pose);

      // extreme points along local x (long axis of the OBB), in world frame
      geometry_msgs::Pose ext_lo, ext_hi;
      ext_lo.position.x = P(lx_min_i, 0);
      ext_lo.position.y = P(lx_min_i, 1);
      ext_lo.position.z = P(lx_min_i, 2);
      ext_lo.orientation.w = 1.0;
      ext_hi.position.x = P(lx_max_i, 0);
      ext_hi.position.y = P(lx_max_i, 1);
      ext_hi.position.z = P(lx_max_i, 2);
      ext_hi.orientation.w = 1.0;
      extremes.poses.push_back(ext_lo);
      extremes.poses.push_back(ext_hi);

      visualization_msgs::Marker txt;
      txt.header = boxes.header;
      txt.ns = "obstacle_ids";
      txt.id = label;
      txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::Marker::ADD;
      txt.pose = box.pose;
      txt.pose.position.z += 0.5 * box.dimensions.z + 0.3;
      txt.scale.z = 0.6;
      txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 0.0; txt.color.a = 1.0;
      txt.lifetime = ros::Duration(0.3);
      txt.text = "obs " + std::to_string(label) + " (" + std::to_string(N) + ")";
      markers.markers.push_back(txt);

      // line marker for the OBB long-axis extremes
      visualization_msgs::Marker line;
      line.header = boxes.header;
      line.ns = "obstacle_extremes";
      line.id = label;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.08;
      line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;
      line.lifetime = ros::Duration(0.3);
      line.points.push_back(ext_lo.position);
      line.points.push_back(ext_hi.position);
      markers.markers.push_back(line);

      ++label;
    }

    boxes_pub_.publish(boxes);
    poses_pub_.publish(poses);
    extremes_pub_.publish(extremes);
    marker_pub_.publish(markers);
  }

  // params
  std::string map_topic_, input_topic_, output_frame_;
  double novel_threshold_, cluster_tolerance_, max_extent_xy_;
  double map_voxel_leaf_, input_voxel_leaf_, tf_timeout_, min_range_;
  int min_cluster_size_, max_cluster_size_;

  // state
  std::mutex map_mutex_;
  bool map_ready_ = false;
  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> map_kdtree_;

  // ros
  ros::Subscriber map_sub_, input_sub_;
  ros::Publisher boxes_pub_, poses_pub_, cloud_pub_, marker_pub_, extremes_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
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
