#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string pcd_path, frame_id, topic;
  pnh.param<std::string>("pcd_path", pcd_path, std::string(std::getenv("HOME")) + "/talos_maps/clean.pcd");
  pnh.param<std::string>("frame_id", frame_id, "map");
  pnh.param<std::string>("topic", topic, "/map_cloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) < 0)
  {
    ROS_FATAL("map_publisher: failed to load PCD: %s", pcd_path.c_str());
    return 1;
  }
  ROS_INFO("map_publisher: loaded %zu points from %s", cloud->size(), pcd_path.c_str());

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
  pub.publish(msg);
  ROS_INFO("map_publisher: published latched on %s (frame=%s)", topic.c_str(), frame_id.c_str());

  ros::spin();
  return 0;
}
