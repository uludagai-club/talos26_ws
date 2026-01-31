#!/usr/bin/env python3
"""
Ground Truth TF Broadcaster for 2D SLAM

This node creates the complete TF tree for SLAM:
    map -> world -> chassis (-> velodyne, etc.)
    
Also publishes a planar laser_2d frame for gmapping compatibility.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft
import math


class GtTfBroadcaster:
    def __init__(self):
        rospy.init_node('gt_tf_broadcaster')

        # TF2 broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        
        # Track if we've received odometry
        self.received_odom = False
        
        # Publish static transform for planar laser frame
        self.publish_static_laser_frame()
        
        # Subscribe to ground truth odometry
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback, queue_size=1)
        
        rospy.loginfo("Ground Truth TF Broadcaster Started")
        rospy.loginfo("Will publish: map -> world -> chassis")
        rospy.loginfo("Also publishing: chassis -> laser_2d (planar frame for gmapping)")

    def publish_static_laser_frame(self):
        """Publish a static transform for a planar 2D laser frame"""
        # Create a planar laser frame at the same position as velodyne but perfectly horizontal
        # This compensates for the 0.02 radian pitch in the URDF
        laser_frame = TransformStamped()
        laser_frame.header.stamp = rospy.Time.now()
        laser_frame.header.frame_id = "chassis"
        laser_frame.child_frame_id = "laser_2d"
        
        # Same position as lidar_link: xyz="0.9 0 0.92" + velodyne offset
        # lidar_link is at (0.9, 0, 0.92) with pitch 0.02 rad
        # velodyne_base_link is at (0, 0, 0.05) relative to lidar_link
        # velodyne is at (0, 0, 0.0377) relative to velodyne_base_link
        # Total Z offset: 0.92 + 0.05 + 0.0377 = 1.0077
        laser_frame.transform.translation.x = 0.9
        laser_frame.transform.translation.y = 0.0
        laser_frame.transform.translation.z = 1.0
        
        # No rotation - keep it perfectly horizontal (planar)
        laser_frame.transform.rotation.x = 0.0
        laser_frame.transform.rotation.y = 0.0
        laser_frame.transform.rotation.z = 0.0
        laser_frame.transform.rotation.w = 1.0
        
        self.static_br.sendTransform(laser_frame)

    def odom_callback(self, msg):
        """Convert odometry to TF transforms"""
        stamp = msg.header.stamp
        
        # Get the frame names from the odometry message
        parent_frame = msg.header.frame_id if msg.header.frame_id else "world"
        child_frame = msg.child_frame_id if msg.child_frame_id else "chassis"
        
        if not self.received_odom:
            rospy.loginfo(f"First odometry received: {parent_frame} -> {child_frame}")
            self.received_odom = True
        
        # 1. Publish world -> chassis (from P3D odometry data)
        world_to_chassis = TransformStamped()
        world_to_chassis.header.stamp = stamp
        world_to_chassis.header.frame_id = parent_frame  # "world"
        world_to_chassis.child_frame_id = child_frame    # "chassis"
        world_to_chassis.transform.translation.x = msg.pose.pose.position.x
        world_to_chassis.transform.translation.y = msg.pose.pose.position.y
        world_to_chassis.transform.translation.z = msg.pose.pose.position.z
        world_to_chassis.transform.rotation = msg.pose.pose.orientation
        
        # 2. Publish map -> world (identity transform)
        # This connects SLAM's map frame to the simulation's world frame
        map_to_world = TransformStamped()
        map_to_world.header.stamp = stamp
        map_to_world.header.frame_id = "map"
        map_to_world.child_frame_id = parent_frame  # "world"
        map_to_world.transform.translation.x = 0.0
        map_to_world.transform.translation.y = 0.0
        map_to_world.transform.translation.z = 0.0
        map_to_world.transform.rotation.x = 0.0
        map_to_world.transform.rotation.y = 0.0
        map_to_world.transform.rotation.z = 0.0
        map_to_world.transform.rotation.w = 1.0
        
        # Publish both transforms
        self.br.sendTransform([map_to_world, world_to_chassis])


if __name__ == '__main__':
    try:
        node = GtTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
