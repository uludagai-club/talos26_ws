#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

class GtTfBroadcaster:
    def __init__(self):
        rospy.init_node('gt_tf_broadcaster')

        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        self.br = tf.TransformBroadcaster()

        rospy.loginfo("Ground Truth TF Broadcaster Started")

    def odom_callback(self, msg):
        # Use the timestamp from the message to ensure synchronization with simulation time
        current_time = msg.header.stamp

        # Helper to broadcast transform
        def broadcast_tf(translation, rotation, time, child, parent):
            self.br.sendTransform(
                translation,
                rotation,
                time,
                child,
                parent
            )

        pose = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # Broadcast map -> odom (Static Identity for now, assuming perfect localization or no drift in GT)
        # In a real scenario, this would be the difference between map and odom, but for GT mapping we can assume they align or odom is perfect.
        # However, slam_toolbox might publish map->odom. If we want to force GT, we usually pretend odom IS map or publish map->odom as identity.
        # The user requested: map -> odom -> base_link.
        
        # 1. map -> odom
        # We publish an identity transform for map -> odom. 
        # CAUTION: If slam_toolbox also publishes this, it will conflict. 
        # The user's prompt says: "Algoritma kendi lokalizasyonunu hesaplamak yerine gelen TF verisine (Ground Truth) güvenerek sadece haritayı 'boyamalı'."
        # This implies we dictate the location.
        broadcast_tf(
            (0, 0, 0),
            (0, 0, 0, 1),
            current_time,
            "odom",
            "map"
        )

        # 2. odom -> base_link (chassis)
        # The robot URDF uses "chassis" as the base, not "base_link".
        # We use the child_frame_id from message if valid, else default to chassis.
        child_frame = msg.child_frame_id if msg.child_frame_id else "chassis"
        
        broadcast_tf(
            (pose.x, pose.y, pose.z),
            (orient.x, orient.y, orient.z, orient.w),
            current_time,
            child_frame,
            "odom"
        )

if __name__ == '__main__':
    try:
        GtTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
