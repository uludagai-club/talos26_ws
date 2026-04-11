#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    br = tf.TransformBroadcaster()
    
    # Odometri mesajından pozisyonu (x, y, z) al
    pos = (msg.pose.pose.position.x, 
           msg.pose.pose.position.y, 
           msg.pose.pose.position.z)
    
    # Odometri mesajından yönelimi (quaternion) al
    ori = (msg.pose.pose.orientation.x, 
           msg.pose.pose.orientation.y, 
           msg.pose.pose.orientation.z, 
           msg.pose.pose.orientation.w)
    
    # TF Köprüsünü Yayınla! (Child: base_link, Parent: odom)
    br.sendTransform(pos, ori, rospy.Time.now(), "base_link", "odom")

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    rospy.loginfo("TF Koprusu Kuruldu: odom -> base_link yayinlaniyor...")
    
    # Gazebo'dan gelen kusursuz odometri verisini dinle
    rospy.Subscriber('/base_pose_ground_truth', Odometry, odom_callback)
    rospy.spin()