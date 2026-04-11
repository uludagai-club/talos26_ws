#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
from tf.transformations import euler_from_quaternion

# Global Degiskenler
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

def imu_callback(msg):
    global current_yaw
    # Quaternion -> Euler Donusumu
    q = msg.orientation
    orientation_list = [q.x, q.y, q.z, q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw

def odom_callback(msg):
    global current_x, current_y
    # Konumu Ground Truth'dan al ve guncelle
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

def main():
    global current_x, current_y, current_yaw
    rospy.init_node('arac_konum_node')
    
    # YAYINCI: RTAB-Map için Pose2D yerine Odometry kullanmak zorunludur (Zaman damgası için)
    pub = rospy.Publisher('/konum', Odometry, queue_size=10)
    
    # ABONELIKLER
    rospy.Subscriber('/base_pose_ground_truth', Odometry, odom_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    # TF Broadcaster (Odom -> Base_link baglantisi icin)
    br = tf.TransformBroadcaster()
    
    # HIZ AYARI: 50 Hz (Saniyede 50 kere dongu doner)
    rate = rospy.Rate(50)
    
    print("------------------------------------------------")
    print("Sistem Aktif: 50 Hz Sabit Konum Yayını")
    print("RTAB-Map Senkronizasyonu için Odometry kullanılıyor.")
    print("------------------------------------------------")
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        
        # 1. TF YAYINLA (RTAB-Map'in haritada nerede oldugunu anlaması icin sart)
        br.sendTransform(
            (current_x, current_y, 0),
            tf.transformations.quaternion_from_euler(0, 0, current_yaw),
            current_time,
            "base_link",  # Robotun gövdesi
            "odom"        # Dünya referansı
        )
        
        # 2. ODOMETRY MESAJINI OLUSTUR
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time # ISTE KRITIK NOKTA: Zaman damgası
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Konum bilgilerini doldur
        odom_msg.pose.pose.position.x = current_x
        odom_msg.pose.pose.position.y = current_y
        
        # Açı bilgisini Quaternion olarak doldur
        q = tf.transformations.quaternion_from_euler(0, 0, current_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # MESAJI YAYINLA
        pub.publish(odom_msg)
        
        # 50 Hz hizi korumak icin bekle
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass