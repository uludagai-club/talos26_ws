#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import math

class KonumYoneticisi:
    def __init__(self):
        rospy.init_node('konum_yoneticisi')

        # --- DURUM DEGISKENLERI ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.target_x = None      
        self.target_y = None      
        self.target2_x = None     
        self.target2_y = None     
        
        self.is_mission_active = False

        # --- ABONELIKLER ---
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/hedef', String, self.hedef_callback)

        # --- YAYINCILAR ---
        self.pub_konum = rospy.Publisher('/konum', Pose2D, queue_size=10)
        # Hedef yöneticisinin beklediği kanal
        self.pub_durum = rospy.Publisher('/gorev_durumu', String, queue_size=10)

        rospy.loginfo("Konum Yoneticisi Calisiyor...")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # FIX: /imu yönelim yayınlamadığı için yaw 0'da kalıyordu (/konum.theta=0).
        # control.py gibi yaw'ı ground-truth Odometry orientation'ından türet.
        q = msg.pose.pose.orientation
        (_, _, self.current_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.publish_pose()
        
        if self.is_mission_active:
            self.check_distance()

    def imu_callback(self, msg):
        # NOT: yaw artık odom_callback'te ground-truth'tan alınıyor. /imu yönelim
        # vermediği için buradan yaw yazmak (0/identity) gerçek yaw'ı ezerdi → devre dışı.
        # q = msg.orientation
        # orientation_list = [q.x, q.y, q.z, q.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # self.current_yaw = yaw
        pass
        
    def hedef_callback(self, msg):
        """ Hedef Yoneticisinden gelen WP1|WP2 verisi """
        try:
            if not msg.data or '|' not in msg.data:
                return

            # hedef_yoneticisi artık 5 ileri-WP yayınlıyor (x,y,tip|...×5).
            # İlk iki WP yeterli: wp1=hedef, wp2=bakış yönü. Fazla alanları yok say.
            parts = msg.data.split('|')
            if len(parts) < 2:
                return
            wp1_str, wp2_str = parts[0], parts[1]
            
            # WP1 (Gidilecek ana nokta)
            wp1_data = wp1_str.split(',')
            new_x = float(wp1_data[0].strip())
            new_y = float(wp1_data[1].strip())

            # WP2 (Bakis acisi)
            wp2_data = wp2_str.split(',')
            new2_x = float(wp2_data[0].strip())
            new2_y = float(wp2_data[1].strip())

            # Sadece hedef gercekten degistiyse gorevi aktif et
            if (self.target_x is None) or (abs(new_x - self.target_x) > 0.01) or (abs(new_y - self.target_y) > 0.01):
                self.target_x = new_x
                self.target_y = new_y
                self.target2_x = new2_x
                self.target2_y = new2_y
                self.is_mission_active = True
            
        except Exception as e:
            rospy.logerr(f"Hedef ayristirma hatasi: {e}")

    def publish_pose(self):
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = self.current_yaw 
        self.pub_konum.publish(pose)

    def check_distance(self):
        if self.target_x is None: return

        dist = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)

        # TOLERANS: 1.2 metre (Hedef yoneticisindeki bakis mesafesiyle uyumlu)
        if dist < 1.2:
            rospy.logwarn(f">>> HEDEFE VARILDI: WP1 ({self.target_x}, {self.target_y})")
            
            # KRITIK DUZELTME: Kucuk harfle "varildi" gonderiyoruz
            msg = String()
            msg.data = "varildi"
            self.pub_durum.publish(msg)
            
            # Gorevi pasif yapıyoruz ki yeni hedef gelene kadar ust uste mesaj atmasın
            self.is_mission_active = False

    def start_monitoring(self):
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            if self.is_mission_active and self.target_x is not None:
                dist = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
                print(f"\n[TAKIP] WP1: {self.target_x:.1f}, {self.target_y:.1f}")
                print(f"[TAKIP] Kalan Mesafe: {dist:.2f} m")
            else:
                print(f"[BEKLEMEDE] Konum: {self.current_x:.2f}, {self.current_y:.2f}", end='\r')
            rate.sleep()

if __name__ == '__main__':
    try:
        KonumYoneticisi().start_monitoring()
    except rospy.ROSInterruptException:
        pass