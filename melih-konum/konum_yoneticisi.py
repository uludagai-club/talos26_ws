#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import json
import math

class KonumYoneticisi:
    def __init__(self):
        # Node ismi
        rospy.init_node('konum_yoneticisi')

        # --- DURUM DEGISKENLERI ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.target_x = None
        self.target_y = None
        self.target_name = ""
        self.is_mission_active = False

        # --- ABONELIKLER ---
        # 1. KONUM (X, Y) -> Ground Truth (Simulasyon)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        
        # 2. Yaw -> IMU
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # 3. HEDEF -> Samed
        rospy.Subscriber('/hedef', String, self.hedef_callback)

        # --- YAYINCILAR ---
        self.pub_konum = rospy.Publisher('/konum', Pose2D, queue_size=10)
        self.pub_durum = rospy.Publisher('/gorev_durumu', String, queue_size=10)

        rospy.loginfo("Sistem Baslatildi.")

    def odom_callback(self, msg):
        """ Konum X, Y buradan alinir """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Konum verisi /konum topic'ine basılır
        self.publish_pose()
        
        if self.is_mission_active:
            self.check_distance()

    def imu_callback(self, msg):
        """ Yaw buradan alinir """
        q = msg.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def hedef_callback(self, msg):
        """ Samed'den gelen hedef """
        try:
            data = json.loads(msg.data)
            self.target_x = data['coordinates'][0]
            self.target_y = data['coordinates'][1]
            self.target_name = data.get('target_name', 'Bilinmeyen')
            self.is_mission_active = True
        except:
            pass

    def publish_pose(self):
        """ Sensor Fusion (Odom + IMU) ciktisini yayinla """
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = self.current_yaw 
        self.pub_konum.publish(pose)

    def check_distance(self):
        if self.target_x is None: return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.sqrt(dx**2 + dy**2)

        # 1.5 metre tolerans
        if dist < 1.5:
            rospy.logwarn(f"*** VARILDI: {self.target_name} ***")
            self.pub_durum.publish("VARILDI")
            self.is_mission_active = False
            self.target_x = None

    def start_monitoring(self):
        """ Terminal Arayuzu (1 Hz) """
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            # Ekrani temizle (Opsiyonel: print("\033c", end="") )
            
            if self.is_mission_active and self.target_x is not None:
                dx = self.target_x - self.current_x
                dy = self.target_y - self.current_y
                dist = math.sqrt(dx**2 + dy**2)
                
                print("-" * 40)
                print(f"HEDEF  : X={self.target_x:.2f}, Y={self.target_y:.2f}")
                print(f"MEVCUT KONUM  : X={self.current_x:.2f}, Y={self.current_y:.2f}")
                print(f"AÇI    : {math.degrees(self.current_yaw):.1f}°")
                print(f"KALAN  : {dist:.2f} metre")
                print("-" * 40)
            else:
                print(f"[BEKLEMEDE] Konum: {self.current_x:.2f}, {self.current_y:.2f}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = KonumYoneticisi()
        node.start_monitoring()
    except rospy.ROSInterruptException:
        pass