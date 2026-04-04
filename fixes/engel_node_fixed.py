#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Engel Tespiti Node - v3 (Dinamik Tarama + Kacinma Verisi)

Yenilikler:
  - /steer_angle subscribe ederek tarama merkezini direksiyonla esler
  - Sol/Sag sektor mesafesi yayinlar (kacinma icin)
  - Engel acisi yayinlar (engelin hangi tarafta oldugu)
  - Tarama genisligi: +/- 15 derece (toplam 30 derece)
  - Merkez sektor: +/- 5 derece (fren karari icin)
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32
import math


class EngelTespitiNode:
    def __init__(self):
        rospy.init_node('engel_tespiti', anonymous=True)

        # --- Parametreler ---
        self.engel_mesafesi_limiti = 2.0  # metre - engel var/yok esigi
        self.scan_half_fov = 15.0         # derece - toplam tarama: +/- 15 = 30 derece
        self.merkez_half_fov = 5.0        # derece - merkez sektor: +/- 5 = 10 derece
        self.min_range = 0.3              # 30cm altindaki okumalar sahte
        self.steer_angle_deg = 0.0        # direksiyon acisi (derece)
        self.steer_scale = 0.5            # direksiyon -> lidar offset katsayisi

        # --- Publishers ---
        self.engel_pub = rospy.Publisher('/engel', Int32, queue_size=10)
        self.mesafe_pub = rospy.Publisher('/engel_distance', Float32, queue_size=10)
        self.engel_aci_pub = rospy.Publisher('/engel_angle', Float32, queue_size=10)
        self.sol_mesafe_pub = rospy.Publisher('/engel_sol_mesafe', Float32, queue_size=10)
        self.sag_mesafe_pub = rospy.Publisher('/engel_sag_mesafe', Float32, queue_size=10)

        # --- Subscribers ---
        self.scan_sub = rospy.Subscriber('/converted_scan', LaserScan, self.scan_callback)
        self.steer_sub = rospy.Subscriber('/steer_angle', Float32, self.steer_callback)

        rospy.loginfo("Engel Tespiti Node v3 Baslatildi (dinamik tarama)")
        rospy.loginfo(f"Mesafe Limiti: {self.engel_mesafesi_limiti}m, "
                      f"Tarama: +/- {self.scan_half_fov} derece, "
                      f"Merkez: +/- {self.merkez_half_fov} derece")

    def steer_callback(self, msg):
        """Direksiyon acisi callback (derece, sag: pozitif, sol: negatif)"""
        self.steer_angle_deg = msg.data

    def scan_callback(self, data):
        # Direksiyon acisina gore tarama merkezini kaydir
        # Negatif steer = sola donus -> tarama merkezi sola kayar
        center_offset_rad = math.radians(self.steer_angle_deg * self.steer_scale)

        half_fov_rad = math.radians(self.scan_half_fov)
        merkez_half_rad = math.radians(self.merkez_half_fov)

        angle_min = data.angle_min
        angle_increment = data.angle_increment

        # Sektor minimumlari
        min_mesafe = float('inf')       # Toplam minimum
        min_mesafe_aci = 0.0            # Minimum mesafenin acisi
        min_sol = float('inf')          # Sol sektor minimum
        min_sag = float('inf')          # Sag sektor minimum
        min_merkez = float('inf')       # Merkez sektor minimum

        for i, range_val in enumerate(data.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
            if range_val < self.min_range:
                continue

            # Bu noktanin acisi (radyan)
            current_angle = angle_min + (i * angle_increment)

            # -pi..pi araligina normalize et
            while current_angle > math.pi:
                current_angle -= 2 * math.pi
            while current_angle < -math.pi:
                current_angle += 2 * math.pi

            # Direksiyon offseti uygula - tarama merkezine gore bagil aci
            relative_angle = current_angle - center_offset_rad

            # Toplam FOV icerisinde mi?
            if -half_fov_rad <= relative_angle <= half_fov_rad:
                if range_val < min_mesafe:
                    min_mesafe = range_val
                    min_mesafe_aci = math.degrees(relative_angle)

                # Sol sektor (negatif acilar)
                if relative_angle < 0 and range_val < min_sol:
                    min_sol = range_val

                # Sag sektor (pozitif acilar)
                if relative_angle >= 0 and range_val < min_sag:
                    min_sag = range_val

                # Merkez sektor
                if -merkez_half_rad <= relative_angle <= merkez_half_rad:
                    if range_val < min_merkez:
                        min_merkez = range_val

        # Engel var/yok (merkez sektordeki engele gore karar ver)
        engel_durumu = 0
        if min_merkez < self.engel_mesafesi_limiti:
            engel_durumu = 1

        # Yakin engel varsa loglama (her engel icin degil, sadece merkezdeki)
        if engel_durumu == 1:
            rospy.loginfo(f"ENGEL! Merkez: {min_merkez:.2f}m | "
                          f"Sol: {min_sol:.2f}m | Sag: {min_sag:.2f}m | "
                          f"Steer offset: {self.steer_angle_deg:.1f} derece")

        # Yayinla
        self.engel_pub.publish(engel_durumu)
        self.mesafe_pub.publish(min_mesafe)
        self.engel_aci_pub.publish(min_mesafe_aci)
        self.sol_mesafe_pub.publish(min_sol)
        self.sag_mesafe_pub.publish(min_sag)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = EngelTespitiNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
