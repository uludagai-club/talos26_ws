#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32
import math

class EngelTespitiNode:
    def __init__(self):
        # Node başlatılıyor
        rospy.init_node('engel_tespiti', anonymous=True)

        # Publisher tanımları
        self.engel_pub = rospy.Publisher('/engel', Int32, queue_size=10)
        self.mesafe_pub = rospy.Publisher('/engel_distance', Float32, queue_size=10)

        # Subscriber tanımı (LaserScan verisi dinleniyor)
        # 2d_mapping_gt.launch dosyasından gördüğümüz kadarıyla topic adı /converted_scan
        self.scan_sub = rospy.Subscriber('/converted_scan', LaserScan, self.scan_callback)

        # Parametreler
        self.engel_mesafesi_limiti = 2.0  # metre
        self.on_aci_araligi = 30.0  # derece (sağ ve sol toplam tarama alanı +/- 30 derece)
        
        rospy.loginfo("Engel Tespiti Node Başlatıldı.")
        rospy.loginfo(f"Mesafe Limiti: {self.engel_mesafesi_limiti}m, Açı Aralığı: +/- {self.on_aci_araligi} derece")

    def scan_callback(self, data):
        """
        Lazer tarayıcı verilerini işler ve engel durumunu kontrol eder.
        """
        min_mesafe = float('inf')
        
        # Radyan cinsinden açı aralığı
        half_fov_rad = math.radians(self.on_aci_araligi)
        
        # Lazer verisinin açısal çözünürlüğü ve başlangıç açısı
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        
        # Ön bölgedeki verileri filtrele
        detected_indices = []
        
        for i, range_val in enumerate(data.ranges):
            # Geçersiz verileri (inf, nan) atla
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            # Bu ölçümün açısını hesapla
            current_angle = angle_min + (i * angle_increment)
            
            # Açıyı -pi ile +pi arasına normalize et (gerekirse)
            while current_angle > math.pi:
                current_angle -= 2 * math.pi
            while current_angle < -math.pi:
                current_angle += 2 * math.pi
            
            # Eğer açı bizim ilgilendiğimiz ön sektör içindeyse
            if -half_fov_rad <= current_angle <= half_fov_rad:
                if range_val < min_mesafe:
                    min_mesafe = range_val

        # Engel durumu belirle
        engel_durumu = 0
        if min_mesafe < self.engel_mesafesi_limiti:
            engel_durumu = 1
            rospy.loginfo(f"Araç önündeki engelleri algılıyor! Mesafe: {min_mesafe:.2f}m")
        
        # Sonuçları yayınla
        self.engel_pub.publish(engel_durumu)
        self.mesafe_pub.publish(min_mesafe)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EngelTespitiNode()
        node.run()
    except rospy.ROSInterruptException:
        pass