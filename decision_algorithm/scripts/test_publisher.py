#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test Publisher - Görüntü İşleme Simülatörü
Bu script görüntü işleme ekibini simüle eder.
"""

import rospy
from std_msgs.msg import String
import random

class VisionSimulator:
    def __init__(self):
        rospy.init_node('vision_simulator', anonymous=True)
        self.traffic_sign_pub = rospy.Publisher('/trafik_levha', String, queue_size=10)
        self.crosswalk_pub = rospy.Publisher('/yaya_gecidi', String, queue_size=10)
        rospy.loginfo("="*60)
        rospy.loginfo("Görüntü İşleme Simülatörü Başlatıldı")
        rospy.loginfo("="*60)
        rospy.sleep(1)
    
    def print_menu(self):
        print("\n" + "="*60)
        print("TEST SENARYOLARI")
        print("="*60)
        print("1. Normal Sürüş")
        print("2. Yaya Geçidi (10m -> 3m)")
        print("3. STOP Levhası (8m -> 2.5m)")
        print("4. Sola Dönüş")
        print("5. Sağa Dönüş")
        print("6. ACİL DURUM (2m)")
        print("7. Rastgele Test")
        print("0. Çıkış")
        print("="*60)
    
    def publish_traffic_sign(self, sign_type, distance, confidence):
        msg = String()
        msg.data = f"{sign_type},{distance},{confidence}"
        self.traffic_sign_pub.publish(msg)
        rospy.loginfo(f"📋 Levha: {sign_type}, {distance}m, {confidence}")
    
    def publish_crosswalk(self, detected, distance, confidence):
        msg = String()
        detected_str = "true" if detected else "false"
        msg.data = f"{detected_str},{distance},{confidence}"
        self.crosswalk_pub.publish(msg)
        rospy.loginfo(f"🚶 Yaya geçidi: {detected}, {distance}m, {confidence}")
    
    def scenario_1_normal(self):
        rospy.loginfo("\n🚗 SENARYO 1: Normal Sürüş")
        for i in range(10):
            self.publish_traffic_sign("none", 0, 0)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_2_crosswalk(self):
        rospy.loginfo("\n🚶 SENARYO 2: Yaya Geçidi")
        distances = [15.0, 12.0, 10.0, 8.0, 6.0, 4.0, 3.0, 2.5, 2.0]
        for dist in distances:
            rospy.loginfo(f"  → Mesafe: {dist}m")
            self.publish_traffic_sign("none", 0, 0)
            self.publish_crosswalk(True, dist, 0.95)
            rospy.sleep(1.5)
        rospy.loginfo("  → Duruldu, 5s bekleniyor...")
        for i in range(5):
            self.publish_traffic_sign("none", 0, 0)
            self.publish_crosswalk(True, 2.0, 0.95)
            rospy.sleep(1)
        rospy.loginfo("  → Geçiliyor...")
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(False, 0, 0)
        rospy.sleep(2)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_3_stop(self):
        rospy.loginfo("\n🛑 SENARYO 3: STOP Levhası")
        distances = [10.0, 8.0, 6.0, 4.0, 2.5, 2.0]
        for dist in distances:
            rospy.loginfo(f"  → STOP mesafe: {dist}m")
            self.publish_traffic_sign("stop", dist, 0.92)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        rospy.loginfo("  → Duruldu, 3s bekleniyor...")
        for i in range(3):
            self.publish_traffic_sign("stop", 2.0, 0.92)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        rospy.loginfo("  → Devam...")
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(False, 0, 0)
        rospy.sleep(2)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_4_turn_left(self):
        rospy.loginfo("\n↰ SENARYO 4: Sola Dönüş")
        distances = [10.0, 7.0, 5.0, 3.0]
        for dist in distances:
            self.publish_traffic_sign("turn_left", dist, 0.88)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        rospy.loginfo("  → Dönülüyor...")
        for i in range(3):
            self.publish_traffic_sign("turn_left", 2.0, 0.88)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(False, 0, 0)
        rospy.sleep(2)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_5_turn_right(self):
        rospy.loginfo("\n↱ SENARYO 5: Sağa Dönüş")
        distances = [10.0, 7.0, 5.0, 3.0]
        for dist in distances:
            self.publish_traffic_sign("turn_right", dist, 0.90)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        rospy.loginfo("  → Dönülüyor...")
        for i in range(3):
            self.publish_traffic_sign("turn_right", 2.0, 0.90)
            self.publish_crosswalk(False, 0, 0)
            rospy.sleep(1)
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(False, 0, 0)
        rospy.sleep(2)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_6_emergency(self):
        rospy.loginfo("\n🚨 SENARYO 6: ACİL DURUM")
        rospy.loginfo("  → Aniden 2m'de yaya geçidi!")
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(True, 2.0, 0.98)
        rospy.sleep(2)
        rospy.loginfo("  → Acil fren, bekleniyor...")
        for i in range(5):
            self.publish_traffic_sign("none", 0, 0)
            self.publish_crosswalk(True, 1.8, 0.98)
            rospy.sleep(1)
        rospy.loginfo("  → Güvenli, devam...")
        self.publish_traffic_sign("none", 0, 0)
        self.publish_crosswalk(False, 0, 0)
        rospy.sleep(2)
        rospy.loginfo("✓ Tamamlandı\n")
    
    def scenario_7_random(self):
        rospy.loginfo("\n🎲 SENARYO 7: Rastgele Test (Ctrl+C ile dur)")
        signs = ["none", "stop", "go", "turn_left", "turn_right"]
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            sign = random.choice(signs)
            sign_dist = random.uniform(2.0, 15.0)
            sign_conf = random.uniform(0.7, 0.99)
            crosswalk = random.choice([True, False, False, False])
            cross_dist = random.uniform(2.0, 15.0) if crosswalk else 0
            cross_conf = random.uniform(0.6, 0.99) if crosswalk else 0
            self.publish_traffic_sign(sign, sign_dist, sign_conf)
            self.publish_crosswalk(crosswalk, cross_dist, cross_conf)
            rate.sleep()
    
    def run(self):
        scenarios = {
            '1': self.scenario_1_normal,
            '2': self.scenario_2_crosswalk,
            '3': self.scenario_3_stop,
            '4': self.scenario_4_turn_left,
            '5': self.scenario_5_turn_right,
            '6': self.scenario_6_emergency,
            '7': self.scenario_7_random
        }
        
        while not rospy.is_shutdown():
            self.print_menu()
            try:
                choice = input("\nSenaryo seçin (0-7): ").strip()
                if choice == '0':
                    rospy.loginfo("Çıkılıyor...")
                    break
                if choice in scenarios:
                    scenarios[choice]()
                else:
                    rospy.logwarn("Geçersiz seçim!")
            except KeyboardInterrupt:
                rospy.loginfo("\nDurduruldu")
                break
            except Exception as e:
                rospy.logerr(f"Hata: {e}")

if __name__ == '__main__':
    try:
        simulator = VisionSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
