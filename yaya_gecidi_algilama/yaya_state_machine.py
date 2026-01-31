#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class YayaGecidiRobotu:
    def __init__(self):
        rospy.init_node('yaya_gecidi_node')
        
        # ROS Bağlantıları
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        
        # Durum Değişkenleri
        self.state = "YAKLASMA"  # Başlangıç durumu
        self.wait_start_time = None
        self.yaya_gecidi_tespit_edildi = False

        rospy.loginfo("Robot Hazır! Yaya geçidi aranıyor...")

    def camera_callback(self, msg):
        try:
            # 1. Görüntüyü Al
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = frame.shape

            # 2. İLGİ ALANI (ROI) BELİRLEME - Sadece yere bak!
            # Görüntünün üst %60'ını at (Duvarları görmesin)
            # Alt %40'lık kısma odaklan
            roi_img = frame[int(height*0.60):, :] 

            # 3. RENK FİLTRESİ (GÜNCELLENDİ)
            hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
            
            # Alt sınır: Parlaklığı (Value) 30'a kadar indirdik.
            # Böylece gölgedeki gri çizgiyi bile "beyaz" sayacak.
            lower_white = np.array([0, 0, 30])     
            upper_white = np.array([180, 100, 255])
            
            mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # 4. Beyaz Piksel Sayımı
            beyaz_piksel_sayisi = cv2.countNonZero(mask)
            
            # Ekrana Bilgi Yaz (Debug için)
            if beyaz_piksel_sayisi > 100:
                rospy.loginfo(f"Görülen Beyazlık: {beyaz_piksel_sayisi}")

            # 5. DURUM MAKİNESİ (Karar Verme Kısmı)
            
            # DURUM 1: YAKLAŞMA
            if self.state == "YAKLASMA":
                self.move_robot(0.15)  # İleri git
                
                # Eğer yerde yeterince beyazlık görürse DUR
                if beyaz_piksel_sayisi > 500:  # Eşik değeri
                    rospy.loginfo(">>> YAYA GECIDI GORDUM! DURUYORUM... <<<")
                    self.stop_robot()
                    self.state = "BEKLEME"
                    self.wait_start_time = rospy.Time.now()

            # DURUM 2: BEKLEME (3 Saniye)
            elif self.state == "BEKLEME":
                self.stop_robot()
                gecen_sure = rospy.Time.now() - self.wait_start_time
                
                # Ekrana geri sayım yazdıralım
                rospy.loginfo(f"Bekliyorum... {3 - gecen_sure.to_sec():.1f} sn")

                if gecen_sure.to_sec() > 3.0:
                    rospy.loginfo(">>> BEKLEME BITTI, GECIS BASLIYOR... <<<")
                    self.state = "GECIS"
                    self.wait_start_time = rospy.Time.now() # Geciş süresi için sıfırla

            # DURUM 3: GEÇİŞ (Yaya geçidinden uzaklaş)
            elif self.state == "GECIS":
                self.move_robot(0.15) # Tekrar hareket et
                # 5 saniye boyunca dümdüz git ki çizgileri tamamen geçsin
                gecen_sure = rospy.Time.now() - self.wait_start_time
                if gecen_sure.to_sec() > 5.0:
                    self.stop_robot()
                    rospy.loginfo("Gorev Tamamlandi.")

            # 6. Pencereleri Göster
            cv2.putText(frame, f"MOD: {self.state}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Kamera (Orjinal)", frame)
            cv2.imshow("Algilan (ROI Maske)", mask)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Hata olustu: {e}")

    def move_robot(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        yaya_robot = YayaGecidiRobotu()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
