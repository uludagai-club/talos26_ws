#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

class SafeZoneDetector:
    def __init__(self):
        # ROS düğümünü (node) başlat
        rospy.init_node('safe_zone_detector_node', anonymous=True)
        
        # YOLO modelini yükle (best.pt dosyanın TAM YOLUNU yazmayı unutma)
        self.model = YOLO('dosya_yolu/best.pt')
        
        # ROS ve OpenCV görüntüleri arasında çeviri yapan köprü
        self.bridge = CvBridge()
        
        # Gazebo kamerasından gelen görüntü topic'ine abone ol
        # Simülasyonundaki kamera topic adını buraya yaz (örn: /camera/image_raw veya /rrbot/camera1/image_raw)
        self.image_sub = rospy.Subscriber("/zed2/rgb/image_raw", Image, self.callback)
        
        # İşlenmiş görüntüleri yayınlayacağımız yeni topic
        self.image_pub = rospy.Publisher("/camera/safe_zone_detections", Image, queue_size=10)
        
        rospy.loginfo("Güvenli Alan Tespit Modeli Yüklendi ve Çalışıyor!")

    def callback(self, data):
        try:
            # 1. ROS Image mesajını OpenCV formatına (BGR8) çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 2. YOLO modelini çalıştır (Gereksiz logları kapatmak için verbose=False)
            results = self.model(cv_image, verbose=False)
            
            # 3. Modelin sonuçlarını (çizilmiş kutular, maskeler vb.) görüntü üzerine uygula
            annotated_frame = results[0].plot()
            
            # 4. İşlenmiş OpenCV resmini tekrar ROS Image mesajına çevir
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            
            # 5. Yeni görüntüyü ROS ağına yayınla
            self.image_pub.publish(output_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"Görüntü dönüştürme hatası: {e}")
        except Exception as e:
            rospy.logerr(f"Hata oluştu: {e}")

if __name__ == '__main__':
    try:
        detector = SafeZoneDetector()
        # Kodun kapanmamasını ve sürekli dinlemesini sağlar
        rospy.spin()
    except rospy.ROSInterruptException:
        pass