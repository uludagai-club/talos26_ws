#!/usr/bin/env python3
import os
import rospy
import cv2

# --- OpenCV 4.13 fix: copyMakeBorder float arguman hatasi ---
# (lane_follow_node.py / yolov8_ros_node_fixed.py ile ayni patch — YOLO letterbox float deger gonderiyor)
_orig_copyMakeBorder = cv2.copyMakeBorder
def _fixed_copyMakeBorder(src, top, bottom, left, right, *args, **kwargs):
    return _orig_copyMakeBorder(src, int(top), int(bottom), int(left), int(right), *args, **kwargs)
cv2.copyMakeBorder = _fixed_copyMakeBorder

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# /line uretimi (lane_follow_node.py ile ayni Kp ve clamp)
DIRECT_CLASS_NAME = 'direct'
LINE_KP = 0.12
LINE_CLAMP_DEG = 30.0
MIN_CONF = 0.5

class SafeZoneDetector:
    def __init__(self):
        # ROS düğümünü (node) başlat
        rospy.init_node('safe_zone_detector_node', anonymous=True)
        
        # YOLO modelini yükle (docker-compose bind-mount: prototip-1/best.pt -> /app/models/best.pt)
        self.model = YOLO('/app/models/best.pt')
        rospy.loginfo(f"[DEBUG] Model class names: {self.model.names}")
        rospy.loginfo(f"[DEBUG] Model task: {getattr(self.model, 'task', 'unknown')}")
        self._first_frame_logged = False

        # ROS ve OpenCV görüntüleri arasında çeviri yapan köprü
        self.bridge = CvBridge()
        
        # Gazebo kamerasından gelen görüntü topic'ine abone ol
        # Simülasyonundaki kamera topic adını buraya yaz (örn: /camera/image_raw veya /rrbot/camera1/image_raw)
        self.image_sub = rospy.Subscriber("/zed2/rgb/image_raw", Image, self.callback)
        
        # İşlenmiş görüntüleri yayınlayacağımız yeni topic
        self.image_pub = rospy.Publisher("/camera/safe_zone_detections", Image, queue_size=10)

        # /line yayini — lane_follow_node.py ile birebir kontrat (control.py bunu okur)
        self.line_pub = rospy.Publisher('/line', Float32, queue_size=10)

        # GUI (lane_follow_node.py / yolov8_ros_node_fixed.py ile ayni kalip)
        self.show_gui = bool(os.environ.get('DISPLAY'))
        self.frame_to_show = None

        rospy.loginfo("Güvenli Alan Tespit Modeli Yüklendi ve Çalışıyor!")

    def callback(self, data):
        try:
            # 1. ROS Image mesajını OpenCV formatına (BGR8) çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 2. YOLO modelini çalıştır (Gereksiz logları kapatmak için verbose=False)
            results = self.model(cv_image, verbose=False)

            # --- DEBUG: ilk birkaç frame'de model çıktısının yapısını dök ---
            if not self._first_frame_logged:
                r0 = results[0]
                h, w = cv_image.shape[:2]
                n_boxes = 0 if r0.boxes is None else len(r0.boxes)
                has_masks = r0.masks is not None
                n_masks = 0 if not has_masks else len(r0.masks)
                rospy.loginfo(f"[DEBUG] Frame {w}x{h} | boxes={n_boxes} | masks={n_masks}")
                if n_boxes > 0:
                    for i, box in enumerate(r0.boxes[:8]):
                        cls_id = int(box.cls[0])
                        cls_name = self.model.names.get(cls_id, "?")
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                        rospy.loginfo(f"[DEBUG]   box[{i}] cls={cls_name}({cls_id}) conf={conf:.2f} xyxy=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})")
                if has_masks and n_masks > 0:
                    m0 = r0.masks[0]
                    mshape = tuple(m0.data.shape) if hasattr(m0, 'data') else "?"
                    rospy.loginfo(f"[DEBUG]   mask[0] shape={mshape}")
                self._first_frame_logged = True

            # 3. /line uretimi: en yuksek conf'lu 'direct' bbox'inin merkez x'i
            #    -> frame ortasindan offset -> Kp ile derece -> ±30°'ye clamp.
            #    Lane node'unun /line kontrati ile birebir uyumlu (control.py degismeden okuyabilir).
            r0 = results[0]
            best_direct_cx = None
            best_direct_conf = 0.0
            if r0.boxes is not None:
                for box in r0.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, "?")
                    if cls_name != DIRECT_CLASS_NAME:
                        continue
                    conf = float(box.conf[0])
                    if conf < MIN_CONF or conf < best_direct_conf:
                        continue
                    x1, _, x2, _ = [float(v) for v in box.xyxy[0]]
                    best_direct_cx = 0.5 * (x1 + x2)
                    best_direct_conf = conf

            h, w = cv_image.shape[:2]
            offset = None
            angle = None
            if best_direct_cx is not None:
                offset = best_direct_cx - (w / 2.0)
                angle = max(min(offset * LINE_KP, LINE_CLAMP_DEG), -LINE_CLAMP_DEG)
                self.line_pub.publish(Float32(float(angle)))

            # 4. Modelin sonuçlarını (çizilmiş kutular, maskeler vb.) görüntü üzerine uygula
            annotated_frame = results[0].plot()

            # 5. Overlay: vehicle_center (kirmizi), direct_center (sari) ve metrikler
            vc = w // 2
            cv2.line(annotated_frame, (vc, h), (vc, h - 200), (0, 0, 255), 2)
            if best_direct_cx is not None:
                dx = int(best_direct_cx)
                cv2.line(annotated_frame, (dx, h), (dx, h - 200), (0, 255, 255), 3)
                cv2.putText(annotated_frame, f"Offset: {offset:+.0f} px", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(annotated_frame, f"Angle:  {angle:+.2f} deg", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Conf:   {best_direct_conf:.2f}", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            else:
                cv2.putText(annotated_frame, "direct: NONE", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.frame_to_show = annotated_frame

            # 6. İşlenmiş OpenCV resmini tekrar ROS Image mesajına çevir + yayınla
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(output_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"Görüntü dönüştürme hatası: {e}")
        except Exception as e:
            rospy.logerr(f"Hata oluştu: {e}")

    def run(self):
        """GUI varsa 'Safe Zone' penceresini ana thread'de gosterir,
        yoksa duz rospy.spin() ile calisir (lane_follow_node.py ile ayni kalip)."""
        if not self.show_gui:
            rospy.spin()
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.frame_to_show is not None:
                cv2.imshow("Safe Zone", self.frame_to_show)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        detector = SafeZoneDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass