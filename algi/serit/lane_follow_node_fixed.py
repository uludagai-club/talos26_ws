#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TALOS Lane Detection Node

Şerit algılama ve viraj tespiti.
Genişletilmiş topic'ler ile şerit tabanlı kontrol sistemi desteği.

Topic'ler:
    /line              - Direksiyon açısı (Float32)
    /lane/left_line    - Sol çizgi X pozisyonu (Float32)
    /lane/right_line   - Sağ çizgi X pozisyonu (Float32)
    /lane/center_offset- Merkez sapması piksel (Float32)
    /lane/confidence   - Algılama güvenilirliği 0-1 (Float32)
    /lane/turn_type    - Viraj tipi: "left_90", "right_90", "curve", "none" (String)
"""

import rospy
import os
import cv2
import numpy as np

# --- OpenCV 4.13 fix: copyMakeBorder float arguman hatasi ---
# (fixes/yolov8_ros_node_fixed.py ile ayni patch — YOLO letterbox float deger gonderiyor)
_orig_copyMakeBorder = cv2.copyMakeBorder
def _fixed_copyMakeBorder(src, top, bottom, left, right, *args, **kwargs):
    return _orig_copyMakeBorder(src, int(top), int(bottom), int(left), int(right), *args, **kwargs)
cv2.copyMakeBorder = _fixed_copyMakeBorder

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime
from collections import deque

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER — hepsi burada
#   (canlı: config/canli_params.yaml 'serit:' — restart'sız uygulanır)
# ════════════════════════════════════════════════════════════════════════
KP                     = 0.12   # piksel offset → direksiyon açısı kazancı
MODEL_CONF             = 0.6    # YOLO tespit güven eşiği
STEER_CLAMP_DEG        = 30.0   # /line çıkış açısı kelepçesi (±derece)
CONF_VIRAJ_ESIK        = 0.3    # bu güvenin altında viraj analizi yapılmaz
CONF_CURVE_ESIK        = 0.6    # bu güvenin altı "curve" sayılır
TURN_COOLDOWN_FRAMES   = 20     # viraj tespiti sonrası bekleme (kare)
NO_DETECT_CURVE_FRAMES = 10     # bu kadar kare tespitsiz → "curve"
SLOPE_SPREAD_MIN_PX    = 50     # 90° viraj için çizgi yayılım eşiği (piksel)

try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("serit", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[lane_follower] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower_node', anonymous=True)

        # ==== PUBLISHERS ====
        # Mevcut (geriye uyumluluk)
        self.pub = rospy.Publisher('/line', Float32, queue_size=10)
        # FIX(kayit): isaretlenmis goruntu topic'i — diger perception node'lari gibi
        # (/yolov8/image_annotated kalibi) kayit/viz icin pencereye bagimli olmadan.
        self.annotated_pub = rospy.Publisher('/lane/image_annotated', Image, queue_size=1)
        self.show_gui = bool(os.environ.get('DISPLAY', ''))
        self.vehicle_center_pub = rospy.Publisher('/vehicle_center', Float32, queue_size=10)
        self.road_center_pub = rospy.Publisher('/road_center', Float32, queue_size=10)
        self.offset_pub = rospy.Publisher('/lane_offset', Float32, queue_size=10)

        # Yeni genişletilmiş topic'ler
        self.left_line_pub = rospy.Publisher('/lane/left_line', Float32, queue_size=10)
        self.right_line_pub = rospy.Publisher('/lane/right_line', Float32, queue_size=10)
        self.center_offset_pub = rospy.Publisher('/lane/center_offset', Float32, queue_size=10)
        self.confidence_pub = rospy.Publisher('/lane/confidence', Float32, queue_size=10)
        self.turn_type_pub = rospy.Publisher('/lane/turn_type', String, queue_size=10)
        self.turn_detected_pub = rospy.Publisher('/lane/turn_detected', Bool, queue_size=10)

        # ==== MODEL ====
        # Model yolu hem repo (lane/scripts/ -> ../models) hem container (/app/ -> ./models)
        # yerlesimlerinde bulunsun diye birden fazla aday denenir.
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_candidates = [
            os.path.join(current_dir, '..', 'models', 'best.pt'),  # repo: lane/scripts/ -> lane/models/
            os.path.join(current_dir, 'models', 'best.pt'),        # container: /app/ -> /app/models/
            '/models/best.pt',
        ]
        model_path = next((p for p in model_candidates if os.path.exists(p)), model_candidates[0])
        rospy.loginfo(f"Lane modeli yukleniyor: {model_path}")
        self.model = YOLO(model_path)

        # ==== ROS ====
        self.bridge = CvBridge()
        rospy.Subscriber(
            "/cart/front_camera/image_raw/compressed",
            CompressedImage,
            self.callback
        )

        # ==== PARAMETRELER üst blokta (AYARLANABİLİR PARAMETRELER) ====
        self.frame_to_show = None
        self.log = open("lane_log.txt", "w")

        # ==== VİRAJ ALGILAMA ====
        self.confidence_history = deque(maxlen=10)
        self.left_slope_history = deque(maxlen=5)
        self.right_slope_history = deque(maxlen=5)
        self.last_valid_left = None
        self.last_valid_right = None
        self.frames_without_detection = 0
        self.turn_cooldown = 0

        rospy.loginfo("Lane follower hazır (genişletilmiş topic'ler aktif).")

    def calculate_confidence(self, left_points, right_points, total_boxes):
        """Algılama güvenilirliği hesapla (0-1 arası)"""
        if total_boxes == 0:
            return 0.0

        # Her iki çizgi de varsa yüksek güven
        if len(left_points) > 0 and len(right_points) > 0:
            base_conf = 0.8
        elif len(left_points) > 0 or len(right_points) > 0:
            base_conf = 0.5
        else:
            return 0.0

        # Daha fazla nokta = daha yüksek güven
        point_bonus = min(0.2, (len(left_points) + len(right_points)) * 0.05)

        return min(1.0, base_conf + point_bonus)

    def detect_turn_type(self, left_points, right_points, frame_width, confidence):
        """Viraj tipini algıla"""
        # Cooldown kontrolü
        if self.turn_cooldown > 0:
            self.turn_cooldown -= 1
            return "none"

        # Düşük güvende viraj algılama yapma
        if confidence < CONF_VIRAJ_ESIK:
            self.frames_without_detection += 1
            # Uzun süre algılama yoksa potansiyel keskin viraj
            if self.frames_without_detection > NO_DETECT_CURVE_FRAMES:
                return "curve"
            return "none"

        self.frames_without_detection = 0

        # Eğim analizi için yeterli veri yok
        if len(left_points) < 2 and len(right_points) < 2:
            return "none"

        # Çizgilerin eğimini hesapla (basit yaklaşım)
        center = frame_width // 2

        # Sol çizgi analizi
        if len(left_points) >= 2:
            left_spread = max(left_points) - min(left_points)
            left_avg = np.mean(left_points)
            # Sol çizgi merkeze çok yaklaşıyorsa sağa dönüş
            if left_avg > center * 0.7 and left_spread > SLOPE_SPREAD_MIN_PX:
                self.turn_cooldown = TURN_COOLDOWN_FRAMES
                return "right_90"

        # Sağ çizgi analizi
        if len(right_points) >= 2:
            right_spread = max(right_points) - min(right_points)
            right_avg = np.mean(right_points)
            # Sağ çizgi merkeze çok yaklaşıyorsa sola dönüş
            if right_avg < center * 1.3 and right_spread > SLOPE_SPREAD_MIN_PX:
                self.turn_cooldown = TURN_COOLDOWN_FRAMES
                return "left_90"

        # Genel eğri yol kontrolü
        if confidence < CONF_CURVE_ESIK:
            return "curve"

        return "none"

    def callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = frame.shape
            vehicle_center = w // 2

            results = self.model.predict(frame, conf=MODEL_CONF, verbose=False)

            left_points = []
            right_points = []
            left_y_points = []
            right_y_points = []

            draw = frame.copy()

            # YOLO sonuç kontrolü
            total_boxes = 0
            if len(results) > 0 and len(results[0].boxes) > 0:
                total_boxes = len(results[0].boxes)

                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                    # Şeridin yere değdiği alt merkez noktası
                    bottom_x = (x1 + x2) // 2
                    bottom_y = y2

                    # Çizimler
                    cv2.rectangle(draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(draw, (bottom_x, bottom_y), 5, (255, 0, 0), -1)

                    # Sol / Sağ ayır
                    if bottom_x < vehicle_center:
                        left_points.append(bottom_x)
                        left_y_points.append(bottom_y)
                    else:
                        right_points.append(bottom_x)
                        right_y_points.append(bottom_y)

            # Güvenilirlik hesapla
            confidence = self.calculate_confidence(left_points, right_points, total_boxes)
            self.confidence_history.append(confidence)
            avg_confidence = np.mean(self.confidence_history)

            # Viraj tipi algıla
            turn_type = self.detect_turn_type(left_points, right_points, w, avg_confidence)
            turn_detected = turn_type != "none"

            # Topic'leri yayınla
            self.confidence_pub.publish(Float32(avg_confidence))
            self.turn_type_pub.publish(String(turn_type))
            self.turn_detected_pub.publish(Bool(turn_detected))

            # Sol/sağ çizgi pozisyonları
            if len(left_points) > 0:
                left_x = int(np.mean(left_points))
                self.left_line_pub.publish(Float32(left_x))
                self.last_valid_left = left_x
            elif self.last_valid_left is not None:
                self.left_line_pub.publish(Float32(self.last_valid_left))

            if len(right_points) > 0:
                right_x = int(np.mean(right_points))
                self.right_line_pub.publish(Float32(right_x))
                self.last_valid_right = right_x
            elif self.last_valid_right is not None:
                self.right_line_pub.publish(Float32(self.last_valid_right))

            # Her iki çizgi de varsa merkez hesapla
            if len(left_points) > 0 and len(right_points) > 0:
                left_x = int(np.mean(left_points))
                right_x = int(np.mean(right_points))

                road_center = (left_x + right_x) // 2
                offset = road_center - vehicle_center
                angle = offset * KP
                angle = max(min(angle, STEER_CLAMP_DEG), -STEER_CLAMP_DEG)

                # Publish (mevcut + yeni)
                self.pub.publish(float(angle))
                self.offset_pub.publish(float(offset))
                self.center_offset_pub.publish(Float32(offset))
                self.vehicle_center_pub.publish(Float32(vehicle_center))
                self.road_center_pub.publish(Float32(road_center))

                # Log
                self.log.write(f"{datetime.now()} | Offset: {offset} | Angle: {angle} | Conf: {avg_confidence:.2f} | Turn: {turn_type}\n")

                # Çizimler
                cv2.line(draw, (vehicle_center, h), (vehicle_center, h-150), (0, 0, 255), 3)
                cv2.line(draw, (left_x, h), (left_x, h-150), (255, 255, 0), 3)
                cv2.line(draw, (right_x, h), (right_x, h-150), (255, 255, 0), 3)
                cv2.line(draw, (road_center, h), (road_center, h-150), (255, 0, 0), 4)

                cv2.putText(draw, f"Offset: {offset}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(draw, f"Angle: {angle:.2f}", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Güvenilirlik ve viraj bilgisi göster
            conf_color = (0, 255, 0) if avg_confidence > 0.6 else (0, 165, 255) if avg_confidence > 0.3 else (0, 0, 255)
            cv2.putText(draw, f"Conf: {avg_confidence:.2f}", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, conf_color, 2)

            if turn_detected:
                cv2.putText(draw, f"TURN: {turn_type}", (20, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.frame_to_show = draw
            if self.annotated_pub.get_num_connections() > 0:
                self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(draw, "bgr8"))

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        # DISPLAY yoksa pencere acmadan sadece topic yayini (/lane/image_annotated)
        if not self.show_gui:
            rospy.spin()
            self.log.close()
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.frame_to_show is not None:
                cv2.imshow("Lane Follow", self.frame_to_show)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()

        self.log.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = LaneFollower()
    node.run()
