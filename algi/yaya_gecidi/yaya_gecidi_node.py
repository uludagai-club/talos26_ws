#!/usr/bin/env python3
"""
Yaya Gecidi (Crosswalk) Algilama ROS Node'u — ADANMIS MODEL

MODELLER 20.06.2026/YAYA GEÇİDİ/best.pt — ozel 2-sinifli yaya gecidi
dedektoru ({0: crosswalk, 1: object}). traffic-node'daki levha modeli
yaya_gecidi'ni 26 sinifin biri olarak yakaliyordu; bu node ise yalnizca
yaya gecidine egitilmis adanmis modeli kullanir.

Cikti (yolov8_ros_node_fixed.py kalibi):
  - /yaya_gecidi/image_annotated (Image)  isaretlenmis goruntu (kayit/viz)
  - /yaya_gecidi/model           (String) "mesafe,offset" veya "none"

NOT: traffic-node'un yayinladigi bare /yaya_gecidi (karar.py tuketiyor)
EZILMEZ — bu node /yaya_gecidi/ namespace'ine yayinlar. Hangi modelin
karar'a beslenecegi ayri bir karardir.
"""
import os
import rospy
import cv2

# --- OpenCV 4.13 fix: copyMakeBorder float arguman hatasi (yolov8 node ile ayni) ---
_orig_copyMakeBorder = cv2.copyMakeBorder
def _fixed_copyMakeBorder(src, top, bottom, left, right, *args, **kwargs):
    return _orig_copyMakeBorder(src, int(top), int(bottom), int(left), int(right), *args, **kwargs)
cv2.copyMakeBorder = _fixed_copyMakeBorder

import threading
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

# Bbox yuksekliginden kaba mesafe proxy'si (yolov8 node ile ayni kalibrasyon)
MESAFE_K = 1280.0
LATERAL_SCALE = 3.0
# Sim renderinda yaya gecidi ~0.40-0.70 guven veriyor; 0.5 cok kati kaliyordu.
CONF_ESIK = 0.30

# Canlı parametreler: config/canli_params.yaml 'yaya_gecidi:' — restart'sız uygulanır
try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("yaya_gecidi", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[yaya_gecidi] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class YayaGecidiNode:
    def __init__(self):
        rospy.init_node('yaya_gecidi_node', anonymous=True)

        self.bridge = CvBridge()
        # Model yolu: container'da /app/models/best.pt mount edilir
        candidates = ['/app/models/best.pt',
                      os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'best.pt')]
        model_path = next((p for p in candidates if os.path.exists(p)), candidates[0])
        rospy.loginfo(f"Yaya gecidi modeli yukleniyor: {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo(f"Model siniflari: {self.model.names}")

        self.img_pub = rospy.Publisher('/yaya_gecidi/image_annotated', Image, queue_size=1)
        self.det_pub = rospy.Publisher('/yaya_gecidi/model', String, queue_size=1)
        self.sub = rospy.Subscriber('/cart/front_camera/image_raw', Image,
                                    self.image_callback, queue_size=1, buff_size=2 ** 24)

        self.show_gui = bool(os.environ.get('DISPLAY', ''))
        self._frame_lock = threading.Lock()
        self.frame_to_show = None
        rospy.loginfo("Yaya gecidi node hazir" + (" (GUI aktif)" if self.show_gui else " (DISPLAY yok)"))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge hata: {e}")
            return

        try:
            h, w = frame.shape[:2]
            results = self.model(frame, verbose=False)

            en_yakin = None
            en_yakin_mesafe = 99.0
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    cls_name = self.model.names.get(int(box.cls[0]), "")
                    if cls_name != 'crosswalk' or float(box.conf[0]) < CONF_ESIK:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0]
                    bbox_h = max(abs(float(y2) - float(y1)), 1.0)
                    mesafe = MESAFE_K / bbox_h
                    cx = float(x1 + x2) / 2.0
                    offset = (cx - w / 2.0) / (w / 2.0) * LATERAL_SCALE  # sol -, sag +
                    if mesafe < en_yakin_mesafe:
                        en_yakin_mesafe = mesafe
                        en_yakin = f"{mesafe:.1f},{offset:.1f}"

            self.det_pub.publish(en_yakin if en_yakin else "none")

            annotated = results[0].plot()
            with self._frame_lock:
                self.frame_to_show = annotated
            if self.img_pub.get_num_connections() > 0:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        except Exception as e:
            rospy.logerr(f"yaya_gecidi callback hata: {e}")

    def run(self):
        if not self.show_gui:
            rospy.spin()
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self._frame_lock:
                frame = self.frame_to_show
            if frame is not None:
                cv2.imshow("Yaya Gecidi Tespit", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    YayaGecidiNode().run()
