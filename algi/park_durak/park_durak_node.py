#!/usr/bin/env python3
"""
Park + Durak Alani Algilama ROS Node'u

MODELLER 20.06.2026/PARK ALANI VE DURAK ALANI altindaki parktespit.py ve
duraktespit.py'nin HSV mavi-renk + kontur mantigini ROS node'una tasir.
Gercek aracta da ayni kamera akisi (/cart/front_camera/image_raw) uzerinden
calisir; webcam yerine ROS kamerasini dinler.

Ciktilar (yolov8_ros_node_fixed.py ile ayni kalip):
  - /park_alani            (String)  "mesafe,offset" veya "none"
  - /durak_alani           (String)  "mesafe,offset" veya "none"
  - /park_durak/image_annotated (Image) isaretlenmis goruntu (kayit/viz icin)

offset isaretli yayinlanir (sol negatif / sag pozitif) ki downstream manevra
hangi yone donecegini bilebilsin. mesafe bbox yuksekliginden kaba bir
proxy'dir (yere yatik alanlarda perspektif nedeniyle mutlak deger guvenilmez;
yalnizca "en yakin" siralamasi icin kullanilir - dogru mesafe homografi ister).

DISPLAY varsa 'Park/Durak Tespit' cv2 penceresi acilir (traffic-node kalibi).
"""
import os
import threading
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Bbox yuksekliginden kaba mesafe proxy'si (yolov8 node ile ayni kalibrasyon)
MESAFE_K = 1280.0
# Yatay offset metre olcegi (goruntu yari-genisligi ~ +-3 m varsayimi)
LATERAL_SCALE = 3.0
# Morfoloji kernel'i bir kez tahsis edilir (her callback'te yeniden yaratmamak icin)
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# Park: parktespit.py degerleri (daha doygun mavi, buyuk alan)
PARK_LOWER = np.array([90, 100, 100])
PARK_UPPER = np.array([130, 255, 255])
PARK_MIN_AREA = 2000

# Durak: duraktespit.py degerleri (acik mavi, daha kucuk alan)
DURAK_LOWER = np.array([90, 80, 80])
DURAK_UPPER = np.array([130, 255, 255])
DURAK_MIN_AREA = 1000

# Canlı parametreler: config/canli_params.yaml 'park_durak:' — restart'sız uygulanır
# (HSV eşikleri YAML'da [H, S, V] listesi olarak yazılır; izleyici np.array'e çevirir)
try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("park_durak", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[park_durak] canli_params yok, statik parametreler: {_canli_e}", flush=True)


def _mesafe_offset(x, y, w, h, img_w, img_h):
    """Bbox'tan kaba (mesafe, isaretli yatay offset) tahmini - karar_node uyumlu."""
    bbox_h = max(float(h), 1.0)
    mesafe = MESAFE_K / bbox_h
    cx = x + w / 2.0
    x_offset = (cx - img_w / 2.0) / (img_w / 2.0) * LATERAL_SCALE  # sol -, sag +
    return mesafe, x_offset


class ParkDurakNode:
    def __init__(self):
        rospy.init_node('park_durak_node', anonymous=True)
        rospy.loginfo("Park/Durak node baslatildi")

        self.bridge = CvBridge()
        self.park_pub = rospy.Publisher('/park_alani', String, queue_size=1)
        self.durak_pub = rospy.Publisher('/durak_alani', String, queue_size=1)
        self.img_pub = rospy.Publisher('/park_durak/image_annotated', Image, queue_size=1)
        # buff_size: kamera frame'i varsayilan 64KB buffer'a sigmaz; kucuk buff_size
        # + queue_size=1 eski frame birikmesine yol acar -> 16MB ver.
        self.sub = rospy.Subscriber('/cart/front_camera/image_raw', Image,
                                    self.image_callback, queue_size=1, buff_size=2 ** 24)

        self.show_gui = bool(os.environ.get('DISPLAY', ''))
        self._frame_lock = threading.Lock()
        self.frame_to_show = None
        if self.show_gui:
            rospy.loginfo("GUI aktif: 'Park/Durak Tespit' penceresi acilacak")
        else:
            rospy.loginfo("DISPLAY yok - sadece topic yayini")

    def _detect(self, hsv, lower, upper, min_area, morph):
        """Onceden hesaplanmis HSV uzerinde mavi maske + kontur;
        (mesafe, offset, bbox) listesi doner. cvtColor cagirmaz - callback bir
        kez hesaplayip paylasir."""
        mask = cv2.inRange(hsv, lower, upper)
        if morph:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = hsv.shape[:2]
        dets = []
        for cnt in contours:
            if cv2.contourArea(cnt) >= min_area:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                mesafe, offset = _mesafe_offset(bx, by, bw, bh, w, h)
                dets.append((mesafe, offset, (bx, by, bw, bh)))
        return dets

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge hata: {e}")
            return

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            annotated = frame.copy()

            # --- Park (parktespit.py: morfolojik temizlik var) ---
            park_dets = self._detect(hsv, PARK_LOWER, PARK_UPPER, PARK_MIN_AREA, morph=True)
            for i, (_, _, (bx, by, bw, bh)) in enumerate(park_dets, 1):
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.circle(annotated, (bx + bw // 2, by + bh // 2), 5, (0, 0, 255), -1)
                cv2.putText(annotated, f"Park {i}", (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # --- Durak (duraktespit.py: morfolojik temizlik yok) ---
            durak_dets = self._detect(hsv, DURAK_LOWER, DURAK_UPPER, DURAK_MIN_AREA, morph=False)
            for (_, _, (bx, by, bw, bh)) in durak_dets:
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (255, 200, 0), 3)
                cv2.circle(annotated, (int(bx + bw / 2), int(by + bh / 2)), 5, (0, 0, 255), -1)
                cv2.putText(annotated, "DURAK ALANI", (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

            # En yakin tespitleri karar_node formatinda yayinla
            if park_dets:
                m, o, _ = min(park_dets, key=lambda d: d[0])
                self.park_pub.publish(f"{m:.1f},{o:.1f}")
            else:
                self.park_pub.publish("none")

            if durak_dets:
                m, o, _ = min(durak_dets, key=lambda d: d[0])
                self.durak_pub.publish(f"{m:.1f},{o:.1f}")
            else:
                self.durak_pub.publish("none")

            with self._frame_lock:
                self.frame_to_show = annotated

            # Annotated goruntu yalnizca abone varsa yayinla (bant genisligi)
            if self.img_pub.get_num_connections() > 0:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        except Exception as e:
            rospy.logerr(f"park_durak callback hata: {e}")

    def run(self):
        if not self.show_gui:
            rospy.spin()
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self._frame_lock:
                frame = self.frame_to_show
            if frame is not None:
                cv2.imshow("Park/Durak Tespit", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    ParkDurakNode().run()
