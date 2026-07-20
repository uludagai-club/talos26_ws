#!/usr/bin/env python3
"""
YOLOv8 Trafik Isareti Algilama - Duzeltilmis Versiyon

Duzeltmeler:
  - OpenCV 4.13 copyMakeBorder float/int uyumsuzlugu giderildi
  - /trafik_levha (String) ve /yaya_gecidi (String) ciktilari eklendi
    (karar_node ile uyumlu format)
  - Bbox boyutundan mesafe tahmini eklendi
"""
import os
import rospy
import cv2

# --- OpenCV 4.13 fix: copyMakeBorder float arguman hatasi ---
_orig_copyMakeBorder = cv2.copyMakeBorder
def _fixed_copyMakeBorder(src, top, bottom, left, right, *args, **kwargs):
    return _orig_copyMakeBorder(src, int(top), int(bottom), int(left), int(right), *args, **kwargs)
cv2.copyMakeBorder = _fixed_copyMakeBorder

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

# Sinif -> karar_node format eslestirmesi
SINIF_ESLESTIRME = {
    'dur':                      'DUR',
    'lamba_kirmizi':            'DUR',
    'lamba_sari':               'YAVAS',
    'saga_mecburi_yon':         'SAG',
    'ileriden_saga_mecburi_yon':'SAG',
    'sola_mecburi_yon':         'SOL',
    'ileriden_sola_mecburi_yon':'SOL',
    'ileri_ve_saga_mecburi_yon':'SAG',
    'ileri_ve_sola_mecburi_yon':'SOL',
}

# Bbox yuksekliginden mesafe tahmini icin kalibrasyon (piksel * metre ~ sabit)
# Yaklasik deger: 640px yukseklikte 2m mesafe -> K = 1280
MESAFE_K = 1280.0
# Yatay offset metre olcegi (goruntu yari-genisligi ~ +-3 m varsayimi)
LATERAL_SCALE = 3.0
# YOLO levha tespiti guven esigi
CONF_ESIK = 0.5

# Canlı parametreler: config/canli_params.yaml 'levha:' — restart'sız uygulanır
try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("levha", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[levha] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class YOLOv8Node:
    def __init__(self):
        rospy.init_node('yolov8_node', anonymous=True)
        rospy.loginfo("Node initialized (fixed version)")

        self.bridge = CvBridge()

        try:
            rospy.loginfo("Loading YOLO model...")
            self.model = YOLO("/root/catkin_ws/src/yolov8_ros/scripts/best.pt")
            rospy.loginfo("Model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Model loading failed: {e}")
            rospy.signal_shutdown("Model load error")
            return

        # Orijinal publisher'lar
        self.pub = rospy.Publisher('/yolov8/image_annotated', Image, queue_size=1)

        # karar_node uyumlu publisher'lar
        self.levha_pub = rospy.Publisher('/trafik_levha', String, queue_size=10)
        self.yaya_pub = rospy.Publisher('/yaya_gecidi', String, queue_size=10)

        self.sub = rospy.Subscriber('/cart/front_camera/image_raw', Image, self.image_callback)
        rospy.loginfo("Subscriber and Publishers initialized")
        rospy.loginfo(f"Model siniflari: {self.model.names}")

        # GUI penceresi (lane-follower ile ayni kalip) - DISPLAY varsa acilir
        self.show_gui = bool(os.environ.get('DISPLAY'))
        self.frame_to_show = None
        if self.show_gui:
            rospy.loginfo("GUI aktif: 'Levha Tespit' penceresi acilacak")
        else:
            rospy.loginfo("DISPLAY yok - GUI penceresi devre disi, sadece topic yayini")

    def bbox_mesafe_tahmin(self, y1, y2, img_h):
        """Bbox yuksekliginden kaba mesafe tahmini (metre)."""
        bbox_h = abs(float(y2) - float(y1))
        if bbox_h < 1:
            return 99.0
        return MESAFE_K / bbox_h

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, verbose=False)
            h, w, _ = cv_image.shape

            en_yakin_levha = None
            en_yakin_levha_mesafe = 99.0
            en_yakin_yaya = None
            en_yakin_yaya_mesafe = 99.0
            en_yakin_bilinmeyen = None
            en_yakin_bilinmeyen_mesafe = 99.0

            if results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, "unknown")
                    conf = float(box.conf[0])

                    if conf < CONF_ESIK:
                        continue

                    # Merkez x (goruntude yatay konum, metre cinsinden kaba tahmin)
                    cx = float(x1 + x2) / 2
                    x_offset = (cx - w / 2) / (w / 2) * LATERAL_SCALE  # yaklasik -3m ile +3m

                    # Mesafe tahmini
                    mesafe = self.bbox_mesafe_tahmin(y1, y2, h)

                    if cls_name == 'yaya_gecidi':
                        if mesafe < en_yakin_yaya_mesafe:
                            en_yakin_yaya_mesafe = mesafe
                            en_yakin_yaya = f"{mesafe:.1f},{abs(x_offset):.1f}"
                    elif cls_name in SINIF_ESLESTIRME:
                        if mesafe < en_yakin_levha_mesafe:
                            en_yakin_levha_mesafe = mesafe
                            levha_isim = SINIF_ESLESTIRME[cls_name]
                            en_yakin_levha = f"{levha_isim},{mesafe:.1f},{abs(x_offset):.1f}"
                    else:
                        # Tabloda olmayan sinif: dusurme, ham adiyla yayinla ki
                        # karar/Decision'da gorunsun (BT bilinmeyen isimleri es gecer).
                        if mesafe < en_yakin_bilinmeyen_mesafe:
                            en_yakin_bilinmeyen_mesafe = mesafe
                            en_yakin_bilinmeyen = f"{cls_name.upper()},{mesafe:.1f},{abs(x_offset):.1f}"

            # karar_node'a yayinla — tablodaki sinif her zaman oncelikli
            if en_yakin_levha:
                self.levha_pub.publish(en_yakin_levha)
            elif en_yakin_bilinmeyen:
                self.levha_pub.publish(en_yakin_bilinmeyen)
            else:
                self.levha_pub.publish("none")

            if en_yakin_yaya:
                self.yaya_pub.publish(en_yakin_yaya)
            else:
                self.yaya_pub.publish("none")

            # Isaretlenmis goruntu
            annotated_frame = results[0].plot()
            self.frame_to_show = annotated_frame
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.pub.publish(out_msg)

        except Exception as e:
            rospy.logerr(f"YOLO node error: {e}")

    def run(self):
        """GUI varsa 'Levha Tespit' penceresini ana thread'de gosterir,
        yoksa duz rospy.spin() ile calisir (lane_follow_node.py ile ayni kalip)."""
        if not self.show_gui:
            rospy.spin()
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.frame_to_show is not None:
                cv2.imshow("Levha Tespit", self.frame_to_show)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    YOLOv8Node().run()
