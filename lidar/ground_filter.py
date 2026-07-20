#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal zemin-ayıklama (ground removal) — glue kodu.

kerem'in talos_obstacle_detector'ı zemin-ayıklanmış `/cart/points_noground`
bekliyor (Patchwork++ çıktısı), ama o pipeline yok. Bu node, ham velodyne
bulutundan basit bir z-passthrough ile zemini atıp /cart/points_noground
yayınlar — engelli sim'in (dev koni/varil) demosu için yeterli.

velodyne frame'inde sensör zeminin ~1.0m üstünde → zemin z ≈ -1.0. z_min
üstündeki noktalar (engeller) tutulur. Çıktı header'ı girdiyle aynı (frame
korunur → detector + karar_bt sensor_offset fix'iyle tutarlı).

NOT: Bu Patchwork++ DEĞİL — kaba bir düzlemsel eşik. Eğimli/engebeli zeminde
yetersiz; gerçek saha için RANSAC/Patchwork++ ile değiştirilmeli.
"""
import os
import sys

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER — hepsi burada
#   (canlı: config/canli_params.yaml 'ground_filter:' — restart'sız uygulanır;
#    rosparam ~ override'ları başlangıçta hâlâ geçerlidir)
# ════════════════════════════════════════════════════════════════════════
IN_TOPIC  = "/cart/center_laser/scan"   # (RESTART) abonelik başlangıçta kurulur
OUT_TOPIC = "/cart/points_noground"     # (RESTART)
Z_MIN_M   = -0.8   # m - bunun altı zemin sayılır
Z_MAX_M   = 5.0    # m - çok yüksek artefakt eşiği
R_MIN_M   = 0.5    # m - araç gövdesi öz-yansıma yarıçapı

# Bu node HOST'ta koşar (baslat.sh) → talos_common repo kökünden import edilir
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("ground_filter", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[ground_filter] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class GroundFilter:
    def __init__(self):
        rospy.init_node("ground_filter")
        # rosparam (~) verilmişse başlangıçta üst bloğu ezer
        for _ad, _param in [("IN_TOPIC", "~input"), ("OUT_TOPIC", "~output"),
                            ("Z_MIN_M", "~z_min"), ("Z_MAX_M", "~z_max"),
                            ("R_MIN_M", "~r_min")]:
            globals()[_ad] = rospy.get_param(_param, globals()[_ad])
        self.pub = rospy.Publisher(OUT_TOPIC, PointCloud2, queue_size=1)
        rospy.Subscriber(IN_TOPIC, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo(f"[ground_filter] {IN_TOPIC} → {OUT_TOPIC} "
                      f"(z_min={Z_MIN_M}, z_max={Z_MAX_M})")

    def cb(self, msg):
        kept = []
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < Z_MIN_M or z > Z_MAX_M:
                continue
            if (x * x + y * y) < (R_MIN_M * R_MIN_M):
                continue
            kept.append((x, y, z))
        out = pc2.create_cloud_xyz32(msg.header, kept)
        self.pub.publish(out)


if __name__ == "__main__":
    try:
        GroundFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
