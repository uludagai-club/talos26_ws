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
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class GroundFilter:
    def __init__(self):
        rospy.init_node("ground_filter")
        self.in_topic = rospy.get_param("~input", "/cart/center_laser/scan")
        self.out_topic = rospy.get_param("~output", "/cart/points_noground")
        self.z_min = float(rospy.get_param("~z_min", -0.8))   # bunun altı zemin
        self.z_max = float(rospy.get_param("~z_max", 5.0))    # çok yüksek artefakt
        self.r_min = float(rospy.get_param("~r_min", 0.5))    # araç gövdesi öz-yansıma
        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.in_topic, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo(f"[ground_filter] {self.in_topic} → {self.out_topic} "
                      f"(z_min={self.z_min}, z_max={self.z_max})")

    def cb(self, msg):
        kept = []
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.z_min or z > self.z_max:
                continue
            if (x * x + y * y) < (self.r_min * self.r_min):
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
