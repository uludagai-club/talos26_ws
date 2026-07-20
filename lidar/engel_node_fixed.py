#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Engel Tespiti Node - v3 (Dinamik Tarama + Kacinma Verisi)

Yenilikler:
  - /steer_angle subscribe ederek tarama merkezini direksiyonla esler
  - Sol/Sag sektor mesafesi yayinlar (kacinma icin)
  - Engel acisi yayinlar (engelin hangi tarafta oldugu)
  - Tarama genisligi: +/- 15 derece (toplam 30 derece)
  - Merkez sektor: +/- 5 derece (fren karari icin)
"""

import math
import sys

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32

sys.path.insert(0, "/app")
try:
    from talos_common import TalosLogger
except Exception:
    TalosLogger = None

try:
    from cart_sim.msg import Decision as DecisionMsg
    _HAS_DECISION = True
except Exception:
    DecisionMsg = None
    _HAS_DECISION = False

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER — hepsi burada
#   (canlı: config/canli_params.yaml 'engel:' — restart'sız uygulanır)
# ════════════════════════════════════════════════════════════════════════
ENGEL_MESAFE_LIMITI_M = 3.0   # m - merkez sektörde engel var/yok eşiği (BT kendi eşiklerini uygular; geniş tut)
SCAN_HALF_FOV_DEG     = 15.0  # derece - toplam tarama yarı-açısı (+/-)
MERKEZ_HALF_FOV_DEG   = 5.0   # derece - fren kararı merkez sektörü (+/-)
MIN_RANGE_M           = 0.3   # m - bundan yakın okumalar yok sayılır (öz-yansıma)
STEER_SCALE           = 0.5   # direksiyon açısı → tarama merkezi kaydırma oranı

try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("engel", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[engel_node] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class EngelTespitiNode:
    def __init__(self):
        rospy.init_node('engel_tespiti', anonymous=True)

        # --- Parametreler üst blokta (AYARLANABİLİR PARAMETRELER) ---
        self.steer_angle_deg = 0.0

        # --- Publishers ---
        self.engel_pub = rospy.Publisher('/engel', Int32, queue_size=10)
        self.mesafe_pub = rospy.Publisher('/engel_distance', Float32, queue_size=10)
        self.engel_aci_pub = rospy.Publisher('/engel_angle', Float32, queue_size=10)
        self.sol_mesafe_pub = rospy.Publisher('/engel_sol_mesafe', Float32, queue_size=10)
        self.sag_mesafe_pub = rospy.Publisher('/engel_sag_mesafe', Float32, queue_size=10)

        # --- Subscribers ---
        self.scan_sub = rospy.Subscriber('/converted_scan', LaserScan, self.scan_callback)
        self.steer_sub = rospy.Subscriber('/steer_angle', Float32, self.steer_callback)

        # En son karar id'si (decision_id zinciri için).
        # N5: ilk karar gelene kadar "pre_karar" sentinel — orphan boş string yerine.
        self._last_decision_id = "pre_karar"
        if _HAS_DECISION:
            rospy.Subscriber('/karar_decision', DecisionMsg, self._decision_callback)

        # P0 — yapısal CSV
        if TalosLogger is not None:
            self.tlog = TalosLogger(
                component="engel",
                schema=[
                    "decision_id",
                    "min_d_center", "min_d_left", "min_d_right",
                    "min_d_overall", "min_angle_deg",
                    "steer_offset_deg", "obstacle_present",
                ],
            )
            self.tlog.event("INFO", "engel_node_started")
            self.tlog.start_health_loop(interval_s=1.0, node="engel")
        else:
            self.tlog = None

        rospy.loginfo("Engel Tespiti Node v3 Baslatildi (dinamik tarama)")
        rospy.loginfo(f"Mesafe Limiti: {ENGEL_MESAFE_LIMITI_M}m, "
                      f"Tarama: +/- {SCAN_HALF_FOV_DEG} derece, "
                      f"Merkez: +/- {MERKEZ_HALF_FOV_DEG} derece")

    def _decision_callback(self, msg):
        self._last_decision_id = msg.decision_id or ""

    def steer_callback(self, msg):
        """Direksiyon acisi callback (derece, sag: pozitif, sol: negatif)"""
        self.steer_angle_deg = msg.data

    def scan_callback(self, data):
        # Direksiyon acisina gore tarama merkezini kaydir
        # Negatif steer = sola donus -> tarama merkezi sola kayar
        center_offset_rad = math.radians(self.steer_angle_deg * STEER_SCALE)

        half_fov_rad = math.radians(SCAN_HALF_FOV_DEG)
        merkez_half_rad = math.radians(MERKEZ_HALF_FOV_DEG)

        angle_min = data.angle_min
        angle_increment = data.angle_increment

        # Sektor minimumlari
        min_mesafe = float('inf')       # Toplam minimum
        min_mesafe_aci = 0.0            # Minimum mesafenin acisi
        min_sol = float('inf')          # Sol sektor minimum
        min_sag = float('inf')          # Sag sektor minimum
        min_merkez = float('inf')       # Merkez sektor minimum

        for i, range_val in enumerate(data.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
            if range_val < MIN_RANGE_M:
                continue

            # Bu noktanin acisi (radyan)
            current_angle = angle_min + (i * angle_increment)

            # -pi..pi araligina normalize et
            while current_angle > math.pi:
                current_angle -= 2 * math.pi
            while current_angle < -math.pi:
                current_angle += 2 * math.pi

            # Direksiyon offseti uygula - tarama merkezine gore bagil aci
            relative_angle = current_angle - center_offset_rad

            # Toplam FOV icerisinde mi?
            if -half_fov_rad <= relative_angle <= half_fov_rad:
                if range_val < min_mesafe:
                    min_mesafe = range_val
                    min_mesafe_aci = math.degrees(relative_angle)

                # Sol sektor (negatif acilar)
                if relative_angle < 0 and range_val < min_sol:
                    min_sol = range_val

                # Sag sektor (pozitif acilar)
                if relative_angle >= 0 and range_val < min_sag:
                    min_sag = range_val

                # Merkez sektor
                if -merkez_half_rad <= relative_angle <= merkez_half_rad:
                    if range_val < min_merkez:
                        min_merkez = range_val

        # Engel var/yok (merkez sektordeki engele gore karar ver)
        engel_durumu = 0
        if min_merkez < ENGEL_MESAFE_LIMITI_M:
            engel_durumu = 1

        # Yakin engel varsa loglama (her engel icin degil, sadece merkezdeki)
        if engel_durumu == 1:
            rospy.loginfo(f"ENGEL! Merkez: {min_merkez:.2f}m | "
                          f"Sol: {min_sol:.2f}m | Sag: {min_sag:.2f}m | "
                          f"Steer offset: {self.steer_angle_deg:.1f} derece")

        # Yayinla
        self.engel_pub.publish(engel_durumu)
        # /engel_distance → merkez sektör minimumu (BT engel_d_center olarak kullanır)
        self.mesafe_pub.publish(min_merkez)
        self.engel_aci_pub.publish(min_mesafe_aci)
        self.sol_mesafe_pub.publish(min_sol)
        self.sag_mesafe_pub.publish(min_sag)

        # Yapısal CSV — inf değerlerini -1 ile ifade et
        if self.tlog is not None:
            def _fin(v):
                return v if math.isfinite(v) else -1.0
            self.tlog.metric(
                decision_id=self._last_decision_id,
                min_d_center=f"{_fin(min_merkez):.3f}",
                min_d_left=f"{_fin(min_sol):.3f}",
                min_d_right=f"{_fin(min_sag):.3f}",
                min_d_overall=f"{_fin(min_mesafe):.3f}",
                min_angle_deg=f"{min_mesafe_aci:.2f}",
                steer_offset_deg=f"{self.steer_angle_deg:.2f}",
                obstacle_present=engel_durumu,
            )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = EngelTespitiNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
