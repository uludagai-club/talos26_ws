#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
beemobs_gazebo_emulatoru.py — Sim'de Bee1 aracının /beemobs/* arayüzünü taklit eder
==================================================================================
DİJİTAL İKİZ: bu düğüm sim modunda smart_can_stuff (state-bridge) yerine geçer.
beemobs_bridge.py'nin yayınladığı /beemobs/* KOMUTLARINI dinler, cart_sim/cart_control
üretip /cart'a (Gazebo) yazar; Gazebo ground truth'tan /beemobs/FB_* geri beslemesini
üretir. Böylece stack (control.py -> CAN -> beemobs_bridge -> /beemobs/* -> BU EMÜLATÖR
-> /cart -> Gazebo -> FB -> beemobs_bridge) GERÇEK ARAÇ ARAYÜZÜYLE kapalı döngü koşar.

!!! GERÇEK ARAÇTA BU EMÜLATÖR KAPALI OLMALI !!!
    Araçta /beemobs/* komutlarını gerçek araç yorumlar ve gerçek FB'yi kendisi yayınlar.
    docker-compose.beemobs.yml araçta state-bridge'i BAŞLATMA (emülatör sadece sim).

ŞEMA NOTU: /beemobs mesajlarında Header YOK. Savunmacı _stamp() deseni kullanılır.
"""

import math
import os
import sys

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


# --- Kinematik/eşleme parametreleri ---
STEER_RATE_DEG_S = 40.0    # [deg/s] direksiyon motoru tam PWM'de açısal hız (SAHA ADIM5'te ölç)
# Direksiyon limiti TEK kaynaktan (ackermann.py; incele 2026-07-04 — elle kopyalar
# çoğalmasın; container'da ackermann.py mount'u docker-compose.beemobs.yml'de).
try:
    import ackermann as _ackermann
    STEER_LIMIT_DEG = float(_ackermann.max_bicycle_angle())   # ≈28.95
except Exception:
    STEER_LIMIT_DEG = 28.95   # [deg]  mekanik teker limiti -> cart_control.steer = aci/limit
THR_BAND_MIN = 50          # gaz POSITION band alt/üst (50-250 -> 0-1)
THR_BAND_MAX = 250
IGNITION_FB_GECIKME = 0.5  # [s]  RC_Ignition=1 sonrası FB_IGNITION=1 gecikmesi
CART_HZ = 20.0             # /cart yayın frekansı
FB_HZ = 10.0              # /beemobs/FB_* yayın frekansı

_BEEMOBS_MESAJLARI = (
    "rc_unittoOmux", "RC_THRT_DATA", "AUTONOMOUS_BrakePedalControl",
    "AUTONOMOUS_SteeringMot_Control", "AUTONOMOUS_HB_MotorControl",
    "FB_VehicleSpeed", "FeedbackSteeringAngle", "FB_OMUX_to_AUTONOMOUS",
)


def _devel_yollarini_ekle():
    for _p in ("/can_ws/devel/lib/python3/dist-packages",
               "/talos-devel/lib/python3/dist-packages",
               os.path.expanduser("~/talos-sim/devel/lib/python3/dist-packages")):
        if os.path.isdir(_p) and _p not in sys.path:
            sys.path.insert(0, _p)


def _yukle_mesajlar(paket_tercih="smart_can_msgs"):
    """beemobs mesaj sınıflarını (smart_can_msgs veya cart_sim) + cart_control'ü yükler."""
    import importlib
    _devel_yollarini_ekle()
    son_hata = None
    for pkg in [paket_tercih] + (["cart_sim"] if paket_tercih != "cart_sim" else []):
        try:
            mod = importlib.import_module(pkg + ".msg")
            m = {ad: getattr(mod, ad) for ad in _BEEMOBS_MESAJLARI}
            rospy.loginfo("[EMU] beemobs mesajlari '%s' paketinden yuklendi.", pkg)
            break
        except (ImportError, AttributeError) as e:
            son_hata = e
            m = None
    if m is None:
        raise ImportError("beemobs mesajlari yuklenemedi. Son hata: %s" % son_hata)
    from cart_sim.msg import cart_control  # /cart komutu (Gazebo)
    return m, cart_control


class BeemobsGazeboEmulatoru:
    def __init__(self, m, cart_control_cls):
        self.m = m
        self.cart_control = cart_control_cls

        # --- gelen komut durumu ---
        self.ignition = 0
        self.gear = 0            # RC_SelectionGear: 0=N,1=D,2=R
        self.thr_press = 1       # 0=güç ver, 1=serbest
        self.thr_pos = THR_BAND_MIN
        self.brk_en = 0
        self.brk_per = 0
        self.steer_en = 0
        self.steer_pwm = 128     # 0/128 = dur
        self.hb_motstate = 1     # 0=ÇEK, 1=İNDİR(serbest)
        self.hb_moten = 0

        # --- iç durum ---
        self.steer_deg = 0.0     # entegre edilen tekerlek açısı (+ = SOL)
        self.speed_ms = 0.0      # Gazebo ground truth'tan
        self.ignition_t = None   # RC_Ignition=1 olduğu an
        self.injected_emergency = 0   # /beemobs_emu/estop_enjekte test kancası
        self._son_cart_t = None

        # --- publisher'lar ---
        self.pub_cart = rospy.Publisher("/cart", cart_control_cls, queue_size=10)
        self.pub_speed = rospy.Publisher("/beemobs/FB_VehicleSpeed", m["FB_VehicleSpeed"], queue_size=1)
        self.pub_steer_fb = rospy.Publisher("/beemobs/FeedbackSteeringAngle",
                                            m["FeedbackSteeringAngle"], queue_size=1)
        self.pub_omux_fb = rospy.Publisher("/beemobs/FB_OMUX_to_AUTONOMOUS",
                                           m["FB_OMUX_to_AUTONOMOUS"], queue_size=1)

        # --- komut subscriber'ları ---
        rospy.Subscriber("/beemobs/rc_unittoOmux", m["rc_unittoOmux"], self._omux_cb, queue_size=1)
        rospy.Subscriber("/beemobs/RC_THRT_DATA", m["RC_THRT_DATA"], self._thr_cb, queue_size=1)
        rospy.Subscriber("/beemobs/AUTONOMOUS_BrakePedalControl",
                         m["AUTONOMOUS_BrakePedalControl"], self._brk_cb, queue_size=1)
        rospy.Subscriber("/beemobs/AUTONOMOUS_SteeringMot_Control",
                         m["AUTONOMOUS_SteeringMot_Control"], self._steer_cb, queue_size=1)
        rospy.Subscriber("/beemobs/AUTONOMOUS_HB_MotorControl",
                         m["AUTONOMOUS_HB_MotorControl"], self._hb_cb, queue_size=1)

        # --- Gazebo ground truth + test kancası ---
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("/beemobs_emu/estop_enjekte", Bool, self._estop_enjekte_cb, queue_size=1)

        # timers
        rospy.Timer(rospy.Duration(1.0 / CART_HZ), self._cart_tik)
        rospy.Timer(rospy.Duration(1.0 / FB_HZ), self._fb_tik)

        rospy.loginfo("=" * 70)
        rospy.loginfo("  BEEMOBS GAZEBO EMULATORU (sim'de gercek arac arayuzunu taklit eder)")
        rospy.loginfo("  /beemobs/* -> /cart (%.0f Hz) | ground truth -> /beemobs/FB_* (%.0f Hz)",
                      CART_HZ, FB_HZ)
        rospy.loginfo("=" * 70)

    def _stamp(self, msg, now):
        """Savunmacı Header damgası (beemobs'ta Header yok -> dokunmaz)."""
        if "header" in getattr(msg, "__slots__", ()):
            try:
                msg.header.stamp = now
            except Exception:
                pass
        return msg

    # ---- komut callback'leri ----
    def _omux_cb(self, msg):
        yeni_ign = int(getattr(msg, "RC_Ignition", 0))
        if yeni_ign == 1 and self.ignition != 1:
            self.ignition_t = rospy.Time.now()
        if yeni_ign == 0:
            self.ignition_t = None
        self.ignition = yeni_ign
        self.gear = int(getattr(msg, "RC_SelectionGear", 0))

    def _thr_cb(self, msg):
        self.thr_press = int(getattr(msg, "RC_THRT_PEDAL_PRESS", 1))
        self.thr_pos = int(getattr(msg, "RC_THRT_PEDAL_POSITION", THR_BAND_MIN))

    def _brk_cb(self, msg):
        self.brk_en = int(getattr(msg, "AUTONOMOUS_BrakePedalMotor_EN", 0))
        self.brk_per = int(getattr(msg, "AUTONOMOUS_BrakePedalMotor_PER", 0))

    def _steer_cb(self, msg):
        self.steer_en = int(getattr(msg, "AUTONOMOUS_SteeringMot_EN", 0))
        self.steer_pwm = int(getattr(msg, "AUTONOMOUS_SteeringMot_PWM", 128))

    def _hb_cb(self, msg):
        self.hb_moten = int(getattr(msg, "AUTONOMOUS_HB_MotEN", 0))
        self.hb_motstate = int(getattr(msg, "AUTONOMOUS_HB_MotState", 1))

    def _odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_ms = math.hypot(vx, vy)

    def _estop_enjekte_cb(self, msg):
        self.injected_emergency = 1 if bool(msg.data) else 0
        rospy.logwarn("[EMU] estop_enjekte = %d (FB_EMERGENCY)", self.injected_emergency)

    # ---- eşlemeler ----
    def _pwm_to_steer_rate(self, pwm):
        """PWM -> direksiyon açısal hızı (deg/s). 1-127 sol (+), 128-255 sağ (-), 0/128 dur."""
        if not self.steer_en:
            return 0.0
        if pwm == 0 or pwm == 128:
            return 0.0
        if pwm < 128:                                    # SOL (+)
            return +STEER_RATE_DEG_S * (pwm / 127.0)     # pwm 127 -> en hızlı sol
        return -STEER_RATE_DEG_S * ((pwm - 128) / 127.0)  # pwm 255 -> en hızlı sağ

    def _cart_komutu(self):
        """Gelen /beemobs komutlarından cart_control alanlarını üretir (steer entegrasyonu
        _cart_tik'te yapılır; burada steer_deg zaten güncel)."""
        cc = self.cart_control()
        # RC_Ignition=0 -> araç KAPALI: gaz kes, tam fren.
        if self.ignition != 1:
            cc.throttle = 0.0
            cc.brake = 1.0
            cc.steer = max(-1.0, min(1.0, self.steer_deg / STEER_LIMIT_DEG))
            cc.handbrake = 1.0
            cc.shift_gears = self.cart_control.NEUTRAL
            return cc
        # Gaz: PRESS==0 -> (POSITION-50)/200 (band 50-250 -> 0-1), aksi 0.
        if self.thr_press == 0:
            cc.throttle = max(0.0, min(1.0, (self.thr_pos - THR_BAND_MIN) / float(THR_BAND_MAX - THR_BAND_MIN)))
        else:
            cc.throttle = 0.0
        # Fren: EN ? PER/100 : 0
        cc.brake = max(0.0, min(1.0, self.brk_per / 100.0)) if self.brk_en else 0.0
        # El freni: MotState==0 -> ÇEK (1.0)
        cc.handbrake = 1.0 if self.hb_motstate == 0 else 0.0
        # Vites: RC_SelectionGear {0->N, 1->FORWARD, 2->REVERSE}
        cc.shift_gears = {0: self.cart_control.NEUTRAL,
                          1: self.cart_control.FORWARD,
                          2: self.cart_control.REVERSE}.get(self.gear, self.cart_control.NEUTRAL)
        # Direksiyon: entegre açı -> normalize
        cc.steer = max(-1.0, min(1.0, self.steer_deg / STEER_LIMIT_DEG))
        return cc

    # ---- timer'lar ----
    def _cart_tik(self, event):
        now = rospy.Time.now()
        # direksiyon açısını PWM'den entegre et (±STEER_LIMIT_DEG)
        if self._son_cart_t is not None:
            dt = (now - self._son_cart_t).to_sec()
            if 0.0 < dt < 0.5:
                self.steer_deg = max(-STEER_LIMIT_DEG,
                                     min(STEER_LIMIT_DEG,
                                         self.steer_deg + self._pwm_to_steer_rate(self.steer_pwm) * dt))
        self._son_cart_t = now

        cc = self._cart_komutu()
        self._stamp(cc, now)
        self.pub_cart.publish(cc)

    def _fb_tik(self, event):
        now = rospy.Time.now()

        # FB_VehicleSpeed: GERÇEK ŞEMA uint8 (kaba çözünürlük bilinçli korunuyor).
        kmh = int(round(max(0.0, min(255.0 / 3.6, self.speed_ms)) * 3.6))
        ms = int(round(max(0.0, min(255.0, self.speed_ms))))
        sp = self.m["FB_VehicleSpeed"]()
        self._stamp(sp, now)
        sp.FB_ReelVehicleSpeed_Ms = min(255, max(0, ms))
        sp.FB_ReelVehicleSpeed_KMh = min(255, max(0, kmh))
        sp.FB_VehicleSpeed_KMh = min(255, max(0, kmh))
        self.pub_speed.publish(sp)

        # FeedbackSteeringAngle: int8, entegre açı
        st = self.m["FeedbackSteeringAngle"]()
        self._stamp(st, now)
        st.FeedBackSteeringAngle = max(-127, min(127, int(round(self.steer_deg))))
        self.pub_steer_fb.publish(st)

        # FB_OMUX_to_AUTONOMOUS
        fb_ign = 0
        if self.ignition == 1 and self.ignition_t is not None:
            if (now - self.ignition_t).to_sec() >= IGNITION_FB_GECIKME:
                fb_ign = 1
        om = self.m["FB_OMUX_to_AUTONOMOUS"]()
        self._stamp(om, now)
        om.FB_IGNITION = fb_ign
        om.FB_EMERGENCY = self.injected_emergency
        om.FB_VehicleStatus = 1
        om.FB_BatteryVoltage = 72
        om.FB_BatterySOC = 95
        self.pub_omux_fb.publish(om)


def main():
    rospy.init_node("beemobs_gazebo_emulatoru")
    paket_tercih = rospy.get_param("~msg_paketi", os.environ.get("BEEMOBS_MSG_PAKETI", "smart_can_msgs"))
    m, cart_control_cls = _yukle_mesajlar(paket_tercih)
    BeemobsGazeboEmulatoru(m, cart_control_cls)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
