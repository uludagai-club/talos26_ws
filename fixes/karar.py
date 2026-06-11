#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import uuid

import rospy
from std_msgs.msg import String

# talos_common bind-mount: /app/talos_common
sys.path.insert(0, "/app")
try:
    from talos_common import TalosLogger
except Exception:  # pragma: no cover
    TalosLogger = None

# cart_sim/Decision msg — devel mount edilmişse mevcut
try:
    from cart_sim.msg import Decision as DecisionMsg
    _HAS_DECISION = True
except Exception:
    DecisionMsg = None
    _HAS_DECISION = False


# 1. Mesafe Eşikleri (Metre)
MESAFE_ACIL_DURUS = 2.0
MESAFE_YAYA_DUR = 4.0
MESAFE_YAYA_YAVAS = 12.0
MESAFE_LEVHA_DUR = 3.5
MESAFE_LEVHA_OKU = 10.0

# 2. Zamanlayıcılar (Saniye)
SURE_DUR_LEVHASI_BEKLEME = 3.0


class KararMekanizmasi:
    def __init__(self):
        rospy.init_node('karar_mekanizmasi_enes', anonymous=True)
        rospy.loginfo(">> KARAR MEKANİZMASI BAŞLATILDI (Logic Unit Active)")
        rospy.loginfo(">> Beklenen Veri Formatı: 'x,y' (Metre cinsinden bağıl konum)")

        rospy.Subscriber("/trafik_levha", String, self.levha_callback)
        rospy.Subscriber("/yaya_gecidi", String, self.yaya_callback)

        # Geriye dönük uyumluluk için /karar (String) kalıyor; ek olarak yapısal /karar_decision
        self.karar_pub = rospy.Publisher('/karar', String, queue_size=10)
        if _HAS_DECISION:
            self.decision_pub = rospy.Publisher('/karar_decision', DecisionMsg, queue_size=10)
        else:
            self.decision_pub = None
            rospy.logwarn("[karar] cart_sim.msg.Decision import edilemedi — sadece /karar yayınlanacak.")

        self.yaya_verisi = "none"
        self.levha_verisi = "none"

        self.dur_levhasi_aktif = False
        self.durma_baslangic_zamani = None

        # P0: yapısal CSV + decision_id
        if TalosLogger is not None:
            self.tlog = TalosLogger(
                component="karar",
                schema=[
                    "decision_id", "karar", "reason",
                    "input_yaya", "input_levha", "input_engel",
                    "yaya_distance", "levha_class",
                    "phase", "wait_remaining_s",
                ],
            )
            self.tlog.event("INFO", "karar_node_started")
            self.tlog.start_health_loop(interval_s=1.0, node="karar")
        else:
            self.tlog = None
            rospy.logwarn("[karar] talos_common bulunamadı — yapısal CSV devre dışı.")

        self._last_karar = None
        self._last_decision_id = None

        self.rate = rospy.Rate(10)

    def levha_callback(self, msg):
        self.levha_verisi = msg.data

    def yaya_callback(self, msg):
        self.yaya_verisi = msg.data

    def mesafe_hesapla(self, x_str, y_str):
        try:
            return math.hypot(float(x_str), float(y_str))
        except ValueError:
            return -1.0

    def _publish_decision(self, karar, reason, yaya_mesafesi, levha_ismi, phase, wait_remaining):
        decision_id = uuid.uuid4().hex
        self._last_decision_id = decision_id

        # 1) Geri uyumlu String
        self.karar_pub.publish(karar)

        # 2) Yapısal mesaj
        if self.decision_pub is not None:
            d = DecisionMsg()
            d.header.stamp = rospy.Time.now()
            d.header.frame_id = "karar"
            d.decision_id = decision_id
            d.karar = karar
            d.reason = reason
            d.input_yaya = self.yaya_verisi
            d.input_levha = self.levha_verisi
            d.input_engel = ""  # engel-node ayrı doldurur (rx tarafında join)
            d.yaya_distance = float(yaya_mesafesi if yaya_mesafesi is not None else -1.0)
            d.levha_class = levha_ismi or "none"
            d.phase = phase
            d.wait_remaining_s = float(wait_remaining)
            self.decision_pub.publish(d)

        # 3) CSV
        if self.tlog is not None:
            self.tlog.metric(
                decision_id=decision_id,
                karar=karar,
                reason=reason,
                input_yaya=self.yaya_verisi,
                input_levha=self.levha_verisi,
                input_engel="",
                yaya_distance=f"{(yaya_mesafesi if yaya_mesafesi is not None else -1.0):.3f}",
                levha_class=levha_ismi or "none",
                phase=phase,
                wait_remaining_s=f"{wait_remaining:.2f}",
            )

        # 4) Olay logu — sadece karar değişince
        if karar != self._last_karar:
            if self.tlog is not None:
                self.tlog.event("INFO", f"karar_change: {self._last_karar} -> {karar} ({reason})",
                                decision_id=decision_id, reason=reason)
            self._last_karar = karar

    def mantik_yurut(self):
        while not rospy.is_shutdown():
            nihai_karar = "normal"
            reason = "default"
            phase = "driving"
            wait_remaining = 0.0

            yaya_mesafesi = -1
            levha_mesafesi = -1
            levha_ismi = "none"

            if self.yaya_verisi != "none":
                try:
                    parcalar = self.yaya_verisi.split(',')
                    yaya_mesafesi = self.mesafe_hesapla(parcalar[0], parcalar[1])
                except Exception:
                    rospy.logwarn("Veri Hatasi: Yaya verisi parse edilemedi!")

            if self.levha_verisi != "none":
                try:
                    parcalar = self.levha_verisi.split(',')
                    levha_ismi = parcalar[0]
                    levha_mesafesi = self.mesafe_hesapla(parcalar[1], parcalar[2])
                except Exception:
                    rospy.logwarn("Veri Hatasi: Levha verisi parse edilemedi!")

            if (yaya_mesafesi != -1 and yaya_mesafesi < MESAFE_ACIL_DURUS):
                nihai_karar = "acildurus"
                reason = f"yaya_mesafesi<{MESAFE_ACIL_DURUS}"
                rospy.logerr(f"!!! ACİL DURUM !!! Yaya Çok Yakın: {yaya_mesafesi:.2f}m")

            elif yaya_mesafesi != -1:
                if yaya_mesafesi < MESAFE_YAYA_DUR:
                    nihai_karar = "dur"
                    reason = "yaya_dur"
                    rospy.logwarn(f"Yaya Geçidi: Tam Durus. Mesafe: {yaya_mesafesi:.2f}m")
                elif yaya_mesafesi < MESAFE_YAYA_YAVAS:
                    nihai_karar = "slow"
                    reason = "yaya_yavas"
                    rospy.loginfo(f"Yaya Geçidi: Yavaşlaniyor. Mesafe: {yaya_mesafesi:.2f}m")

            elif levha_mesafesi != -1 and levha_mesafesi < MESAFE_LEVHA_OKU:
                if levha_ismi == "DUR":
                    if levha_mesafesi < MESAFE_LEVHA_DUR or self.dur_levhasi_aktif:
                        if not self.dur_levhasi_aktif:
                            self.dur_levhasi_aktif = True
                            self.durma_baslangic_zamani = rospy.get_time()
                            rospy.loginfo("🛑 DUR LEVHASI: 3 Saniyelik bekleme başlatıldı.")

                        gecen_sure = rospy.get_time() - self.durma_baslangic_zamani
                        wait_remaining = max(0.0, SURE_DUR_LEVHASI_BEKLEME - gecen_sure)
                        if gecen_sure < SURE_DUR_LEVHASI_BEKLEME:
                            nihai_karar = "dur"
                            reason = "dur_levhasi_bekleme"
                            phase = "waiting_at_stop"
                            rospy.loginfo(f"🛑 Bekleniyor... ({gecen_sure:.1f}/{SURE_DUR_LEVHASI_BEKLEME}s)")
                        else:
                            nihai_karar = "normal"
                            reason = "dur_levhasi_tamamlandi"
                            if levha_mesafesi > MESAFE_LEVHA_DUR + 2.0:
                                self.dur_levhasi_aktif = False

                    elif levha_mesafesi < MESAFE_LEVHA_OKU:
                        nihai_karar = "slow"
                        reason = "dur_levhasi_yaklasma"

                elif levha_ismi == "30" or levha_ismi == "OKUL":
                    nihai_karar = "slow"
                    reason = f"hiz_siniri_{levha_ismi}"

                elif levha_ismi == "SAG" and levha_mesafesi < 5.0:
                    nihai_karar = "sag"
                    reason = "yon_sag"
                elif levha_ismi == "SOL" and levha_mesafesi < 5.0:
                    nihai_karar = "sol"
                    reason = "yon_sol"

            else:
                nihai_karar = "normal"
                reason = "no_threat"
                if self.levha_verisi == "none":
                    self.dur_levhasi_aktif = False

            self._publish_decision(nihai_karar, reason, yaya_mesafesi, levha_ismi, phase, wait_remaining)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        beyin = KararMekanizmasi()
        beyin.mantik_yurut()
    except rospy.ROSInterruptException:
        pass
