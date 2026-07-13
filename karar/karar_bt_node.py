#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TALOS karar mekanizması — Behavior Tree tabanlı.

Eski fixes/karar.py ile bire-bir uyumlu çıkış:
  - /karar (std_msgs/String)
  - /karar_decision (cart_sim/Decision)
  - /karar_bt/snapshot (std_msgs/String, JSON, debug)
"""
from __future__ import annotations

import os
import sys
import time

import rospy
import yaml
import py_trees

# /app yolu Docker mount kalıbı; lokal çalıştırıldığında klasör de eklenir
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/app")

from bb import Blackboard
from ros_bridge import RosBridge
from trees.main_tree import build_root
from reroute import RerouteManager, RerouteParams
from levha_kisit import LevhaKisitManager, LevhaKisitParams

# talos_common bind-mount: /app/talos_common
try:
    from talos_common import TalosLogger
except Exception:
    TalosLogger = None

# Detaylı tanı logu (hedef_logger.py ikizi) — opsiyonel, import edilemese de çalışır
try:
    from karar_logger import KararLogger
except Exception:
    KararLogger = None


DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "params.yaml")


def load_params(path: str = DEFAULT_CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    rospy.init_node("karar_bt", anonymous=False)
    rospy.loginfo(">> KARAR MEKANİZMASI (BT) başlatıldı")

    cfg_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
    p = load_params(cfg_path)
    tick_hz = float(p.get("tick_hz", 10.0))
    rospy.loginfo(f"[karar_bt] config: {cfg_path} | tick={tick_hz}Hz")

    bb = Blackboard()
    bridge = RosBridge(bb, p)

    root = build_root(bb, p)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5.0)

    # P0: yapısal CSV — eski karar.py ile aynı şema
    if TalosLogger is not None:
        tlog = TalosLogger(
            component="karar",
            schema=[
                "decision_id", "karar", "reason",
                "input_yaya", "input_levha", "input_engel",
                "yaya_distance", "levha_class",
                "phase", "wait_remaining_s",
            ],
        )
        tlog.event("INFO", "karar_bt_started")
        tlog.start_health_loop(interval_s=1.0, node="karar")
    else:
        tlog = None
        rospy.logwarn("[karar_bt] talos_common bulunamadı — CSV log devre dışı.")

    # Detaylı tanı logu (hedef ile uyumlu: logs/<RUN_ID>/karar/{trace.csv,events.jsonl})
    if KararLogger is not None:
        klog = KararLogger()
        rospy.on_shutdown(lambda: klog.close())
    else:
        klog = None
        rospy.logwarn("[karar_bt] karar_logger bulunamadı — detaylı log devre dışı.")

    # Cone reroute yöneticisi (§16 E-A/E-B): bloklu cone'u /hedef_komut kenar_blok
    # ile hedef'e bildirir, temizlenince kenar_serbest. Eski OvertakeManager (sollama)
    # bununla değiştirildi — cone artık direksiyonla değil rotayla geçiliyor (§12.13).
    rm = RerouteManager(RerouteParams.from_cfg(
        p.get("reroute", {}), p.get("overtake", {})))

    # Levha yön-kısıtı yöneticisi: dönülmez/mecburi/giriş-yok levhalarını
    # kavşağın yasak koluna kenar_blok bırakarak rota kısıtına çevirir
    # (yeni hedef komutu yok; cone reroute ile aynı kanal, tick'te tek komut).
    lkm = LevhaKisitManager(LevhaKisitParams.from_cfg(
        p.get("levha_kisit", {}), p.get("freshness", {})))

    rate = rospy.Rate(tick_hz)
    last_logged_karar = None
    tick_count = 0
    ascii_every_n = int(p.get("debug", {}).get("ascii_tree_log_every_n_tick", 0) or 0)

    while not rospy.is_shutdown():
        # 1) Ağacı bir tick ileri sür
        try:
            tree.tick()
        except Exception as e:
            rospy.logerr_throttle(2.0, f"[karar_bt] tree tick hatası: {e}")
            # Güvenli mod: BT çöktü → en sert duruş (acildurus = brake100+N), "dur"
            # (brake80) değil. /incele güvenlik bulgusu (2026-06-24).
            bb.last_decision = {
                "karar": "acildurus",
                "reason": f"tick_exception:{type(e).__name__}",
                "phase": "fault",
                "wait_remaining_s": 0.0,
            }

        # 2) Karar yayını (her tick)
        bridge.publish_decision()

        # 2.5) Cone reroute → /hedef_komut (§16 E-A/E-B): bloklu cone'u dünya
        # frame'de kenar_blok ile hedef'e bildir; temizlenince kenar_serbest.
        # acildurus'ta RerouteManager bloğu KORUR (cone hâlâ orada; e-stop control'de).
        o = bb.obs
        now = time.time()
        dkarar = bb.last_decision.get("karar", "normal")
        hedef_komut_dolu = False   # bu tick /hedef_komut'a komut yayınlandı mı
        try:
            rres = rm.update(
                reroute_request=bb.state.reroute_request,
                cone_world=bb.state.reroute_cone_world,
                decision_karar=dkarar,
                now=now,
            )
            bb.state.overtake_active = rres.active        # snapshot/log aynası
            if rres.command:
                bridge.publish_hedef_komut(rres.command)
                hedef_komut_dolu = True
            if rres.event is not None:
                faz, ev = rres.event
                if klog is not None:
                    klog.log_reroute(faz, **ev)
                rospy.loginfo(f"[karar_bt] reroute_{faz}: {ev}")
        except Exception as e:
            rospy.logerr_throttle(2.0, f"[karar_bt] reroute hata: {e}")
        finally:
            # Tek-tick pulse: tree her tick reroute_request'i yeniden set eder;
            # set etmediği tick'te (cone block bandından çıktı) burada düşer →
            # RerouteManager debounce ile kenar_serbest verir.
            bb.state.reroute_request = False

        # 2.6) Levha yön-kısıtı → /hedef_komut: dönülmez/mecburi/giriş-yok
        # levhası görülünce kavşağın yasak koluna kenar_blok (soft ceza) bırak;
        # kavşak geçilince kenar_serbest. Cone komutu yayınlanan tick'te susar.
        try:
            px, py, pyaw, podom_t = bb.read_pose()
            lres = lkm.update(
                levha_isim=o.levha_isim,
                levha_ileri_m=o.levha_x,
                levha_age_s=(now - o.levha_last_seen) if o.levha_last_seen else 1e9,
                pose=(px, py, pyaw),
                odom_age_s=(now - podom_t) if podom_t else 1e9,
                decision_karar=dkarar,
                channel_busy=hedef_komut_dolu,
                now=now,
            )
            if lres.command:
                bridge.publish_hedef_komut(lres.command)
            if lres.event is not None:
                faz, ev = lres.event
                if klog is not None:
                    klog.log_event(f"levha_kisit_{faz}", **ev)
                rospy.loginfo(f"[karar_bt] levha_kisit_{faz}: {ev}")
        except Exception as e:
            rospy.logerr_throttle(2.0, f"[karar_bt] levha_kisit hata: {e}")

        # 3) Snapshot (rate-limited)
        if p.get("debug", {}).get("publish_snapshot", True):
            ascii_dump = ""
            if ascii_every_n > 0 and (tick_count % ascii_every_n == 0):
                try:
                    ascii_dump = py_trees.display.ascii_tree(root, show_status=True)
                except Exception:
                    ascii_dump = ""
            bridge.publish_snapshot(tree_ascii=ascii_dump)

        # 4) CSV log
        d = bb.last_decision
        karar = d.get("karar", "normal")
        decision_id = bb.state.last_decision_id or ""
        if tlog is not None:
            yaya_d = bb.obs.yaya_distance if bb.obs.yaya_distance is not None else -1.0
            tlog.metric(
                decision_id=decision_id,
                karar=karar,
                reason=d.get("reason", ""),
                input_yaya=bb.obs.raw_yaya,
                input_levha=bb.obs.raw_levha,
                input_engel="1" if bb.obs.engel_present else "0",
                yaya_distance=f"{yaya_d:.3f}",
                levha_class=bb.obs.levha_isim or "NONE",
                phase=d.get("phase", "driving"),
                wait_remaining_s=f"{d.get('wait_remaining_s', 0.0):.2f}",
            )

        # 4b) Detaylı karar izi (trace.csv, throttle'lı) — hedef pose.csv ikizi
        if klog is not None and klog.trace_due():
            klog.log_trace(
                x=o.x, y=o.y, yaw=o.yaw, speed_kmh=o.speed_kmh,
                karar=karar, reason=d.get("reason", ""), phase=d.get("phase", "driving"),
                engel_present=o.engel_present,
                d_center=o.engel_d_center, d_left=o.engel_d_left, d_right=o.engel_d_right,
                angle_deg=o.engel_angle_deg,
                kacis_yon=bb.state.kacis_yon, overtake_active=bb.state.overtake_active,
                hedef_x=o.hedef_x, hedef_y=o.hedef_y,
                engel_mem=o.engel_mem_count,
            )

        # 5) Karar değişiminde event logu
        if karar != last_logged_karar:
            msg = f"karar_change: {last_logged_karar} -> {karar} ({d.get('reason', '')})"
            rospy.loginfo(f"[karar_bt] {msg}")
            if tlog is not None:
                tlog.event("INFO", msg, decision_id=decision_id, reason=d.get("reason", ""))
            if klog is not None:
                klog.log_karar_change(last_logged_karar, karar, d.get("reason", ""),
                                      kacis_yon=bb.state.kacis_yon,
                                      kacis_kaynak=bb.state.kacis_kaynak)
                # Yola göre seçilmiş kaçış kararıysa zengin gerekçeyi ayrı olayla kaydet
                if karar in ("sol", "sag") and d.get("reason", "").startswith("engel_kacis"):
                    ex, ey = bb.state.kacis_engel_dunya
                    klog.log_kacis(
                        karar, bb.state.kacis_kaynak,
                        lateral_m=round(bb.state.kacis_lateral_m, 2),
                        engel_dunya=[round(ex, 2), round(ey, 2)],
                        return_dist_m=round(bb.state.overtake_return_dist_m, 2),
                    )
            last_logged_karar = karar

        tick_count += 1
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
