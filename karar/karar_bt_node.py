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
from overtake import OvertakeManager, OvertakeParams

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

    # Sollama (ters şeride geç) + Ackermann geri-dönüş yöneticisi
    ovt = OvertakeManager(OvertakeParams.from_cfg(
        p.get("overtake", {}), p.get("ackermann", {})))
    hedef_max_age_s = float(p.get("freshness", {}).get("hedef_max_age_s", 2.0))

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

        # 2.5) Sollama (ters şeride geç) + Ackermann güvenli geri-dönüş → /hedef_komut
        o = bb.obs
        now = time.time()
        dkarar = bb.last_decision.get("karar", "normal")
        # GÜVENLİK (/incele Kritik): acil durusta sollamayı İPTAL et. Aksi halde
        # latch'li sollama, acil çözülünce aracı yanlış (karşı) şeride geri çekebilir.
        # ("dur" latch'i KORUR — engel sollama yolunu da kapattıysa durup devam eder.)
        if dkarar == "acildurus":
            ovt.reset()
            bb.state.overtake_active = False
            bb.state.overtake_return_dist_m = 0.0
        else:
            hedef_fresh = (o.hedef_last_seen > 0.0
                           and (now - o.hedef_last_seen) <= hedef_max_age_s)
            try:
                ores = ovt.update(
                    rx=o.x, ry=o.y, yaw=o.yaw,
                    engel_present=o.engel_present,
                    d_overall=o.engel_d_overall, d_center=o.engel_d_center,
                    angle_deg=o.engel_angle_deg,
                    hedef_x=o.hedef_x, hedef_y=o.hedef_y,
                    next_hedef_x=o.next_hedef_x, next_hedef_y=o.next_hedef_y,
                    hedef_fresh=hedef_fresh,
                    decision_karar=dkarar,
                    now=now,
                )
                bb.state.overtake_active = ores.active
                bb.state.overtake_return_dist_m = ores.return_dist_m
                if ores.command:
                    bridge.publish_hedef_komut(ores.command)
                if ores.event is not None:
                    faz, ev = ores.event
                    if klog is not None:
                        klog.log_overtake(faz, **ev)
                    rospy.loginfo(f"[karar_bt] sollama_{faz}: {ev}")
            except Exception as e:
                rospy.logerr_throttle(2.0, f"[karar_bt] overtake hata: {e}")

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
