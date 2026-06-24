#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""overtake.py (OvertakeManager) birim testleri — ROS yok.

Çalıştır:
    python3 karar/test/test_overtake.py
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from overtake import OvertakeManager, OvertakeParams  # noqa: E402

_fail = []


def check(name, cond, detay=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        _fail.append(f"{name} {detay}")
        print(f"  ✗ {name} {detay}")


def base_params(enabled=True):
    return OvertakeParams(enabled=enabled, lane_offset_m=1.8, return_steer_deg=20.0,
                          clearance_m=2.0, block_radius_m=1.0, max_s=15.0,
                          refresh_s=1.0, wheelbase_m=1.78, max_steer_deg=30.0)


def commit_args(decision="sol", rx=0.0, now=100.0):
    return dict(rx=rx, ry=0.0, yaw=0.0, engel_present=True,
                d_overall=3.0, d_center=3.0, angle_deg=0.0,
                hedef_x=5.0, hedef_y=0.0, next_hedef_x=10.0, next_hedef_y=0.0,
                hedef_fresh=True, decision_karar=decision, now=now)


print("== devre dışı → commit yok ==")
ovt = OvertakeManager(base_params(enabled=False))
r = ovt.update(**commit_args())
check("enabled=False komut yok", r.command is None and not r.active)

print("\n== yön komutu var ama engel yok → commit yok ==")
ovt = OvertakeManager(base_params())
args = commit_args(); args["engel_present"] = False
r = ovt.update(**args)
check("engel yok latch yok", r.command is None and not r.active)

print("\n== sollamaya commit ==")
ovt = OvertakeManager(base_params())
r = ovt.update(**commit_args(decision="sol", now=100.0))
check("aktif oldu", r.active)
check("komut sollama;sol", r.command == "sollama;sol;3.00;0.00;engel;1.00", f"-> {r.command}")
check("return_dist≈5.79", abs(r.return_dist_m - 5.79) < 0.05, f"-> {r.return_dist_m}")
check("event basla", r.event is not None and r.event[0] == "basla")

print("\n== aktifken engeli geçmeden: tutar, refresh ==")
r = ovt.update(rx=4.0, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=100.2)
check("refresh penceresi içinde komut yok", r.command is None and r.active, f"-> {r.command}")
r = ovt.update(rx=4.5, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=101.5)
check("refresh sonrası sollama tazelenir", r.command == "sollama;sol;3.00;0.00;engel;1.00",
      f"-> {r.command}")

print("\n== engeli Ackermann mesafesi kadar geçince → dönüş ==")
r = ovt.update(rx=9.0, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=102.0)
check("dönüş komutu kenar_serbest", r.command == "kenar_serbest;sol;3.00;0.00;don;1.00",
      f"-> {r.command}")
check("artık aktif değil", not r.active)
check("event donus", r.event is not None and r.event[0] == "donus")

print("\n== erken dönüş YOK (gap < return_dist) ==")
ovt = OvertakeManager(base_params())
ovt.update(**commit_args(now=200.0))
r = ovt.update(rx=5.0, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=200.5)
# gap = 5-3 = 2 < 5.79 → dönmemeli
check("gap<return → dönmez", r.active and "kenar_serbest" not in (r.command or ""))

print("\n== zaman aşımı → fallback dönüş ==")
ovt = OvertakeManager(base_params())
ovt.update(**commit_args(now=300.0))
r = ovt.update(rx=3.5, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=316.0)
check("zaman aşımı kenar_serbest", "kenar_serbest" in (r.command or ""), f"-> {r.command}")
check("zaman aşımı event", r.event is not None and r.event[0] == "zaman_asimi")

print("\n== reset() sollamayı iptal eder (node acildurus guard'ı buna dayanır) ==")
ovt = OvertakeManager(base_params())
ovt.update(**commit_args(now=400.0))
check("commit sonrası aktif", ovt.active)
ovt.reset()
check("reset sonrası pasif", not ovt.active)
r = ovt.update(rx=4.0, ry=0.0, yaw=0.0, engel_present=False, d_overall=float("inf"),
               d_center=float("inf"), angle_deg=0.0, hedef_x=5.0, hedef_y=0.0,
               next_hedef_x=10.0, next_hedef_y=0.0, hedef_fresh=True,
               decision_karar="normal", now=400.5)
check("reset sonrası normal'de komut yok", r.command is None and not r.active)

print("\n" + "=" * 50)
if _fail:
    print(f"FAIL: {len(_fail)} test başarısız")
    for f in _fail:
        print(" -", f)
    sys.exit(1)
print("OK: tüm sollama testleri geçti")
sys.exit(0)
