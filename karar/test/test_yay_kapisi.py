"""Yay-kapısı (arc gate) birim testleri — ROS yok, saf geometri.

Acil bandının bisiklet-modeli farkındalığı (2026-07-15):
  • Canlı vaka 20260713T173335Z: 41° sağ-yanda 1.12 m bordür, araç sola
    dönerken acildurus'u 2 dk kilitledi → dönüşte bant temiz olmalı.
  • Canlı vaka 20260713T162404Z: ölü-merkez koni 1.15 m → HER direksiyonda
    bant içinde kalmalı (acil DOĞRU).

Çalıştır:
    cd talos26_ws/karar && python3 -m test.test_yay_kapisi
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from obstacle_fusion import (ObstacleFusionParams, fuse_obstacles,
                             ArcGateParams, ackermann_path_clears,
                             arc_blocking_distance)

_INF = float("inf")
P = ObstacleFusionParams(corridor_half_w_m=1.0, forward_min_m=0.0,
                         forward_max_m=30.0, side_forward_max_m=8.0)
GP = ArcGateParams()  # 1.86 / 0.75 / 1.76 / 2.34 — control.py ile aynı

fails = []


def check(name, cond):
    if cond:
        print(f"  ✓ {name}")
    else:
        fails.append(name)
        print(f"  ✗ {name}")


print("T1: Düz gidiş — yanal ayrım bant yarı genişliğiyle")
check("T1 merkez 3m bant içinde", not ackermann_path_clears(3.0, 0.0, 0.0, GP))
check("T1 lat 0.74 < 0.75 bant içinde", not ackermann_path_clears(3.0, 0.74, 0.0, GP))
check("T1 lat 0.80 > 0.75 temiz", ackermann_path_clears(3.0, 0.80, 0.0, GP))

print("\nT2: Canlı vaka 173335 — sağ-yan nesne (fwd 0.84, lat −0.74), sola dönüş")
pts = [(0.84, -0.74, 0.0)]
check("T2 düz giderken bloklar (lat 0.74<0.75)",
      math.isfinite(arc_blocking_distance(pts, 0.0, P, GP)))
for d in (5.0, 15.0, 28.95):
    check(f"T2 sola {d}° dönüşte temiz",
          arc_blocking_distance(pts, d, P, GP) == _INF)

print("\nT3: Canlı vaka 162404 — ölü-merkez koni 1.15 m, her direksiyonda bloklu")
pts = [(1.15, 0.0, 0.0)]
for d in (0.0, 10.0, 28.95, -28.95):
    check(f"T3 steer {d}° → d_arc=1.15",
          abs(arc_blocking_distance(pts, d, P, GP) - 1.15) < 1e-9)

print("\nT4: d_center DEĞİŞMEDİ — yan nesne planlama bantlarında görünmeye devam eder")
f = fuse_obstacles([(0.84, -0.74, 0.0)], P)
check("T4 d_center sonlu (dur/reroute/yavasla hâlâ görür)", math.isfinite(f.d_center))
check("T4 d_arc sola dönüşte inf (yalnız ACİL bandı gevşer)",
      arc_blocking_distance([(0.84, -0.74, 0.0)], 15.0, P, GP) == _INF)

print("\nT5: Engel yarı genişliği (hw) banda eklenir")
# lat 1.0, hw 0 → düzde temiz (1.0 > 0.75); hw 0.3 → efektif bant 1.05 → bloklu
check("T5 hw=0 temiz", arc_blocking_distance([(3.0, 1.0, 0.0)], 0.0, P, GP) == _INF)
check("T5 hw=0.3 bloklu", math.isfinite(arc_blocking_distance([(3.0, 1.0, 0.3)], 0.0, P, GP)))

print("\nT6: Dönüşte ÖN DIŞ KÖŞE süpürmesi — burun payı dış kenarı büyütür")
# Sola dönüşte (28.95° → R=3.36) dış-ön köşe sağa taşar: sağda yakın nesne
# düz bantta temiz olsa da süpürme halkasına girebilir.
delta = 28.95
R = GP.wheelbase_m / math.tan(math.radians(delta))
r_dis = math.hypot(R + GP.half_width_m, GP.nose_m)
# ICR'den r_dis−0.05 uzaklıkta, sağ-önde bir nokta üret (halka içinde)
ang = math.radians(70.0)
fx = (r_dis - 0.05) * math.sin(ang) - GP.sensor_to_ra_m
fy = R - (r_dis - 0.05) * math.cos(ang)
check("T6 halka içindeki nokta bloklu", not ackermann_path_clears(fx, fy, delta, GP))
# Halkanın hemen dışı (ICR'den r_dis+0.2) → temiz
fx2 = (r_dis + 0.2) * math.sin(ang) - GP.sensor_to_ra_m
fy2 = R - (r_dis + 0.2) * math.cos(ang)
check("T6 halka dışındaki nokta temiz", ackermann_path_clears(fx2, fy2, delta, GP))

print("\nT7: İç kenar — dönüşün içinde kalan nesne temiz")
# Sola 28.95° dönüşte ICR (−1.76, +3.36); ICR yakınındaki nesne (iç bölge) temiz
check("T7 iç bölge temiz", ackermann_path_clears(-0.5, 3.0, delta, GP))

print("\nT8: Arkadaki nokta yok sayılır (fwd ≤ forward_min)")
check("T8 fwd<0 yok sayılır", arc_blocking_distance([(-1.0, 0.0, 0.0)], 0.0, P, GP) == _INF)

print("\nT9: ArcGateParams.from_cfg — eksik anahtarlar default'a düşer")
gp2 = ArcGateParams.from_cfg({"half_width_m": 0.9})
check("T9 half_width override", gp2.half_width_m == 0.9)
check("T9 wheelbase default", gp2.wheelbase_m == 1.86)
check("T9 enabled default", gp2.enabled is True)

# ---------------------------------------------------------------- #
# BT katmanı (py_trees varsa): EngelCokYakin d_arc okur; release d_arc'a bakar
# ---------------------------------------------------------------- #
try:
    import py_trees  # noqa: F401
    _HAS_PT = True
except Exception:
    _HAS_PT = False

if _HAS_PT:
    from bb import Blackboard
    from behaviors.conditions import EngelCokYakin
    from behaviors.actions import ReleaseEmergencyIfClear
    from py_trees.common import Status

    print("\nT10: EngelCokYakin d_arc okur (d_center değil)")
    bb = Blackboard()
    bb.write(engel_present=True, engel_d_center=1.12, engel_d_arc=_INF)
    c = EngelCokYakin(bb, 1.2)
    check("T10 bant temiz → acil YOK (d_center 1.12 olsa da)", c.update() == Status.FAILURE)
    bb.write(engel_d_arc=1.12)
    check("T10 bant içi 1.12 < 1.2 → acil VAR", c.update() == Status.SUCCESS)

    print("\nT11: Release — direksiyon nesneyi temizleyen yaya dönünce mühür çözülür")
    bb = Blackboard()
    bb.state.emergency_latched = True
    r = ReleaseEmergencyIfClear(bb, release_clear_ticks=2, yaya_esik=3.0, engel_esik=1.8)
    # bant hâlâ bloklu → streak sıfır
    bb.write(engel_present=True, engel_d_center=1.12, engel_d_arc=1.12)
    r.update()
    check("T11 bloklu → mühür durur", bb.state.emergency_latched is True)
    # direksiyon kaçış yayına döndü → d_arc=inf (d_center hâlâ 1.12!)
    bb.write(engel_d_arc=_INF)
    r.update(); rel = r.update()
    check("T11 2 tick temiz → mühür çözüldü", bb.state.emergency_latched is False)
    # d_center inf iken (sensör verisi eksik) present=True → güvenli tarafta kal
    bb.state.emergency_latched = True
    bb.state.emergency_clear_streak = 0
    bb.write(engel_present=True, engel_d_center=_INF, engel_d_arc=_INF)
    r.update(); r.update(); r.update()
    check("T11 veri eksikken çözülMEZ", bb.state.emergency_latched is True)
else:
    print("\n(py_trees yok — T10/T11 BT testleri atlandı)")

print("\n" + "=" * 40)
if fails:
    print(f"FAIL: {len(fails)} test başarısız: {fails}")
    sys.exit(1)
print("OK: tüm yay-kapısı testleri geçti")
sys.exit(0)
