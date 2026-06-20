"""obstacle_fusion birim testleri — ROS yok, saf geometri.

Çalıştır:
    cd talos26_ws/karar_bt && python3 -m test.test_obstacle_fusion
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from obstacle_fusion import ObstacleFusionParams, fuse_obstacles

_INF = float("inf")
P = ObstacleFusionParams(corridor_half_w_m=1.0, forward_min_m=0.0,
                         forward_max_m=30.0, side_forward_max_m=8.0)

fails = []


def check(name, cond):
    if cond:
        print(f"  ✓ {name}")
    else:
        fails.append(name)
        print(f"  ✗ {name}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


print("T1: Boş liste → temiz")
f = fuse_obstacles([], P)
check("T1 present=False", f.present is False)
check("T1 d_center=inf", f.d_center == _INF)
check("T1 d_left=inf", f.d_left == _INF)
check("T1 count=0", f.count == 0)

print("\nT2: Tam önde 3m engel → merkez dolu, yanlar boş")
f = fuse_obstacles([(3.0, 0.0)], P)
check("T2 present", f.present is True)
check("T2 d_center≈3", approx(f.d_center, 3.0))
check("T2 d_left=inf (yan temiz)", f.d_left == _INF)
check("T2 d_right=inf (yan temiz)", f.d_right == _INF)

print("\nT3: Sol şeritte engel (left=2m, fwd=4) → merkez boş, sol dolu")
f = fuse_obstacles([(4.0, 2.0)], P)
check("T3 d_center=inf", f.d_center == _INF)
check("T3 d_left dolu", math.isfinite(f.d_left))
check("T3 d_right=inf", f.d_right == _INF)
check("T3 present=False (merkez boş)", f.present is False)

print("\nT4: Sağ şeritte engel (left=-2m) → sağ dolu")
f = fuse_obstacles([(4.0, -2.0)], P)
check("T4 d_right dolu", math.isfinite(f.d_right))
check("T4 d_left=inf", f.d_left == _INF)

print("\nT5: Merkez engel + sol şerit boş senaryo (kaçış mümkün)")
# Tam ortada engel: merkez dolu ama yanlar temiz → BT sol/sağ kaçış yapabilir
f = fuse_obstacles([(2.0, 0.2)], P)
check("T5 merkez dolu", math.isfinite(f.d_center))
check("T5 sol temiz", f.d_left == _INF)
check("T5 sağ temiz", f.d_right == _INF)

print("\nT6: Arkadaki engel (fwd<0) yok sayılır")
f = fuse_obstacles([(-3.0, 0.0)], P)
check("T6 present=False", f.present is False)
check("T6 count=0", f.count == 0)

print("\nT7: Çok uzak engel (35m > forward_max) merkez/sayım dışı")
f = fuse_obstacles([(35.0, 0.0)], P)
check("T7 d_center=inf", f.d_center == _INF)
check("T7 count=0", f.count == 0)

print("\nT8: En yakın engel açısı — sağdaki engel pozitif açı")
f = fuse_obstacles([(5.0, -5.0)], P)  # sağda
check("T8 angle>0 (sağ pozitif)", f.angle_deg > 0)
f = fuse_obstacles([(5.0, 5.0)], P)   # solda
check("T8 angle<0 (sol negatif)", f.angle_deg < 0)

print("\nT9: Genişlikli kutu — yarı genişlik koridora taşarsa merkez sayılır")
# centroid left=1.5 (>1.0 koridor) ama kutu yarı genişliği 0.8 → kenar 0.7 < 1.0
f = fuse_obstacles([(3.0, 1.5, 0.8)], P)
check("T9 kutu kenarı koridorda → merkez dolu", math.isfinite(f.d_center))

print("\n" + "=" * 40)
if fails:
    print(f"FAIL: {len(fails)} test başarısız: {fails}")
    sys.exit(1)
print("OK: tüm fusion testleri geçti")
sys.exit(0)
