#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_c1_engel_kacinma.py — C1 engel-farkında kaçınma geometrisi (control.py).

ROS'suz: control.py'nin tepe-import'ları (rospy/can/ROS msg'leri) stub'lanır,
sonra saf fonksiyonlar GERÇEK koddan import edilir (kopya değil). Repo'daki
diğer test_*.py'ler gibi `python3 control/test_c1_engel_kacinma.py` ile koşar.

Doğrulananlar:
  - select_blocking_obstacle: koridor/ileri filtresi + en yakını seçme
  - obstacle_offset_target: merkez duba eski 1.8m ile birebir, off-center kayma,
    hard-max clamp, ters-taraf büyümesi
  - obstacle_passed: başlangıç yönünde gap eşiği (geçti/geçmedi)
"""
import sys
import math
import types

# --- ROS tepe-import'larını stub'la (fonksiyonlar bunları RUNTIME'da kullanmaz) ---
for _name in ('rospy', 'can', 'tf'):
    sys.modules.setdefault(_name, types.ModuleType(_name))


def _stub_module(name, attrs):
    m = types.ModuleType(name)
    for a in attrs:
        setattr(m, a, object)
    sys.modules[name] = m


_stub_module('nav_msgs', [])
_stub_module('nav_msgs.msg', ['Odometry'])
_stub_module('std_msgs', [])
_stub_module('std_msgs.msg', ['Float32', 'String'])
_stub_module('visualization_msgs', [])
_stub_module('visualization_msgs.msg', ['Marker'])
_stub_module('geometry_msgs', [])
_stub_module('geometry_msgs.msg', ['Point', 'PoseArray'])
_stub_module('tf.transformations', ['euler_from_quaternion'])

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control as C  # noqa: E402

AVOID = C.AVOID_OFFSET_MAX
HARD = C.OBSTACLE_OFFSET_HARD_MAX

_fail = 0


def chk(cond, msg):
    global _fail
    status = "OK  " if cond else "FAIL"
    if not cond:
        _fail += 1
    print(f"  [{status}] {msg}")


print("== select_blocking_obstacle ==")
# fwd_min=0.3, fwd_max=12, corridor=1.2
sel = C.select_blocking_obstacle
chk(sel([(5.0, 0.0)], 0.3, 12.0, 1.2) == (5.0, 0.0), "tek merkez duba seçilir")
chk(sel([], 0.3, 12.0, 1.2) is None, "boş liste → None")
chk(sel([(0.1, 0.0)], 0.3, 12.0, 1.2) is None, "fwd_min altı (gövde) elenir")
chk(sel([(20.0, 0.0)], 0.3, 12.0, 1.2) is None, "fwd_max üstü (uzak) elenir")
chk(sel([(5.0, 2.0)], 0.3, 12.0, 1.2) is None, "koridor dışı (yan şerit) elenir")
chk(sel([(8.0, 0.1), (4.0, -0.2), (6.0, 0.0)], 0.3, 12.0, 1.2) == (4.0, -0.2),
    "birden çok aday → en yakın (fwd min) seçilir")
chk(sel([(2.0, 5.0), (3.0, -0.3)], 0.3, 12.0, 1.2) == (3.0, -0.3),
    "koridor dışı yakın olsa da içteki seçilir")

print("== obstacle_offset_target (clearance=AVOID_OFFSET_MAX) ==")
off = C.obstacle_offset_target
# Merkez duba: eski sabit dir*AVOID ile BİREBİR (geriye uyum).
chk(abs(off(0.0, +1, AVOID, HARD) - (+AVOID)) < 1e-9, f"merkez duba, SOL → +{AVOID} (eski davranış)")
chk(abs(off(0.0, -1, AVOID, HARD) - (-AVOID)) < 1e-9, f"merkez duba, SAĞ → -{AVOID} (eski davranış)")
# Engel sağda (lat<0), SOLA kaç: hedef = lat + 1*AVOID → merkezden biraz az |offset|
chk(abs(off(-0.5, +1, AVOID, HARD) - (-0.5 + AVOID)) < 1e-9, "engel sağda, SOL kaç → offset = lat+AVOID")
# Ayrım daima clearance: |offset - lat| == AVOID (clamp'e girmediği sürece)
for lat in (-0.4, -0.2, 0.0, 0.2, 0.4):
    for d in (+1, -1):
        o = off(lat, d, AVOID, HARD)
        if abs(o) < HARD - 1e-6:  # clamp'e değmediyse
            chk(abs(abs(o - lat) - AVOID) < 1e-6,
                f"lat={lat:+.1f} dir={d:+d}: |offset-lat|={abs(o-lat):.2f}≈clearance({AVOID})")
# Hard-max clamp: ters tarafta büyür ama sınırlanır
big = off(+1.0, +1, AVOID, HARD)  # lat=+1.0, SOL (engele doğru) → AVOID+1.0 > HARD
chk(abs(big - HARD) < 1e-9, f"ters-taraf büyüme hard-max'a ({HARD}) clamp'lenir")

print("== obstacle_passed ==")
# Başlangıç +x yönünde (dir=(1,0)), engel (10,0)'da, clearance=2.0
passed_fn = C.obstacle_passed
p, g = passed_fn(7.0, 0.0, 10.0, 0.0, 1.0, 0.0, 2.0)
chk((not p) and abs(g - (-3.0)) < 1e-9, "engelin 3m gerisinde → geçmedi (gap=-3)")
p, g = passed_fn(11.0, 0.0, 10.0, 0.0, 1.0, 0.0, 2.0)
chk((not p) and abs(g - 1.0) < 1e-9, "engeli 1m geçti ama clearance(2) altında → geçmedi")
p, g = passed_fn(12.5, 0.0, 10.0, 0.0, 1.0, 0.0, 2.0)
chk(p and abs(g - 2.5) < 1e-9, "engeli 2.5m geçti → GEÇTİ")
# Diagonal yön (45°): dir birim, ileri projeksiyon
d = (math.cos(math.radians(45)), math.sin(math.radians(45)))
p, g = passed_fn(10.0, 10.0, 5.0, 5.0, d[0], d[1], 2.0)
chk(p and abs(g - (5 * math.sqrt(2))) < 1e-6, "45° yön: gap = 5√2 ≈ 7.07 → GEÇTİ")

print()
if _fail == 0:
    print("TÜM C1 TESTLERİ GEÇTİ ✅")
    sys.exit(0)
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    sys.exit(1)
