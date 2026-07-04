#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_c1_engel_kacinma.py — engel geometrisi + H-B e-stop seçimi (control.py).

ROS'suz: control.py'nin tepe-import'ları (rospy/can/ROS msg'leri) stub'lanır,
sonra saf fonksiyon GERÇEK koddan import edilir (kopya değil). Repo'daki diğer
test_*.py'ler gibi `python3 control/test_c1_engel_kacinma.py` ile koşar.

H-A/H-B (2026-06-24) sonrası: control sentetik yanal-offset manevrası YAPMAZ;
kaçınma planlayıcıda, control düz takip eder. Burada test edilen tek saf fonksiyon
`select_blocking_obstacle` — H-B doğrudan e-stop'un dead-ahead dar-koridor seçimi.
(Eski offset apeks/gap fonksiyonları H-A ile kaldırıldı.)
"""
import sys
import types

# --- ROS tepe-import'larını stub'la (fonksiyon bunları RUNTIME'da kullanmaz) ---
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
import control as C  # noqa: E402  (import = H-A/H-B sonrası modül smoke-test)

_fail = 0


def chk(cond, msg):
    global _fail
    if not cond:
        _fail += 1
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")


import math  # noqa: E402

print("== modül smoke-test (H-A/H-B sonrası import + sabitler) ==")
chk(hasattr(C, 'select_blocking_obstacle'), "select_blocking_obstacle var")
chk(hasattr(C, 'ackermann_path_clears'), "ackermann_path_clears var (§12.14)")
chk(not hasattr(C, 'obstacle_offset_target'), "obstacle_offset_target KALDIRILDI (H-A)")
chk(not hasattr(C, 'obstacle_passed'), "obstacle_passed KALDIRILDI (H-A)")
for k in ('ESTOP_HARD_M', 'ESTOP_FWD_M', 'ESTOP_CORRIDOR_M', 'ESTOP_CHECK_CORRIDOR_M',
          'ESTOP_SAFE_RADIUS', 'ESTOP_RELEASE_S', 'OBSTACLE_TIMEOUT', 'OBSTACLE_FWD_MIN',
          'SHARP_TURN_DEG', 'SHARP_LOOKAHEAD'):
    chk(hasattr(C, k), f"sabit {k} tanımlı")

print("== select_blocking_obstacle (H-B e-stop: dead-ahead dar koridor) ==")
sel = C.select_blocking_obstacle
FWD_MIN, FWD, COR = C.OBSTACLE_FWD_MIN, C.ESTOP_FWD_M, C.ESTOP_CORRIDOR_M
print(f"   (FWD_MIN={FWD_MIN}, ESTOP_FWD_M={FWD}, ESTOP_CORRIDOR_M={COR})")
chk(sel([(2.0, 0.0)], FWD_MIN, FWD, COR) == (2.0, 0.0), "dead-ahead yakın engel → tetikler")
chk(sel([], FWD_MIN, FWD, COR) is None, "boş → None (temiz)")
chk(sel([(0.1, 0.0)], FWD_MIN, FWD, COR) is None, "gövde dibi (fwd_min altı) elenir")
chk(sel([(FWD + 1.0, 0.0)], FWD_MIN, FWD, COR) is None, "ESTOP_FWD_M ötesi → tetiklemez (henüz uzak)")
chk(sel([(2.0, 1.0)], FWD_MIN, FWD, COR) is None,
    "dar koridor DIŞI (yana açılmış araç/yandan geçen engel) → tetiklemez = reroute'u öldürmez")
chk(sel([(2.0, 0.4)], FWD_MIN, FWD, COR) == (2.0, 0.4), "koridor içi (|lat|<0.7) → tetikler")
chk(sel([(2.2, 0.1), (1.2, -0.2)], FWD_MIN, FWD, COR) == (1.2, -0.2),
    "birden çok → en yakın (fwd min) seçilir")

print("== ackermann_path_clears (§12.14: keskin dönüş aradan geçer mi) ==")
acl = C.ackermann_path_clears
L, SAFE = C.WHEELBASE, C.ESTOP_SAFE_RADIUS
print(f"   (WHEELBASE={L}, ESTOP_SAFE_RADIUS={SAFE})")
# Düz gidiş: dead-center duba → çarpar (geçmez); yana açık → geçer
chk(acl(2.4, 0.0, 0.0, L, SAFE) is False, "düz + dead-center duba → GEÇMEZ (dur)")
chk(acl(2.4, 1.2, 0.0, L, SAFE) is True, "düz + yana 1.2m duba → geçer (yanından)")
# CANLI senaryo: duba (2.4, -0.27). Yumuşak 12° marjinal vs keskin dönüş geçer.
# 2026-07-04 Bee1 hizalaması: WHEELBASE 1.78→1.86 ile 25°'nin açıklığı 0.8997 m'ye
# düştü (SAFE=0.9 altı) → eşdeğer "keskin" dönüş artık ≥26°. Uzun dingil = geniş yay.
soft = acl(2.4, -0.27, 12.0, L, SAFE)
sharp = acl(2.4, -0.27, 26.0, L, SAFE)
print(f"   canlı duba (2.4,-0.27): 12°→{'geçer' if soft else 'DUR'}, 26°→{'geçer' if sharp else 'DUR'}")
chk(acl(2.4, -0.27, 25.0, L, SAFE) is False, "25° Bee1 dingiliyle sınır altı (0.8997<0.9) → DUR")
chk(sharp is True, "keskin 26° sol dönüş → yay dubayı kaçırır → GEÇER (erken durma yok)")
# Sağ engele yeterince keskin sol dönüş → uzaklaşır → geçer (20°=0.86m marjinal DUR; 28°=0.99m geçer)
chk(acl(2.0, -0.5, 20.0, L, SAFE) is False, "sağdaki engel, 20° (açıklık 0.86<0.9) → marjinal DUR")
chk(acl(2.0, -0.5, 28.0, L, SAFE) is True, "sağdaki engel, 28° (açıklık ≥0.9) → GEÇER")
# Monotonluk: dönüş keskinleştikçe açıklık artar (maks komut açısı = MAX_STEER_ANGLE≈28.95°)
chk(acl(2.4, -0.27, C.MAX_STEER_ANGLE, L, SAFE) is True,
    f"duba (2.4,-0.27) maks açı ({C.MAX_STEER_ANGLE:.2f}°) → kesin geçer")
# Tam üstüne doğru hafif dönüş + yakın merkez duba → çarpar
chk(acl(1.5, 0.0, 5.0, L, SAFE) is False, "hafif dönüş + yakın merkez duba → GEÇMEZ")

print("\n== çok-koni yay-bazlı seçim (incele düzeltmesi 2026-07-04) ==")
chk(hasattr(C, 'select_arc_blocking_obstacle'), "select_arc_blocking_obstacle var")
sab = C.select_arc_blocking_obstacle
# Gölgeleme senaryosu: yakın-yanal koni (1.0, 1.3) yay tarafından geçilir; asıl
# tehlike uzaktaki dead-ahead (2.0, 0.1). Eski kod yakını seçip e-stop'u susturuyordu.
chk(sab([(1.0, 1.3), (2.0, 0.1)], 0.3, 2.5, 1.5, 0.0, L, SAFE) == (2.0, 0.1),
    "yakın-yanal koni dead-ahead koniyi GÖLGELEMEZ → (2.0,0.1) seçilir (e-stop)")
chk(sab([(1.0, 1.3)], 0.3, 2.5, 1.5, 0.0, L, SAFE) is None,
    "tek yanal koni (1.3m) yayca geçilir → None (yanlış alarm yok)")
chk(sab([(2.4, -0.27), (1.0, 1.2)], 0.3, 2.5, 1.5, 26.0, L, SAFE) is None,
    "26° keskin dönüş iki koniyi de kaçırır → None (erken durma yok)")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
    sys.exit(0)
print(f"{_fail} TEST BAŞARISIZ ❌")
sys.exit(1)
