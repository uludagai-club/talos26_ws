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
_stub_module('std_msgs.msg', ['Bool', 'Float32', 'String'])
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
          'ESTOP_BANT_YARIM_M', 'ARAC_GENISLIK_M', 'LIDAR_ARKA_AKS_M', 'ARAC_BURUN_M',
          'ESTOP_RELEASE_S', 'OBSTACLE_TIMEOUT', 'OBSTACLE_FWD_MIN',
          'SHARP_TURN_DEG', 'SHARP_LOOKAHEAD'):
    chk(hasattr(C, k), f"sabit {k} tanımlı")
chk(not hasattr(C, 'ESTOP_SAFE_RADIUS'),
    "ESTOP_SAFE_RADIUS KALDIRILDI (2026-07-04: 2B süpürme bandı geometrisi)")

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

print("== ackermann_path_clears (2B süpürme bandı: genişlik + arka-aks ofseti + burun) ==")
acl = C.ackermann_path_clears
L = C.WHEELBASE
HALF, A, NOSE = C.ESTOP_BANT_YARIM_M, C.LIDAR_ARKA_AKS_M, C.ARAC_BURUN_M
GEO = (L, HALF, A, NOSE)
print(f"   (WHEELBASE={L}, BANT_YARIM={HALF}, LIDAR_ARKA_AKS={A}, BURUN={NOSE})")
# Düz gidiş: bant = ±(genişlik/2 + koni yarıçapı) = ±0.75
chk(acl(2.4, 0.0, 0.0, *GEO) is False, "düz + dead-center duba → GEÇMEZ (dur)")
chk(acl(2.4, 0.8, 0.0, *GEO) is True, "düz + yana 0.8m duba (bant 0.75 dışı) → geçer")
chk(acl(2.4, 0.7, 0.0, *GEO) is False, "düz + yana 0.7m duba (bant içi) → GEÇMEZ")
# CANLI senaryo: duba (2.4, -0.27) SAĞDA; sol dönüş bandı soldan açar.
# Arka-aks ofseti düzeltmesiyle 12° sol bile dubayı bandın DIŞINA atar
# (eski (0,±R)+0.9 modeli 12°'de 'çarpar' diyordu → erken durma buradan geliyordu).
chk(acl(2.4, -0.27, 12.0, *GEO) is True,
    "sağdaki duba, 12° sol dönüş → bant dışı → GEÇER (eski model erken duruyordu)")
chk(acl(2.4, 0.9, 12.0, *GEO) is False,
    "12° sol dönüşün süpürdüğü tarafta (sol önde) duba → bant İÇİ → DUR")
chk(acl(2.0, -0.5, 20.0, *GEO) is True, "sağdaki engel, 20° sol → dış kenar ötesi → GEÇER")
# İç kenar: dönüşün İÇİNDE kalan koni (halkanın iç yarıçapı altında) → geçer
chk(acl(1.0, 2.5, 25.0, *GEO) is True, "25° sol dönüşün İÇİNDE kalan koni → geçer")
chk(acl(1.5, 2.0, 25.0, *GEO) is False, "25° sol dönüş halkasının içindeki koni → DUR")
# Monotonluk: maks açıda sağdaki duba kesin bant dışı
chk(acl(2.4, -0.27, C.MAX_STEER_ANGLE, *GEO) is True,
    f"duba (2.4,-0.27) maks açı ({C.MAX_STEER_ANGLE:.2f}°) → kesin geçer")
# Tam üstüne doğru hafif dönüş + yakın merkez duba → çarpar
chk(acl(1.5, 0.0, 5.0, *GEO) is False, "hafif dönüş + yakın merkez duba → GEÇMEZ")

print("\n== çok-koni yay-bazlı seçim (incele düzeltmesi 2026-07-04) ==")
chk(hasattr(C, 'select_arc_blocking_obstacle'), "select_arc_blocking_obstacle var")
sab = C.select_arc_blocking_obstacle
# Gölgeleme senaryosu: yakın-yanal koni (1.0, 1.3) bant dışı; asıl tehlike
# uzaktaki dead-ahead (2.0, 0.1). Eski kod yakını seçip e-stop'u susturuyordu.
chk(sab([(1.0, 1.3), (2.0, 0.1)], 0.3, 2.5, 1.5, 0.0, *GEO) == (2.0, 0.1),
    "yakın-yanal koni dead-ahead koniyi GÖLGELEMEZ → (2.0,0.1) seçilir (e-stop)")
chk(sab([(1.0, 1.3)], 0.3, 2.5, 1.5, 0.0, *GEO) is None,
    "tek yanal koni (1.3m) bant dışı → None (yanlış alarm yok)")
# 26° sol dönüş: sağdaki (2.4,-0.27) bant dışı AMA soldaki (1.0,1.2) süpürme
# halkasının İÇİNDE (ön dış köşe + arka-aks ofseti — eski model bunu KAÇIRIYORDU).
chk(sab([(2.4, -0.27), (1.0, 1.2)], 0.3, 2.5, 1.5, 26.0, *GEO) == (1.0, 1.2),
    "26° solda: sağdaki geçilir, SOLDAKİ (1.0,1.2) süpürme halkası içinde → seçilir")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
    sys.exit(0)
print(f"{_fail} TEST BAŞARISIZ ❌")
sys.exit(1)
