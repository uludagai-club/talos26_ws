#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_estop_deadlock.py — SOFT (Ackermann-yay) e-stop deadlock fix regresyonu.

CANLI BULGU (run 214118): slalomda 2. duba ~2.5 m tam karşıda (yanal +0.22m)
iken SOFT e-stop kilitlendi ve BİR DAHA AÇILMADI. Kök-neden: latch aktifken
ana döngü steer=0 gönderiyordu → release kontrolü (ackermann_path_clears) DÜZ
yayla yapılıyor → dead-ahead duba düz yayı asla bırakmıyor → kalıcı tam fren.

Fix: e-stop iki kademeye ayrıldı.
  • HARD floor (<ESTOP_HARD_M, dar koridor) → KOŞULSUZ tam fren (_estop_hard=True).
  • SOFT Ackermann-yay (1.0..2.5m) → tam fren YERİNE emekle + S-manevra/pursuit
    direksiyonu sürer; dönen yay engeli kaçırınca latch debounce ile BIRAKILIR.
  • Veri STALE → güvenli tarafa kaç (soft iken bile _estop_hard=True).

_update_estop GERÇEK control.py'den çağrılır (kopya değil); stub `self` ile.
Çalıştır: python3 control/test_estop_deadlock.py
"""
import math
import os
import sys
import time
import types
import threading

# ROS tepe-import'larını stub'la, gerçek control.py'yi import et.
for _name in ('rospy', 'can', 'tf'):
    sys.modules.setdefault(_name, types.ModuleType(_name))
def _stub_module(name, attrs):
    m = types.ModuleType(name)
    for a in attrs:
        setattr(m, a, object)
    sys.modules[name] = m
_stub_module('nav_msgs', []); _stub_module('nav_msgs.msg', ['Odometry'])
_stub_module('std_msgs', []); _stub_module('std_msgs.msg', ['Bool', 'Float32', 'String'])
_stub_module('visualization_msgs', []); _stub_module('visualization_msgs.msg', ['Marker'])
_stub_module('geometry_msgs', []); _stub_module('geometry_msgs.msg', ['Point', 'PoseArray'])
_stub_module('tf.transformations', ['euler_from_quaternion'])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control as C

_fail = 0
def chk(cond, msg):
    global _fail
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def make_stub(points, prev_steer=0.0, fresh=True):
    """_update_estop'un dokunduğu minimum stub `self`."""
    s = types.SimpleNamespace()
    s._obstacle_lock = threading.Lock()
    s._obstacle_points = list(points)
    s._obstacle_time = time.time() if fresh else (time.time() - 10.0)
    s._estop_active = False
    s._estop_hard = False
    s._estop_clear_since = None
    s._prev_cmd_steer = prev_steer
    s.logger = types.SimpleNamespace(log=lambda *a, **k: None)
    return s

def estop(s):
    return C.CANWaypointFollower._update_estop(s)


# Gerçek geometri: deadlock anındaki duba (gövde çerçevesi). Log "2.5m" gösterdi
# ama bu yuvarlama; ESTOP_FWD_M=2.5 sınırı dışlayıcı (fwd<2.5) → 2.4m kullan.
CONE = (2.4, 0.22)
HARD = (0.8, 0.10)   # < ESTOP_HARD_M (1.0), dar koridor → hard floor

print("== SOFT e-stop: dead-ahead duba (2.4m, +0.22m) ==")
s = make_stub([CONE], prev_steer=0.0)
r1 = estop(s)
chk(r1 is True, "duba yayda → e-stop tetiklenir")
chk(s._estop_active is True, "latch aktif")
chk(s._estop_hard is False, "SOFT kademe (hard floor DEĞİL) → ana döngü emekler, tam fren basmaz")

print("\n== DEADLOCK kanıtı: steer=0 sabitken ASLA açılmaz (eski hata) ==")
s2 = make_stub([CONE], prev_steer=0.0)
estop(s2)
held = all(estop(s2) for _ in range(5))   # düz yayla 5 tick daha
chk(held is True and s2._estop_active is True,
    "steer=0 → 5 tick boyunca latch tutar (düz yay dubayı bırakmıyor = deadlock)")

print("\n== FIX: full-lock direksiyon (S-manevra) → yay açılır → release ==")
s3 = make_stub([CONE], prev_steer=0.0)
estop(s3)                       # latch
s3._prev_cmd_steer = -30.0      # S-manevra/pursuit full-lock sağ (engeli sollar)
r_clear = estop(s3)             # bu tick yay AÇIK → debounce başlar (hâlâ True)
chk(s3._estop_active is True and s3._estop_clear_since is not None,
    "yay açıldı → release debounce başladı (henüz tam bırakmadı)")
s3._estop_clear_since -= (C.ESTOP_RELEASE_S + 0.1)   # debounce süresi geçti say
r_released = estop(s3)
chk(r_released is False and s3._estop_active is False,
    "debounce sonrası latch BIRAKILDI → araç devam eder (deadlock çözüldü)")

print("\n== HARD floor: çok yakın (<1.0m) → KOŞULSUZ tam fren ==")
s4 = make_stub([HARD], prev_steer=-30.0)   # full-lock OLSA BİLE
r_hard = estop(s4)
chk(r_hard is True, "hard floor engeli → e-stop")
chk(s4._estop_hard is True, "HARD kademe → direksiyondan bağımsız tam fren (ana döngü continue)")

print("\n== STALE veri: SOFT latch iken donunca güvenli tarafa kaç ==")
s5 = make_stub([CONE], prev_steer=0.0)
estop(s5)
chk(s5._estop_hard is False, "önce SOFT")
s5._obstacle_time = time.time() - 10.0      # tampon dondu (stale)
r_stale = estop(s5)
chk(r_stale is True, "stale → latch KORUNUR")
chk(s5._estop_hard is True, "stale → SOFT iken bile HARD'a yükselt (köre emekleme YOK)")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
    sys.exit(0)
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    sys.exit(1)
