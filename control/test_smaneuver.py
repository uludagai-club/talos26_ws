#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_smaneuver.py — §18 KESKİN S-MANEVRA kapalı-döngü regresyon testi.

Manevra trajektoriyi değiştirir (kapalı döngü) → kayıt replay'i yetmez; bisiklet
modeliyle simüle edip kontrol edilir:
  1. Manevra TAMAMLANIR (IDLE'a döner, timeout YOK, DÖNGÜ yok).
  2. Dubayı GEÇER (min açıklık ≥ gerekli klirens).
  3. Direksiyon osilasyonu YOK (slew-bounded, ters-dönüş yok).
  4. REGRESYON: eski "sol-WP abeam'e kadar full-lock" tasarımı DÖNGÜ yapar (neden
     swing kapısı ŞART — bunu koruyalım).

control.py'deki _sman_update transition mantığının BİREBİR kopyası (rospy'siz).
Çalıştır: python3 control/test_smaneuver.py
"""
import math
import sys
import types
import os

# Sabitler GERÇEK control.py'den alınır (kopya DEĞİL). ROS tepe-import'ları stub'lanır.
for _name in ('rospy', 'can', 'tf'):
    sys.modules.setdefault(_name, types.ModuleType(_name))
def _stub_module(name, attrs):
    m = types.ModuleType(name)
    for a in attrs:
        setattr(m, a, object)
    sys.modules[name] = m
_stub_module('nav_msgs', []); _stub_module('nav_msgs.msg', ['Odometry'])
_stub_module('std_msgs', []); _stub_module('std_msgs.msg', ['Float32', 'String'])
_stub_module('visualization_msgs', []); _stub_module('visualization_msgs.msg', ['Marker'])
_stub_module('geometry_msgs', []); _stub_module('geometry_msgs.msg', ['Point', 'PoseArray'])
_stub_module('tf.transformations', ['euler_from_quaternion'])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control as C

MAX_STEER = C.MAX_STEER_ANGLE
WHEELBASE = C.WHEELBASE
STEER_RATE_MAX_DEG_S = C.STEER_RATE_MAX_DEG_S
DT = C.LOOP_DT
SMANEUVER_MAX_SWING_DEG = C.SMANEUVER_MAX_SWING_DEG
SMANEUVER_ALIGN_DEG = C.SMANEUVER_ALIGN_DEG
SMANEUVER_TIMEOUT = C.SMANEUVER_TIMEOUT

_fail = 0
def chk(cond, msg):
    global _fail
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def simulate(start, fwd_yaw, left_wp, v_kmh, cone, use_swing_gate=True):
    """control.py _sman_update + slew + bisiklet modelini kapalı-döngü koşar.
    Döner: (sonuç, min_cone_dist, sure_s, sert_ters, max_dsteer)."""
    x, y, yaw = start[0], start[1], fwd_yaw
    v = v_kmh / 3.6
    sx, sy = x, y
    sdir = 1 if (-math.sin(fwd_yaw)*(left_wp[0]-sx) + math.cos(fwd_yaw)*(left_wp[1]-sy)) > 0 else -1
    wp_lat = sdir * (-math.sin(fwd_yaw)*(left_wp[0]-sx) + math.cos(fwd_yaw)*(left_wp[1]-sy))
    phase, phase_t, prev, t = 'TOWARD', 0.0, 0.0, 0.0
    mind, steers = 1e9, []
    while t < 16.0:
        d = (yaw - fwd_yaw + math.pi) % (2*math.pi) - math.pi
        swing = math.degrees(sdir * d)
        lat_prog = sdir * (-math.sin(fwd_yaw)*(x-sx) + math.cos(fwd_yaw)*(y-sy))
        if phase == 'TOWARD':
            tgt = sdir * MAX_STEER
            reached = lat_prog >= wp_lat
            capped = use_swing_gate and swing >= SMANEUVER_MAX_SWING_DEG
            if reached or capped:
                phase, phase_t = 'AWAY', t
        elif phase == 'AWAY':
            tgt = -sdir * MAX_STEER
            if swing <= SMANEUVER_ALIGN_DEG:
                phase = 'IDLE'; tgt = 0.0
        else:
            tgt = 0.0
        if phase != 'IDLE' and (t - phase_t) > SMANEUVER_TIMEOUT:
            return ('TIMEOUT', mind, t, 0, 0.0)
        md = STEER_RATE_MAX_DEG_S * DT
        st = max(prev - md, min(prev + md, tgt))
        st = max(-MAX_STEER, min(MAX_STEER, st)); prev = st
        steers.append(st)
        yaw += v / WHEELBASE * math.tan(math.radians(st)) * DT
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        mind = min(mind, math.hypot(x - cone[0], y - cone[1]))
        t += DT
        if phase == 'IDLE':
            rev = sum(1 for i in range(1, len(steers))
                      if abs(steers[i]-steers[i-1]) > 15 and (steers[i] > 0) != (steers[i-1] > 0))
            mx = max(abs(steers[i]-steers[i-1]) for i in range(1, len(steers)))
            return ('OK', mind, t, rev, mx)
    return ('NOEND', mind, t, 0, 0.0)


# Senaryo: run 124844/135822 — araç (−4,−34.27) yaw0, reroute WP 2.2m sola, duba lane-merkez
START, FYAW, LEFT_WP, CONE = (-4.0, -34.27), 0.0, (0.1, -32.05), (-0.3, -34.04)
CLEAR_NEED = 0.75   # duba r0.15 + araç yarı-gen 0.6

print("== KESKİN S-MANEVRA kapalı-döngü (§18) ==")
for v in (2.5, 4.0):
    res, mind, t, rev, mx = simulate(START, FYAW, LEFT_WP, v, CONE, use_swing_gate=True)
    print(f"  v={v}km/h → {res}  min_açıklık={mind:.2f}m  süre={t:.1f}s  ters-dönüş={rev}  max|Δ/tick|={mx:.1f}°")
    chk(res == 'OK', f"v={v}: manevra TAMAMLANDI (IDLE, timeout/döngü yok)")
    chk(mind >= CLEAR_NEED, f"v={v}: dubayı GEÇER (açıklık {mind:.2f}≥{CLEAR_NEED})")
    chk(rev == 0, f"v={v}: direksiyon ters-dönüş YOK (osilasyonsuz)")
    chk(mx <= STEER_RATE_MAX_DEG_S*DT + 1e-9, f"v={v}: slew-bounded (max|Δ/tick|≤4°)")

print("\n== swing kapısı manevrayı SIKI/HIZLI tutar (yavaş golf-cart'ta aşırı dönmeyi keser) ==")
res_g, _, t_g, _, _ = simulate(START, FYAW, LEFT_WP, 2.5, CONE, use_swing_gate=True)
res_ng, _, t_ng, _, _ = simulate(START, FYAW, LEFT_WP, 2.5, CONE, use_swing_gate=False)
print(f"  v=2.5  swing kapısı AÇIK → {res_g} {t_g:.1f}s  |  KAPALI → {res_ng} {t_ng:.1f}s")
chk(t_ng - t_g > 2.0, f"swing kapısı süreyi belirgin kısaltır ({t_ng:.1f}s→{t_g:.1f}s): WP'ye kadar "
                      f"full-lock yavaş golf-cart'ta ~73° döndürür (aşırı), kapı 45°'de keser")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    raise SystemExit(1)
