#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_oscillation_kill.py — §17 slalom direksiyon oscillation-kill regresyon testi.

KAPSAM (control.py minimal fix, §17):
  1. SLEW-RATE limiti: direksiyon tek tick'te STEER_RATE_MAX_DEG_S·dt'den fazla
     değişemez → flip-flop snap (+9°→−30°) rampaya iner, line-gate bang-bang
     titremesi sönümlenir.
  2. /line BASTIRMA: engel-bağlamında (karar≠normal, TTL'li) /line düzeltmesi
     uygulanmaz → 12° kapısındaki +9°↔−2° bang-bang ortadan kalkar.

rospy/can GEREKMEZ — saf mantık testi (diğer test_*.py gibi standalone).
Çalıştır: python3 control/test_oscillation_kill.py
"""

# Sabitler GERÇEK control.py'den alınır (kopya DEĞİL) → control'de değişirse test
# otomatik uyarlanır, sessizce eskimez. ROS tepe-import'ları stub'lanır (test_c1 deseni).
import sys
import types
import os
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

STEER_RATE_MAX_DEG_S = C.STEER_RATE_MAX_DEG_S
LOOP_DT = C.LOOP_DT
LINE_SUPPRESS_TTL = C.LINE_SUPPRESS_TTL
MAX_STEER = C.MAX_STEER_ANGLE

_fail = 0
def chk(cond, msg):
    global _fail
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def slew_limit(target, prev_cmd, rate=STEER_RATE_MAX_DEG_S, dt=LOOP_DT):
    """control.py run-loop'taki slew-rate clamp'inin aynısı."""
    md = rate * dt
    s = max(prev_cmd - md, min(prev_cmd + md, target))
    return max(-MAX_STEER, min(MAX_STEER, s))


def line_suppressed(now, last_obstacle_ctx_t, ttl=LINE_SUPPRESS_TTL):
    """control.py'deki /line bastırma kapısının aynısı."""
    return (now - last_obstacle_ctx_t) < ttl


print("== SLEW-RATE limiti (§17 fix-2) ==")
md = STEER_RATE_MAX_DEG_S * LOOP_DT
chk(abs(md - 4.0) < 1e-9, f"tick başına maks değişim = {md:.1f}° (=4° @200°/s,50Hz)")
# Flip-flop snap: +9° iken hedef −30° → tek tick'te yalnız 4° inebilir
chk(abs(slew_limit(-30.0, 9.0) - 5.0) < 1e-9, "+9°→hedef−30°: tek tick'te yalnız +5.0°'a (4° rampa)")
# +MAX→−MAX tam ters: kaç tick? (2·MAX_STEER/4°; 28.95° için 15 tick = 0.30 s)
# 2026-07-04: uçlar literal ±30 idi; MAX_STEER_ANGLE artık ackermann'dan ≈28.95 —
# clamp yüzünden −30'a hiç ulaşılamıyordu, uçlar sabitten türetildi.
import math as _math
prev = MAX_STEER; ticks = 0
while prev > -MAX_STEER + 1e-6 and ticks < 100:
    prev = slew_limit(-MAX_STEER, prev); ticks += 1
_beklenen = _math.ceil(2.0 * MAX_STEER / md)
chk(ticks == _beklenen,
    f"+{MAX_STEER:.2f}°→−{MAX_STEER:.2f}° tam ters {ticks} tick'te ({ticks*LOOP_DT:.2f}s) — ani snap DEĞİL")
# Düz pursuit (küçük değişim) etkilenmez
chk(abs(slew_limit(9.4, 9.3) - 9.4) < 1e-9, "düz pursuit Δ0.1° → değiştirilmeden geçer")

print("\n== /line BASTIRMA (§17 fix-1, PRIMARY) ==")
chk(line_suppressed(now=100.0, last_obstacle_ctx_t=100.0), "engel kararı henüz geldi (Δ0s) → bastır")
chk(line_suppressed(now=101.5, last_obstacle_ctx_t=100.0), "engel kararından 1.5s sonra (<TTL) → hâlâ bastır (normal blip korunur)")
chk(not line_suppressed(now=102.5, last_obstacle_ctx_t=100.0), "2.5s sonra (>TTL) → /line tekrar aktif (cruise lane-keep)")

print("\n== ENTEGRE: kayıtlı chatter'ı sönümle (§17 124844 imzası) ==")
# control_20260626_124844 t≈14.3–15.6 örüntüsü: /line gate 12°'de +9°↔−2° bang-bang.
# Eski steer (kayıttan, line ON/OFF toggling):
old = [8.6, -0.6, 8.7, 8.9, -0.3, -2.8, 9.1, 9.4, -2.0, -2.2, 9.6, 9.9, -1.4, 16.4, -2.0]
def rev_count(s):
    return sum(1 for i in range(1, len(s)) if abs(s[i]-s[i-1]) > 15 and (s[i] > 0) != (s[i-1] > 0))
def max_jump(s):
    return max(abs(s[i]-s[i-1]) for i in range(1, len(s)))
old_rev, old_jmp = rev_count(old), max_jump(old)
# YENİ: engel-bağlamı (karar=slow) → /line bastırıldı → hedef = pure pursuit (~+9°),
# üstüne slew limit. PP baz çizgisini +9° kabul edip (kayıtta line OFF anları)
# slew uygula:
pp_base = 9.0
new = []; prev = old[0]
for _ in old:
    prev = slew_limit(pp_base, prev)   # /line bastırıldı → hep pp baz çizgisi
    new.append(prev)
new_rev, new_jmp = rev_count(new), max_jump(new)
print(f"  ESKİ : ters-dönüş={old_rev}  max|Δ/tick|={old_jmp:.1f}°  aralık[{min(old):+.0f},{max(old):+.0f}]")
print(f"  YENİ : ters-dönüş={new_rev}  max|Δ/tick|={new_jmp:.1f}°  aralık[{min(new):+.0f},{max(new):+.0f}]")
chk(old_rev > 0, "ESKİ: chatter mevcut (ters-dönüş>0)")
chk(new_rev == 0, "YENİ: ters-dönüş YOK (bang-bang gitti)")
chk(new_jmp <= md + 1e-9, f"YENİ: max|Δ/tick| ≤ slew limiti ({md:.0f}°)")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    raise SystemExit(1)
