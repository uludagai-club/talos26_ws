#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YayaLevhaKapisi birim testleri — ROS yok.

    python3 karar/test/test_yaya_levha_kapisi.py

Kapı: çizgi modeline (/yaya_gecidi/model) yalnız yaya geçidi LEVHASI
(/yaya_gecidi) görülünce güven. Yaşam döngüsü:
  • levha yok → kapı kapalı (FAILURE) → çizgi modeli yok sayılır
  • levha menzil içinde → kapı açılır (SUCCESS) + çizgi FSM sıfırlanır
  • levha menzil dışı → açılmaz
  • çizgi FSM 'released' → kapan
  • levhayı geç (dünya-çapası pass_behind gerisinde) → kapan
  • fail-safe arm_max_s → kapan
  • kapandıktan sonra grace içinde yeniden silahlanmaz
  • enabled=False → pass-through (daima SUCCESS)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import behaviors.actions as actions_mod  # noqa: E402
from behaviors.actions import YayaLevhaKapisi  # noqa: E402
from bb import Blackboard  # noqa: E402
from py_trees.common import Status  # noqa: E402

failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def ilerle(self, dt):
        self.t += dt


_clock = FakeClock()
actions_mod.time = _clock  # kapı `time.time()` → sahte saat


def make_kapi(enabled=True, arm_menzil_m=10.0, pass_behind_m=3.0,
              arm_max_s=30.0, grace_s=5.0):
    return YayaLevhaKapisi(
        Blackboard(), enabled=enabled, levha_max_age_s=0.6, odom_max_age_s=0.5,
        arm_menzil_m=arm_menzil_m, pass_behind_m=pass_behind_m,
        arm_max_s=arm_max_s, grace_s=grace_s,
    )


def set_levha(bb, present=True, d=8.0, taze=True):
    bb.obs.yaya_levha_present = present
    bb.obs.yaya_levha_distance = d if present else -1.0
    bb.obs.yaya_levha_last_seen = _clock.t if taze else 0.0


def set_odom(bb, x=0.0, y=0.0, yaw=0.0, taze=True):
    bb.obs.x, bb.obs.y, bb.obs.yaw = x, y, yaw
    bb.obs.odom_last_seen = _clock.t if taze else 0.0


print("== enabled=False → pass-through (daima SUCCESS) ==")
_clock.t = 1000.0
kapi = make_kapi(enabled=False); bb = kapi.bb
set_levha(bb, present=False)
check("devre dışı → SUCCESS", kapi.update() == Status.SUCCESS)
check("armed yazılmadı", bb.state.yaya_kapi_armed is False)

print("== levha yok → kapı kapalı (FAILURE) ==")
_clock.t = 1100.0
kapi = make_kapi(); bb = kapi.bb
set_levha(bb, present=False); set_odom(bb)
check("levha yok → FAILURE", kapi.update() == Status.FAILURE)
check("armed değil", bb.state.yaya_kapi_armed is False)

print("== levha menzil dışı (>menzil) → açılmaz ==")
_clock.t = 1200.0
kapi = make_kapi(); bb = kapi.bb
set_levha(bb, d=15.0); set_odom(bb)
check("menzil dışı → FAILURE", kapi.update() == Status.FAILURE)

print("== UZAK TABELA: 42m, arm_menzil=45 → açılır (saha fix); dar 10 kapı → açılmaz ==")
_clock.t = 1230.0
kapi = make_kapi(arm_menzil_m=45.0); bb = kapi.bb   # yaya_gecidi.levha_arm_menzil_m
set_levha(bb, d=42.0); set_odom(bb)
check("42m < 45m → arm (geçit duyuruldu)", kapi.update() == Status.SUCCESS and bb.state.yaya_kapi_armed)
kapi2 = make_kapi(arm_menzil_m=10.0); bb2 = kapi2.bb
set_levha(bb2, d=42.0); set_odom(bb2)
check("42m > 10m dar kapı → açılmaz", kapi2.update() == Status.FAILURE and not bb2.state.yaya_kapi_armed)

print("== levha bayat → açılmaz ==")
_clock.t = 1300.0
kapi = make_kapi(); bb = kapi.bb
set_levha(bb, d=8.0, taze=False); set_odom(bb)
check("bayat levha → FAILURE", kapi.update() == Status.FAILURE)

print("== levha menzil içinde → kapı açılır + FSM sıfırlanır ==")
_clock.t = 1400.0
kapi = make_kapi(); bb = kapi.bb
bb.state.yaya_gecidi_phase = "released"   # önceki takılı faz
set_levha(bb, d=8.0); set_odom(bb, x=0.0, yaw=0.0)
st = kapi.update()
check("açıldı → SUCCESS", st == Status.SUCCESS)
check("armed", bb.state.yaya_kapi_armed is True)
check("FSM faz sıfırlandı (idle)", bb.state.yaya_gecidi_phase == "idle")
check("çapa kuruldu (x≈8)", bb.state.yaya_kapi_anchored and abs(bb.state.yaya_kapi_anchor[0] - 8.0) < 1e-6)

print("== armed iken levha kaybolsa bile açık kalır (köprü) ==")
_clock.ilerle(0.2)
set_levha(bb, present=False); set_odom(bb, x=1.0)
check("açık kal → SUCCESS", kapi.update() == Status.SUCCESS)
check("hala armed", bb.state.yaya_kapi_armed is True)

print("== çizgi FSM 'released' → kapan ==")
bb.state.yaya_gecidi_phase = "released"
st = kapi.update()
check("release → FAILURE", st == Status.FAILURE)
check("armed kapandı", bb.state.yaya_kapi_armed is False)

print("== levhayı geç (çapa pass_behind gerisinde) → kapan ==")
_clock.t = 1600.0
kapi = make_kapi(pass_behind_m=3.0); bb = kapi.bb
set_levha(bb, d=5.0); set_odom(bb, x=0.0, yaw=0.0)   # çapa (5,0)
kapi.update()
check("açıldı", bb.state.yaya_kapi_armed is True)
_clock.ilerle(0.2)
set_levha(bb, present=False); set_odom(bb, x=9.0, yaw=0.0)  # 9 > 5+3 → geçildi
st = kapi.update()
check("geçildi → FAILURE", st == Status.FAILURE)
check("armed kapandı", bb.state.yaya_kapi_armed is False)

print("== çapa geçilmeden (ara mesafe) → açık kal ==")
_clock.t = 1700.0
kapi = make_kapi(pass_behind_m=3.0); bb = kapi.bb
set_levha(bb, d=5.0); set_odom(bb, x=0.0)
kapi.update()
_clock.ilerle(0.2)
set_levha(bb, present=False); set_odom(bb, x=6.0)  # 6 < 5+3 → henüz geçilmedi
check("henüz geçilmedi → SUCCESS", kapi.update() == Status.SUCCESS)

print("== odom yoksa geçildi tetiklenmez (yalnız release/TTL kapatır) ==")
_clock.t = 1800.0
kapi = make_kapi(); bb = kapi.bb
set_levha(bb, d=5.0); set_odom(bb, x=0.0)
kapi.update()   # taze odom → çapa (5,0), anchored=True
_clock.ilerle(0.2)
set_levha(bb, present=False); set_odom(bb, x=50.0, taze=False)  # odom BAYAT
check("bayat odom → geçildi YOK → SUCCESS", kapi.update() == Status.SUCCESS)

print("== fail-safe: arm_max_s dolunca kapan ==")
_clock.t = 1900.0
kapi = make_kapi(arm_max_s=10.0); bb = kapi.bb
set_levha(bb, d=5.0); set_odom(bb, x=0.0)
kapi.update()
_clock.ilerle(11.0)   # > arm_max_s
set_levha(bb, present=False); set_odom(bb, x=0.5)  # çok az ilerledi → geçildi değil
st = kapi.update()
check("TTL → FAILURE", st == Status.FAILURE)
check("armed kapandı", bb.state.yaya_kapi_armed is False)

print("== kapandıktan sonra grace içinde yeniden silahlanmaz ==")
_clock.t = 2000.0
kapi = make_kapi(grace_s=5.0); bb = kapi.bb
set_levha(bb, d=5.0); set_odom(bb, x=0.0)
kapi.update()
bb.state.yaya_gecidi_phase = "released"
kapi.update()   # kapan (released_s = 2000)
check("kapandı", bb.state.yaya_kapi_armed is False)
_clock.ilerle(2.0)   # grace(5) içinde
set_levha(bb, d=5.0)   # levha yine menzilde
check("grace içinde → yeniden açılmaz", kapi.update() == Status.FAILURE)
check("hala armed değil", bb.state.yaya_kapi_armed is False)

print("== grace sonrası yeni levha → yeniden silahlan ==")
_clock.ilerle(4.0)   # toplam 6s > grace(5)
set_levha(bb, d=5.0); set_odom(bb, x=0.0)
check("grace sonrası → yeniden açılır", kapi.update() == Status.SUCCESS)
check("armed", bb.state.yaya_kapi_armed is True)


if failures:
    print(f"\n✗ {len(failures)} test BAŞARISIZ: {failures}")
    sys.exit(1)
print("\nOK: tüm yaya geçidi levha-kapısı testleri geçti")
