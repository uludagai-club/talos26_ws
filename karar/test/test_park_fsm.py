#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ParkFSM birim testleri — ROS yok.

    python3 -m test.test_park_fsm

Park müsaitlik: "park tabelası → model → lidar" üç-kapılı AND.
  Kapı 1  PARK_YERI levhası (kapı arm; PARK_ETMEK_YASAKTIR → yasak)
  Kapı 2  /park_alani modeli present + taze
  Kapı 3  lidar engel yok  → 2026-07-24 ERTELENDİ (lidar_enabled=false → True)
Üçü de olumlu → reason=park_musait; biri yoksa → park_musait_degil / park_yasak.

Kapı yaşam döngüsü (YayaLevhaKapisi aynası):
  • park tabelası yok → FAILURE (cruise)
  • PARK_YERI menzilde → armed; model yoksa park_musait_degil, varsa park_musait
  • levha 1-tick düşse de armed sürer (sustain); model taze olduğu sürece müsait
  • levhayı geç (dünya-çapası pass_behind) → kapan (FAILURE) → idle
  • fail-safe arm_max_s → kapan
  • kapandıktan sonra grace içinde yeniden silahlanmaz
  • enabled=False → dal kapalı (FAILURE)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import behaviors.actions as actions_mod  # noqa: E402
from behaviors.actions import ParkFSM  # noqa: E402
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
actions_mod.time = _clock  # ParkFSM `time.time()` → sahte saat


def make_fsm(enabled=True, arm_menzil_m=45.0, pass_behind_m=3.0,
             arm_max_s=60.0, grace_s=5.0, lidar_enabled=False):
    return ParkFSM(
        Blackboard(), enabled=enabled, arm_menzil_m=arm_menzil_m,
        levha_max_age_s=0.6, park_max_age_s=0.6, odom_max_age_s=0.5,
        pass_behind_m=pass_behind_m, arm_max_s=arm_max_s, grace_s=grace_s,
        lidar_enabled=lidar_enabled,
    )


def set_levha(bb, isim="PARK_YERI", d=20.0, taze=True):
    bb.obs.levha_isim = isim
    bb.obs.levha_distance = d
    bb.obs.levha_last_seen = _clock.t if taze else 0.0


def clear_levha(bb):
    bb.obs.levha_isim = "NONE"
    bb.obs.levha_distance = -1.0
    bb.obs.levha_last_seen = 0.0


def set_model(bb, present=True, d=6.0, off=1.5, taze=True):
    bb.obs.park_alani_present = present
    bb.obs.park_alani_distance = d if present else -1.0
    bb.obs.park_alani_offset = off
    bb.obs.park_alani_last_seen = _clock.t if taze else 0.0


def set_odom(bb, x=0.0, y=0.0, yaw=0.0, taze=True):
    bb.obs.x = x
    bb.obs.y = y
    bb.obs.yaw = yaw
    bb.obs.odom_last_seen = _clock.t if taze else 0.0


def reason(bb):
    return bb.last_decision.get("reason")


# ============================================================
print("== park tabelası yok → cruise (FAILURE) ==")
fsm = make_fsm()
bb = fsm.bb
clear_levha(bb)
set_model(bb, present=True)   # model var ama levha yok → kapı açılmaz
set_odom(bb)
check("model var levha yok → FAILURE", fsm.update() == Status.FAILURE)
check("faz idle", bb.state.park_phase == "idle")

print("== PARK_YERI armed + model YOK → park_musait_degil ==")
fsm = make_fsm()
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=False)
set_odom(bb)
st = fsm.update()
check("SUCCESS", st == Status.SUCCESS)
check("armed", bb.state.park_phase == "armed")
check("reason=park_musait_degil", reason(bb) == "park_musait_degil", reason(bb))
check("motion slow", bb.last_decision.get("karar") == "slow")

print("== PARK_YERI + model VAR → park_musait ==")
fsm = make_fsm()
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=True, d=6.0)
set_odom(bb)
st = fsm.update()
check("SUCCESS", st == Status.SUCCESS)
check("reason=park_musait", reason(bb) == "park_musait", reason(bb))
check("phase=park_musait", bb.last_decision.get("phase") == "park_musait")

print("== PARK_ETMEK_YASAKTIR (idle) → park_yasak ==")
fsm = make_fsm()
bb = fsm.bb
set_levha(bb, "PARK_ETMEK_YASAKTIR", d=20.0)
set_model(bb, present=True)
set_odom(bb)
st = fsm.update()
check("SUCCESS", st == Status.SUCCESS)
check("reason=park_yasak", reason(bb) == "park_yasak", reason(bb))
check("faz idle kalır (yasak arm etmez)", bb.state.park_phase == "idle")

print("== SUSTAIN: armed sonrası levha düşse de model varken müsait ==")
fsm = make_fsm()
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=True)
set_odom(bb)
fsm.update()
check("önce armed", bb.state.park_phase == "armed")
clear_levha(bb)                 # levha 1-tick düştü
set_model(bb, present=True)     # model taze
_clock.ilerle(0.1)
set_model(bb, present=True)     # tazeliği güncelle
st = fsm.update()
check("levha düştü ama armed sürer", bb.state.park_phase == "armed")
check("model varken hâlâ müsait", reason(bb) == "park_musait", reason(bb))

print("== model taze değil → park_musait_degil ==")
fsm = make_fsm()
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=True, taze=True)
set_odom(bb)
fsm.update()
_clock.ilerle(1.0)              # model tazeliği eskidi (>0.6s)
set_levha(bb, "PARK_YERI", d=18.0)   # levha taze kalsın (arm sürsün)
set_odom(bb)
st = fsm.update()
check("model bayat → park_musait_degil", reason(bb) == "park_musait_degil", reason(bb))

print("== levhayı geç (pass_behind) → kapan → idle ==")
fsm = make_fsm(pass_behind_m=3.0)
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=5.0)
set_odom(bb, x=0.0, yaw=0.0)    # çapa (5,0)
set_model(bb, present=True)
fsm.update()
check("armed + anchored", bb.state.park_phase == "armed" and bb.state.park_kapi_anchored)
clear_levha(bb)
set_odom(bb, x=9.0, yaw=0.0)    # çapa 9m geride (< -3) → geçildi
st = fsm.update()
check("geçildi → FAILURE", st == Status.FAILURE)
check("faz released", bb.state.park_phase == "released")
st2 = fsm.update()              # levha yok → released→idle
check("released→idle", bb.state.park_phase == "idle")

print("== fail-safe arm_max_s → kapan ==")
fsm = make_fsm(arm_max_s=10.0)
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_odom(bb, taze=False)        # odom yok → geçildi kapısı devre dışı, yalnız TTL
set_model(bb, present=False)
fsm.update()
check("armed", bb.state.park_phase == "armed")
_clock.ilerle(11.0)
set_levha(bb, "PARK_YERI", d=20.0)  # hâlâ görülüyor ama TTL doldu
st = fsm.update()
check("TTL → FAILURE", st == Status.FAILURE)
check("released", bb.state.park_phase == "released")

print("== grace: kapandıktan sonra hemen yeniden arm etmez ==")
# yukarıdaki fsm released fazında; grace_s=5 içinde PARK_YERI → arm etmemeli
set_levha(bb, "PARK_YERI", d=20.0)
_clock.ilerle(1.0)              # grace (5s) içinde
st = fsm.update()
check("grace içinde arm yok → FAILURE (released→idle geçişi)",
      bb.state.park_phase in ("idle", "released"))
_clock.ilerle(6.0)             # grace bitti
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=False)
st = fsm.update()
check("grace sonrası yeniden arm", bb.state.park_phase == "armed")

print("== enabled=False → dal kapalı ==")
fsm = make_fsm(enabled=False)
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=20.0)
set_model(bb, present=True)
set_odom(bb)
check("disabled → FAILURE", fsm.update() == Status.FAILURE)

print("== menzil dışı PARK_YERI → arm yok ==")
fsm = make_fsm(arm_menzil_m=45.0)
bb = fsm.bb
set_levha(bb, "PARK_YERI", d=60.0)   # menzil dışı
set_model(bb, present=True)
set_odom(bb)
check("menzil dışı → FAILURE", fsm.update() == Status.FAILURE)
check("faz idle", bb.state.park_phase == "idle")


if failures:
    print(f"\n✗ {len(failures)} test BAŞARISIZ: {failures}")
    sys.exit(1)
print("\nOK: tüm park FSM testleri geçti")
