#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YayaGecidiFSM birim testleri — ROS yok.

    python3 karar/test/test_yaya_gecidi.py

Adanmış crosswalk modeli yalnız GEÇİT ÇİZGİSİNİ veriyor (yaya değil). FSM:
  • geçit uzak → karar yok (FAILURE)
  • geçit yavaş bandı → slow (yaklasma)
  • geçit yakın → MİNİMAL zorunlu duruş (min_bekleme_s), yaya olsun olmasın
  • min doldu + lidar engel (yaya) var → max_bekleme_s'e dek bekle
  • temiz / max doldu → RELEASED (FAILURE → araç devam)
  • max_bekleme_s statik engelde kalıcı kilidi keser
  • geçit görüşten çıkınca (yaya_present=False) yeniden silahlan
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import behaviors.actions as actions_mod  # noqa: E402
from behaviors.actions import YayaGecidiFSM  # noqa: E402
from bb import Blackboard  # noqa: E402
from py_trees.common import Status  # noqa: E402

failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


# --- Sahte saat: time.time()'ı kontrol et (min/max bekleme testleri için) ---
class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def ilerle(self, dt):
        self.t += dt


_clock = FakeClock()
actions_mod.time = _clock  # FSM `time.time()` → sahte saat


def make_fsm():
    return YayaGecidiFSM(
        Blackboard(), dur_esik_m=4.0, yavas_esik_m=12.0,
        min_bekleme_s=3.0, max_bekleme_s=20.0, engel_bekle_m=8.0,
        release_grace_s=8.0,
    )


def set_gecit(bb, present=True, d=3.0):
    bb.obs.yaya_present = present
    bb.obs.yaya_distance = d


def set_engel(bb, present=False, d=99.0):
    bb.obs.engel_present = present
    bb.obs.engel_d_center = d


print("== geçit yok → FAILURE, idle ==")
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, present=False)
check("gecit yok FAILURE", fsm.update() == Status.FAILURE)
check("faz idle", bb.state.yaya_gecidi_phase == "idle")

print("== geçit uzak (>yavas) → FAILURE ==")
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=15.0); set_engel(bb, present=False)
check("uzak FAILURE", fsm.update() == Status.FAILURE)

print("== geçit yavaş bandı (10m) → slow ==")
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=10.0); set_engel(bb, present=False)
st = fsm.update()
check("slow SUCCESS", st == Status.SUCCESS)
check("karar slow", bb.last_decision["karar"] == "slow", bb.last_decision)
check("reason yaklasma", bb.last_decision["reason"] == "yaya_gecidi_yaklasma")

print("== geçit yakın (3m) → min zorunlu duruş ==")
_clock.t = 1000.0
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=3.0); set_engel(bb, present=False)
st = fsm.update()
check("dur SUCCESS", st == Status.SUCCESS)
check("faz holding", bb.state.yaya_gecidi_phase == "holding")
check("karar dur", bb.last_decision["karar"] == "dur", bb.last_decision)
check("reason min_dur", bb.last_decision["reason"] == "yaya_gecidi_min_dur")

print("== min bekleme dolmadan engel yokken bile DUR ==")
_clock.ilerle(1.5)  # 1.5s < 3.0
set_engel(bb, present=False)
st = fsm.update()
check("hala dur (min<3)", st == Status.SUCCESS and bb.last_decision["karar"] == "dur")

print("== min doldu + engel YOK → RELEASED (devam) ==")
_clock.ilerle(2.0)  # toplam 3.5s > 3.0
set_engel(bb, present=False)
st = fsm.update()
check("release FAILURE", st == Status.FAILURE)
check("faz released", bb.state.yaya_gecidi_phase == "released")

print("== min doldu + lidar engel (yaya) VAR → beklemeye devam ==")
_clock.t = 2000.0
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=3.0); set_engel(bb, present=False)
fsm.update()                       # holding başlat (t=2000)
_clock.ilerle(3.5)                 # min doldu
set_engel(bb, present=True, d=5.0)  # geçitte yaya (8m içinde)
st = fsm.update()
check("engel var → dur", st == Status.SUCCESS and bb.last_decision["karar"] == "dur", bb.last_decision)
check("reason yaya_bekle", bb.last_decision["reason"] == "yaya_gecidi_yaya_bekle")

print("== engel uzak (>engel_bekle_m) → yaya yok say → devam ==")
set_engel(bb, present=True, d=12.0)  # 12 > 8 → geçitte değil
st = fsm.update()
check("uzak engel → release", st == Status.FAILURE and bb.state.yaya_gecidi_phase == "released")

print("== engel yaya geçince → devam ==")
_clock.t = 3000.0
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=3.0); set_engel(bb, present=False)
fsm.update()
_clock.ilerle(4.0)
set_engel(bb, present=True, d=4.0)
check("yaya var dur", fsm.update() == Status.SUCCESS)
set_engel(bb, present=False)  # yaya geçti
check("yaya gitti → release", fsm.update() == Status.FAILURE)

print("== KONİ çakışması: reroute aktifken engeli yaya sayma → min sonrası devam ==")
_clock.t = 3500.0
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=3.0)
set_engel(bb, present=True, d=3.0)     # yakın engel ama...
bb.state.overtake_active = True         # ...bu bir izlenen KONİ (reroute aktif)
fsm.update()                            # holding
_clock.ilerle(3.5)                      # min doldu
st = fsm.update()
check("koni → yaya bekleme YOK → release", st == Status.FAILURE and bb.state.yaya_gecidi_phase == "released")

print("== STATİK engel: max_bekleme_s kalıcı kilidi keser ==")
_clock.t = 4000.0
fsm = make_fsm(); bb = fsm.bb
set_gecit(bb, d=3.0); set_engel(bb, present=True, d=3.0)  # hep engel (statik koni/yanlış)
fsm.update()                        # holding
_clock.ilerle(2.0); fsm.update()    # min içi → dur
_clock.ilerle(10.0)                 # 12s: min doldu, engel var → bekle
check("12s hala dur", fsm.update() == Status.SUCCESS)
_clock.ilerle(10.0)                 # 22s > max(20)
check("max aşıldı → release (kilit kırıldı)", fsm.update() == Status.FAILURE)
check("faz released", bb.state.yaya_gecidi_phase == "released")

print("== RELEASED sonrası release_grace: aynı geçit yeniden tetiklemez ==")
# aynı fsm, released_s=4022 civarı; geçit hâlâ görünür (araç üstünde)
set_gecit(bb, d=2.0); set_engel(bb, present=False)
bb.state.yaya_gecidi_phase = "idle"  # present kapısı idle'a çeker (araç hâlâ geçit görüyor)
st = fsm.update()
check("grace içinde yeniden tetiklemez", st == Status.FAILURE and bb.state.yaya_gecidi_phase == "idle")

print("== geçit görüşten çıkınca yeniden silahlan (grace sonrası yeni geçit) ==")
_clock.ilerle(9.0)  # grace(8) doldu
set_gecit(bb, present=False)  # geçit gitti
fsm.update()
check("present yok → idle", bb.state.yaya_gecidi_phase == "idle")
set_gecit(bb, d=3.0); set_engel(bb, present=False)  # YENİ geçit
st = fsm.update()
check("yeni geçit → yeniden holding", bb.state.yaya_gecidi_phase == "holding" and st == Status.SUCCESS)


if failures:
    print(f"\n✗ {len(failures)} test BAŞARISIZ: {failures}")
    sys.exit(1)
print("\nOK: tüm yaya geçidi FSM testleri geçti")
