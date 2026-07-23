#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TrafikIsigiFSM birim testleri — ROS yok.

    python3 karar/test/test_trafik_isigi.py

Birleşik ışık FSM'i (KIRMIZI/SARI/YEŞİL), DUR levhasından AYRI:
  • KIRMIZI → 'dur' (yeşile kadar; zaman-sınırı yok)
  • SARI    → 'slow'  (kırmızıdan SONRA = yeşile hazırlan, HER ZAMAN slow;
                        yaklaşırken = yellow_action politikası)
  • YEŞİL / ışık yok → FAILURE (geç)
  • son ışık release_grace_s tutulur → kısa flicker duruşu/yavaşı bozmaz
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import behaviors.actions as actions_mod  # noqa: E402
from behaviors.actions import TrafikIsigiFSM  # noqa: E402
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
actions_mod.time = _clock


def make_fsm(oku_esik_m=10.0, release_grace_s=2.0, yellow_action="slow"):
    return TrafikIsigiFSM(Blackboard(), oku_esik_m=oku_esik_m,
                          release_grace_s=release_grace_s, yellow_action=yellow_action)


def set_levha(bb, isim, d=6.0):
    bb.obs.levha_isim = isim
    bb.obs.levha_distance = d


def karar(bb):
    return bb.last_decision["karar"], bb.last_decision["reason"]


print("== ışık yok → FAILURE ==")
_clock.t = 1000.0
fsm = make_fsm(); bb = fsm.bb
set_levha(bb, "NONE", -1.0)
check("ışık yok → FAILURE", fsm.update() == Status.FAILURE)
check("last_light boş", bb.state.trafik_isik_last_light == "")

print("== KIRMIZI menzil içinde → dur ==")
_clock.t = 1100.0
fsm = make_fsm(); bb = fsm.bb
set_levha(bb, "KIRMIZI", 6.0)
st = fsm.update()
check("dur SUCCESS", st == Status.SUCCESS and karar(bb) == ("dur", "trafik_kirmizi"), bb.last_decision)
check("last_light KIRMIZI", bb.state.trafik_isik_last_light == "KIRMIZI")

print("== KIRMIZI menzil dışı → tetiklenmez ==")
_clock.t = 1200.0
fsm = make_fsm(oku_esik_m=10.0); bb = fsm.bb
set_levha(bb, "KIRMIZI", 15.0)
check("menzil dışı → FAILURE", fsm.update() == Status.FAILURE)

print("== KIRMIZI → YEŞİL → anında geç ==")
_clock.t = 1300.0
fsm = make_fsm(); bb = fsm.bb
set_levha(bb, "KIRMIZI", 6.0); fsm.update()
set_levha(bb, "YESIL", 6.0)
check("yeşil → FAILURE", fsm.update() == Status.FAILURE)
check("last_light temizlendi", bb.state.trafik_isik_last_light == "")

print("== KIRMIZI → 1-tick flicker (<grace) → hâlâ dur ==")
_clock.t = 1400.0
fsm = make_fsm(release_grace_s=2.0); bb = fsm.bb
set_levha(bb, "KIRMIZI", 6.0); fsm.update()
_clock.ilerle(0.5); set_levha(bb, "NONE", -1.0)
st = fsm.update()
check("flicker → hâlâ dur", st == Status.SUCCESS and karar(bb)[0] == "dur")

print("== KIRMIZI → grace boyunca yok → geç ==")
_clock.ilerle(2.5); set_levha(bb, "NONE", -1.0)
check("grace doldu → FAILURE", fsm.update() == Status.FAILURE)
check("temizlendi", bb.state.trafik_isik_last_light == "")

print("== YAKLAŞMA sarısı (kırmızısız) → yellow_action (default slow) ==")
_clock.t = 1500.0
fsm = make_fsm(yellow_action="slow"); bb = fsm.bb
set_levha(bb, "YAVAS", 6.0)
st = fsm.update()
check("yaklaşma sarı → slow", st == Status.SUCCESS and karar(bb) == ("slow", "trafik_sari"), bb.last_decision)
check("hazir DEĞİL", bb.state.trafik_isik_hazir is False)

print("== YAKLAŞMA sarısı + yellow_action=dur → dur ==")
_clock.t = 1550.0
fsm = make_fsm(yellow_action="dur"); bb = fsm.bb
set_levha(bb, "YAVAS", 6.0)
st = fsm.update()
check("yaklaşma sarı → dur (politika)", st == Status.SUCCESS and karar(bb) == ("dur", "trafik_sari"), bb.last_decision)

print("== KIRMIZI → SARI → yeşile HAZIRLAN (slow), yellow_action=dur olsa bile ==")
_clock.t = 1600.0
fsm = make_fsm(yellow_action="dur"); bb = fsm.bb
set_levha(bb, "KIRMIZI", 6.0); fsm.update()          # dur
set_levha(bb, "YAVAS", 6.0)
st = fsm.update()
check("kırmızı→sarı → slow (hazir)", st == Status.SUCCESS and karar(bb) == ("slow", "trafik_sari_hazir"), bb.last_decision)
check("hazir True", bb.state.trafik_isik_hazir is True)

print("== KIRMIZI → SARI(hazir) → 1-tick flicker → hâlâ slow(hazir) ==")
_clock.ilerle(0.5); set_levha(bb, "NONE", -1.0)
st = fsm.update()
check("flicker → hâlâ hazir slow", st == Status.SUCCESS and karar(bb) == ("slow", "trafik_sari_hazir"))

print("== KIRMIZI → SARI → YEŞİL → geç ==")
_clock.ilerle(0.2); set_levha(bb, "YESIL", 6.0)
check("yeşil → FAILURE (geç)", fsm.update() == Status.FAILURE)
check("hazir reset", bb.state.trafik_isik_hazir is False)

print("== YEŞİL doğrudan → FAILURE ==")
_clock.t = 1700.0
fsm = make_fsm(); bb = fsm.bb
set_levha(bb, "YESIL", 6.0)
check("yeşil → FAILURE", fsm.update() == Status.FAILURE)


if failures:
    print(f"\n✗ {len(failures)} test BAŞARISIZ: {failures}")
    sys.exit(1)
print("\nOK: tüm trafik ışığı FSM testleri geçti")
