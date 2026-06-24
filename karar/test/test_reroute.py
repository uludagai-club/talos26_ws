#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reroute.py (RerouteManager) birim testleri — ROS yok.

    python3 karar/test/test_reroute.py

Cone reroute durum makinesi (§16/E-A,E-B): bloklu cone'u /hedef_komut kenar_blok
ile bildir, temizlenince kenar_serbest, zaman aşımı fallback, acildurus'ta koru.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from reroute import RerouteManager, RerouteParams  # noqa: E402

failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


def base_params(enabled=True):
    return RerouteParams(enabled=enabled, block_radius_m=1.0, max_s=15.0,
                         refresh_s=1.0, release_clear_ticks=3)


CONE = (3.0, 1.0)
BLOK = "kenar_blok;-;3.00;1.00;cone;1.00"
SERBEST = "kenar_serbest;-;3.00;1.00;cone;1.00"


print("== enabled=False → komut yok ==")
m = RerouteManager(base_params(enabled=False))
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=0.0)
check("enabled=False inaktif", (not r.active) and r.command is None, f"-> {r.command}")

print("\n== aktivasyon: bloklu cone → kenar_blok ==")
m = RerouteManager(base_params())
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=0.0)
check("kenar_blok komutu", r.command == BLOK, f"-> {r.command}")
check("aktif", r.active)
check("event blok", r.event is not None and r.event[0] == "blok")

print("\n== refresh: pencere içinde komut yok, sonra tazelenir ==")
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=0.5)
check("refresh penceresi içinde komut yok", r.command is None and r.active, f"-> {r.command}")
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=1.0)
check("refresh sonrası kenar_blok tazelenir", r.command == BLOK, f"-> {r.command}")

print("\n== cone temizlendi: debounce sonrası kenar_serbest ==")
r = m.update(reroute_request=False, cone_world=CONE, decision_karar="normal", now=1.1)
check("1. temiz tick: komut yok, hâlâ aktif", r.command is None and r.active)
r = m.update(reroute_request=False, cone_world=CONE, decision_karar="normal", now=1.2)
check("2. temiz tick: komut yok", r.command is None and r.active)
r = m.update(reroute_request=False, cone_world=CONE, decision_karar="normal", now=1.3)
check("3. temiz tick (>=release): kenar_serbest", r.command == SERBEST, f"-> {r.command}")
check("serbest sonrası pasif", not r.active)
check("event serbest", r.event is not None and r.event[0] == "serbest")

print("\n== zaman aşımı → fallback kenar_serbest ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=0.0)
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=16.0)
check("zaman aşımı kenar_serbest", r.command == SERBEST, f"-> {r.command}")
check("zaman aşımı pasif", not r.active)
check("event zaman_asimi", r.event is not None and r.event[0] == "zaman_asimi")

print("\n== acildurus: blok KORUNUR (cone hâlâ orada; e-stop control'de) ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=CONE, decision_karar="slow", now=0.0)
r = m.update(reroute_request=True, cone_world=CONE, decision_karar="acildurus", now=0.5)
check("acildurus'ta aktif kalır", r.active)
check("acildurus'ta komut yok (serbest bırakmaz)", r.command is None, f"-> {r.command}")

print("\n== geçersiz cone (0,0) → aktive olmaz ==")
m = RerouteManager(base_params())
r = m.update(reroute_request=True, cone_world=(0.0, 0.0), decision_karar="slow", now=0.0)
check("cone (0,0) → inaktif, komut yok", (not r.active) and r.command is None, f"-> {r.command}")

print("\n" + "=" * 50)
if failures:
    print(f"FAIL: {len(failures)} test başarısız")
    for f in failures:
        print(" -", f)
    sys.exit(1)
print("OK: tüm reroute testleri geçti")
sys.exit(0)
