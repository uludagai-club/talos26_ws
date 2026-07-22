#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EngelMerkezBlokaj yay-kapısı-farkındalığı birim testleri — ROS yok.

    python3 karar/test/test_engel_merkez_blokaj.py

2026-07-23: yavasla/reroute bandı eskiden yalnız engel_d_center (düz-ileri)
okuyordu → araç dönerken (koni yayda ama düz-eksende değil) d_center=inf olup
bant tetiklenmiyor, araç koniye dalıp control hard-floor deadlock'una giriyordu.
Fix: min(d_center, d_arc). d_arc direksiyon bayat/yay kapalıyken d_center'a düşer
→ düz sürüşte davranış aynı.
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behaviors.conditions import EngelMerkezBlokaj  # noqa: E402
from bb import Blackboard  # noqa: E402
from py_trees.common import Status  # noqa: E402

INF = float("inf")
failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


def sonuc(esik, dc, da):
    bb = Blackboard()
    bb.obs.engel_d_center = dc
    bb.obs.engel_d_arc = da
    return EngelMerkezBlokaj(bb, esik).update()


ESIK = 6.0

print("== DÜZ sürüş: d_arc = d_center (fail-safe) → davranış aynı ==")
check("d_center<esik, d_arc=d_center → SUCCESS", sonuc(ESIK, 4.0, 4.0) == Status.SUCCESS)
check("d_center>esik, d_arc=d_center → FAILURE", sonuc(ESIK, 8.0, 8.0) == Status.FAILURE)

print("== DÖNÜŞ (asıl fix): koni yayda, düz-eksende değil ==")
check("d_center=inf, d_arc<esik → SUCCESS (yay yakalar)", sonuc(ESIK, INF, 3.5) == Status.SUCCESS)
check("d_center=inf, d_arc>esik → FAILURE", sonuc(ESIK, INF, 9.0) == Status.FAILURE)

print("== d_arc daha yakın → min alınır ==")
check("d_center=8 (>esik), d_arc=3 (<esik) → SUCCESS", sonuc(ESIK, 8.0, 3.0) == Status.SUCCESS)
check("d_center=3 (<esik), d_arc=inf → SUCCESS", sonuc(ESIK, 3.0, INF) == Status.SUCCESS)

print("== ikisi de temiz → FAILURE ==")
check("d_center=inf, d_arc=inf → FAILURE", sonuc(ESIK, INF, INF) == Status.FAILURE)
check("d_center=None, d_arc=None → FAILURE", sonuc(ESIK, None, None) == Status.FAILURE)

print("== sınır: min == esik → FAILURE (kesin küçük olmalı) ==")
check("min=esik → FAILURE", sonuc(ESIK, 6.0, 6.0) == Status.FAILURE)


if failures:
    print(f"\n✗ {len(failures)} test BAŞARISIZ: {failures}")
    sys.exit(1)
print("\nOK: tüm EngelMerkezBlokaj yay-farkında testleri geçti")
