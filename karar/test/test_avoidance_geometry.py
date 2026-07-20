#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""avoidance_geometry.py birim testleri — ROS yok.

Çalıştır:
    python3 karar/test/test_avoidance_geometry.py
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from avoidance_geometry import (  # noqa: E402
    obstacle_world_pos, project_to_segment, side_to_avoid, longitudinal_gap,
    ackermann_radius, lane_change_longitudinal, ackermann_return_distance,
    required_steer_deg, avoidance_feasible,
)

_fail = []


def approx(name, got, exp, tol=1e-2):
    if abs(got - exp) > tol:
        _fail.append(f"{name}: beklenen≈{exp}, alınan={got}")
        print(f"  ✗ {name}: beklenen≈{exp}, alınan={got}")
    else:
        print(f"  ✓ {name}: {got:.3f}")


def eq(name, got, exp):
    if got != exp:
        _fail.append(f"{name}: beklenen={exp}, alınan={got}")
        print(f"  ✗ {name}: beklenen={exp}, alınan={got}")
    else:
        print(f"  ✓ {name}: {got}")


print("== obstacle_world_pos ==")
# Robot orijinde, yaw=0; engel tam önde (açı 0, d=5) → (5,0)
ox, oy = obstacle_world_pos(0, 0, 0.0, 5.0, 0.0)
approx("önde-x", ox, 5.0); approx("önde-y", oy, 0.0)
# Engel sağda (açı +30 sağ-pozitif, d=10): fwd=8.66, lat=-5 → (8.66,-5)
ox, oy = obstacle_world_pos(0, 0, 0.0, 10.0, 30.0)
approx("sağ-x", ox, 8.66); approx("sağ-y", oy, -5.0)
# yaw=90° (+y'ye bakıyor), engel tam önde d=5 → (0,5)
ox, oy = obstacle_world_pos(0, 0, math.pi / 2, 5.0, 0.0)
approx("yaw90-x", ox, 0.0); approx("yaw90-y", oy, 5.0)

print("\n== project_to_segment ==")
t, s, lat = project_to_segment(5, 2, 0, 0, 10, 0)
approx("t", t, 0.5); approx("s_long", s, 5.0); approx("lateral(sol+)", lat, 2.0)
t, s, lat = project_to_segment(5, -3, 0, 0, 10, 0)
approx("lateral(sağ-)", lat, -3.0)

print("\n== side_to_avoid ==")
# Engel rotanın SOLUNDA (lat>0) → SAĞA kaç
side, lat = side_to_avoid(5, 2, 0, 0, 10, 0, deadband_m=0.4)
eq("sol-engel→sag", side, "sag")
# Engel rotanın SAĞINDA (lat<0) → SOLA kaç
side, lat = side_to_avoid(5, -2, 0, 0, 10, 0, deadband_m=0.4)
eq("sag-engel→sol", side, "sol")
# Engel ~rota üzerinde (deadband içinde) → None
side, lat = side_to_avoid(5, 0.1, 0, 0, 10, 0, deadband_m=0.4)
eq("merkez→None", side, None)
# Dejenere segment (a==b, rota sonu wp1==wp2 padding) → lateral 0 → None (fallback)
_, _, latd = project_to_segment(5, 2, 3, 3, 3, 3)
approx("dejenere lateral=0", latd, 0.0)
side, _ = side_to_avoid(5, 2, 3, 3, 3, 3, deadband_m=0.4)
eq("dejenere→None (yan_sektor fallback)", side, None)

print("\n== longitudinal_gap ==")
approx("robot 5m önde", longitudinal_gap(10, 0, 5, 0, 1, 0), 5.0)
approx("robot 2m arkada", longitudinal_gap(3, 0, 5, 0, 1, 0), -2.0)
approx("dik yön gap=0", longitudinal_gap(5, 9, 5, 0, 1, 0), 0.0)

print("\n== Ackermann ==")
R30 = ackermann_radius(30.0, 1.78)
approx("R@30°", R30, 3.083)
approx("düz→inf", 1.0 if math.isinf(ackermann_radius(0.0, 1.78)) else 0.0, 1.0)
arc = lane_change_longitudinal(R30, 1.8)
approx("yay uzunluğu@R30,Δ1.8", arc, 2.80)
rd = ackermann_return_distance(1.8, 20.0, 1.78, 2.0)
approx("return_dist(Δ1.8,20°,clr2)", rd, 5.79)
# required_steer ↔ lane_change_longitudinal tutarlılığı: 30°'nin ürettiği yayı
# geri verince ~30° çıkmalı
approx("required_steer geri", required_steer_deg(1.8, arc, 1.78), 30.0, tol=0.5)
eq("fizibilite: kısa mesafe yetmez", avoidance_feasible(1.8, 1.0, 1.78, 30.0), False)
eq("fizibilite: uzun mesafe olur", avoidance_feasible(1.8, 6.0, 1.78, 30.0), True)

print("\n" + "=" * 50)
if _fail:
    print(f"FAIL: {len(_fail)} test başarısız")
    sys.exit(1)
print("OK: tüm geometri testleri geçti")
sys.exit(0)
