#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_memory.py (ObstacleMemory) birim testleri — ROS yok.

    python3 karar/test/test_obstacle_memory.py

Duba DÜNYA-konum hafızası (dropout köprüsü, canlı teşhis 2026-06-26):
  - çerçeve dönüşümü gidiş-dönüş
  - konfirme = yeterli hit + yaklaşma (ikisi de şart)
  - dropout'ta konfirme duba gövde-frame'e geri-projekte edilip enjekte edilir
  - geçince (gövde ileri < -pass_behind) iz düşer (doğal serbest bırakma)
  - memory_ttl: algı olmadan uzun süre → düşer; FP (tek görüş) konfirme olmaz
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from obstacle_memory import (  # noqa: E402
    ObstacleMemory, MemParams, body_to_world, world_to_body,
)

failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def base_params(**kw):
    d = dict(enabled=True, assoc_radius_m=1.5, confirm_hits=3, confirm_approach_m=9.0,
             memory_ttl_s=3.0, unconfirmed_ttl_s=0.8, max_memory_s=12.0,
             inject_forward_min_m=0.3, hold_forward_max_m=14.0, pass_behind_m=1.5,
             pos_alpha=0.4, max_tracks=12, max_pose_jump_m=2.0)
    d.update(kw)
    return MemParams(**d)


print("== çerçeve dönüşümü gidiş-dönüş (yaw=0.7) ==")
wx, wy = body_to_world(6.0, -1.2, 3.0, -2.0, 0.7)
fb, lb = world_to_body(wx, wy, 3.0, -2.0, 0.7)
check("body→world→body identity", approx(fb, 6.0) and approx(lb, -1.2), f"-> {fb:.3f},{lb:.3f}")


print("\n== konfirmasyon + dropout enjeksiyonu (yaw=0, +x boyunca yaklaşma) ==")
m = ObstacleMemory(base_params())
# Duba dünyada (10,0). Araç yaw=0, x=0→3 boyunca ilerleyip her tick algılıyor.
inj = 0
for i, x in enumerate([0.0, 1.0, 2.0, 3.0]):
    rng = 10.0 - x
    pts, st = m.update([(rng, 0.0)], rx=x, ry=0.0, yaw=0.0, now=0.1 * i)
    inj = st["injected"]
check("yaklaşırken konfirme oldu (canlı algı varken enjekte YOK)",
      st["confirmed"] == 1 and inj == 0, f"-> conf={st['confirmed']} inj={inj}")

# x=4'te DROPOUT (boş PoseArray) → konfirme duba enjekte edilmeli (~fwd=6)
pts, st = m.update([], rx=4.0, ry=0.0, yaw=0.0, now=0.5)
mem_pts = [p for p in pts]
check("dropout'ta 1 duba enjekte edildi", st["injected"] == 1, f"-> {st}")
check("enjekte edilen duba ~6m önde, ~0 yanal",
      len(mem_pts) == 1 and approx(mem_pts[0][0], 6.0, 0.2) and approx(mem_pts[0][1], 0.0, 0.2),
      f"-> {mem_pts}")


print("\n== geçince iz düşer (doğal serbest bırakma) ==")
m = ObstacleMemory(base_params())
t = 0.0
x = 0.0
while x <= 9.0:                                  # 1m adımlarla yaklaş + konfirme (jump-reset altında)
    m.update([(10.0 - x, 0.0)], rx=x, ry=0.0, yaw=0.0, now=t); x += 1.0; t += 0.1
# Araç dubayı (dünya x=10) küçük adımlarla geçiyor; algı yok (geride kaldı)
st = None
for x in [10.0, 11.0, 12.0]:
    pts, st = m.update([], rx=x, ry=0.0, yaw=0.0, now=t); t += 0.1
check("geçince iz düştü", st["tracks"] == 0 and st["injected"] == 0, f"-> {st}")


print("\n== memory_ttl: algı olmadan uzun süre → iz düşer ==")
m = ObstacleMemory(base_params())
for i, x in enumerate([0.0, 1.0, 2.0, 3.0]):
    m.update([(10.0 - x, 0.0)], rx=x, ry=0.0, yaw=0.0, now=0.1 * i)  # konfirme @now=0.3
# Araç hâlâ önde (x=4) ama 3.5s algı yok (> memory_ttl 3.0) → iz düşer
pts, st = m.update([], rx=4.0, ry=0.0, yaw=0.0, now=0.3 + 3.5)
check("ttl aşımında iz düştü, enjekte yok", st["tracks"] == 0 and st["injected"] == 0, f"-> {st}")


print("\n== FP: tek görüş konfirme OLMAZ, enjekte edilmez ==")
m = ObstacleMemory(base_params())
m.update([(5.0, 0.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.0)   # tek görüş (hits=1)
pts, st = m.update([], rx=0.2, ry=0.0, yaw=0.0, now=0.1)
check("tek görüş konfirme değil → enjekte yok", st["injected"] == 0, f"-> {st}")


print("\n== yaklaşılmadıysa (hep uzak) konfirme OLMAZ ==")
m = ObstacleMemory(base_params(confirm_approach_m=7.0))
for i in range(6):
    pts, st = m.update([(10.0, 0.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.1 * i)  # hep 10m > 7m
check("hit çok ama yaklaşma yok → konfirme değil",
      st["confirmed"] == 0, f"-> {st}")
pts, st = m.update([], rx=0.0, ry=0.0, yaw=0.0, now=1.0)
check("yaklaşılmadan enjekte yok", st["injected"] == 0, f"-> {st}")


print("\n== iki ayrı duba ayrı izlenir ==")
m = ObstacleMemory(base_params())
st = None
for i in range(4):
    pts, st = m.update([(6.0, 2.0), (5.0, -2.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.1 * i)
check("iki iz ayrı tutuldu", st["tracks"] == 2, f"-> {st}")


print("\n== enabled=False → pasif (enjekte yok) ==")
m = ObstacleMemory(base_params(enabled=False))
for i in range(5):
    pts, st = m.update([(3.0, 0.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.1 * i)
pts, st = m.update([], rx=1.0, ry=0.0, yaw=0.0, now=1.0)
check("enabled=False → enjekte/iz yok", st["injected"] == 0 and st["tracks"] == 0, f"-> {st}")


print("\n== yaw≠0: dönüş frame'inde konfirme + dropout enjeksiyonu ==")
m = ObstacleMemory(base_params())
yaw = math.pi / 3.0                              # 60° dönük gövde
# Duba gövdede (8,0); araç sabit konumda 3 kez algılıyor → konfirme
for i in range(3):
    pts, st = m.update([(8.0, 0.0)], rx=0.0, ry=0.0, yaw=yaw, now=0.1 * i)
check("yaw≠0 konfirme oldu", st["confirmed"] == 1, f"-> {st}")
# Araç heading boyunca 1m ilerledi → dropout → duba ~7m önde enjekte edilmeli
nx, ny = math.cos(yaw) * 1.0, math.sin(yaw) * 1.0
pts, st = m.update([], rx=nx, ry=ny, yaw=yaw, now=0.4)
inj_pt = pts[-1] if pts else None
check("yaw≠0 dropout'ta doğru bearing'de enjekte (~7m önde, ~0 yanal)",
      st["injected"] == 1 and inj_pt is not None
      and approx(inj_pt[0], 7.0, 0.2) and approx(inj_pt[1], 0.0, 0.2),
      f"-> {inj_pt} {st}")


print("\n== max_tracks tahliyesi konfirme izi KORUR (FP'leri atar) ==")
m = ObstacleMemory(base_params(max_tracks=2))
# A: konfirme duba, gövde (5,0), 3 tick (t=0..0.2)
for i in range(3):
    m.update([(5.0, 0.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.1 * i)
# B: yeni FP (5,3) @t=0.3 → 2 iz; C: yeni FP (5,-3) @t=0.4 → tavan → biri atılmalı
m.update([(5.0, 3.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.3)
m.update([(5.0, -3.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.4)
# Boş tick: konfirme A (en eski last_seen!) hayatta → enjekte; FP'ler değil
pts, st = m.update([], rx=0.0, ry=0.0, yaw=0.0, now=0.5)
check("konfirme A naive-last_seen'e rağmen korundu (FP atıldı, A enjekte)",
      st["injected"] == 1 and st["confirmed"] == 1, f"-> {st}")


print("\n== tek tick'te aynı dubaya 2 nokta → hits ŞİŞMEZ ==")
m = ObstacleMemory(base_params())
m.update([(5.0, 0.0), (5.2, 0.1)], rx=0.0, ry=0.0, yaw=0.0, now=0.0)  # iki yakın centroid
tracks = list(m._tracks.values())
check("çift-nokta tek iz + hits=1 (çift sayım yok)",
      len(tracks) == 1 and tracks[0].hits == 1, f"-> izler={len(tracks)} hits={tracks[0].hits if tracks else '-'}")


print("\n== odom sıçraması → hafıza sıfırlanır ==")
m = ObstacleMemory(base_params(max_pose_jump_m=2.0))
for i in range(3):
    m.update([(5.0, 0.0)], rx=0.0, ry=0.0, yaw=0.0, now=0.1 * i)  # konfirme @ dünya(5,0)
check("sıçrama öncesi iz var", len(m._tracks) == 1)
pts, st = m.update([], rx=5.0, ry=0.0, yaw=0.0, now=0.4)          # 5m sıçrama (>2m)
check("sıçramada hafıza sıfırlandı (şüpheli dünya izleri atıldı)",
      st["tracks"] == 0 and st["injected"] == 0, f"-> {st}")


print("\n" + "=" * 50)
if failures:
    print(f"FAIL: {len(failures)} test başarısız")
    for f in failures:
        print(" -", f)
    sys.exit(1)
print("OK: tüm obstacle_memory testleri geçti")
sys.exit(0)
