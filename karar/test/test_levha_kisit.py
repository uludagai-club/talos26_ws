#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""levha_kisit.py (LevhaKisitManager) birim testleri — ROS yok.

    python3 karar/test/test_levha_kisit.py

Levha yön-kısıtı durum makinesi:
  • SOLA_DONULMEZ min_hits sonra sol kola kenar_blok; geometri (yaw dahil) doğru
  • ILERI_MECBURI_YON iki blok (sol+sag), tick başına tek komut
  • channel_busy tick'inde susar (kuyruk korunur)
  • geçilince / max_s dolunca kenar_serbest (SON emit koordinatıyla)
  • acildurus dondurur; bilinmeyen levha (DUR) hiçbir şey yapmaz
  • bayat levha/odom çapalamaz; refresh güncel çapayla çıkar
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from levha_kisit import LevhaKisitManager, LevhaKisitParams  # noqa: E402

failures = []


def check(name, cond, extra=""):
    if cond:
        print(f"  ✓ {name}")
    else:
        failures.append(name)
        print(f"  ✗ {name} {extra}")


def base_params(**kw):
    d = dict(enabled=True, etki_m=15.0, min_hits=3, hit_gap_s=0.5,
             ileri_ofset_m=4.0, yan_ofset_m=5.0, block_radius_m=2.5,
             refresh_s=1.0, pass_behind_m=8.0, max_s=45.0,
             levha_max_age_s=0.6, odom_max_age_s=0.5)
    d.update(kw)
    return LevhaKisitParams(**d)


def gor(m, isim, d_ileri, pose, now, *, busy=False, age=0.0, odom_age=0.0,
        karar="normal"):
    """Tek tick: levha gözlemiyle update."""
    return m.update(levha_isim=isim, levha_ileri_m=d_ileri, levha_age_s=age,
                    pose=pose, odom_age_s=odom_age, decision_karar=karar,
                    channel_busy=busy, now=now)


P0 = (0.0, 0.0, 0.0)   # orijin, yaw=0 (doğu)

print("== enabled=False → komut yok ==")
m = LevhaKisitManager(base_params(enabled=False))
for i in range(5):
    r = gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1)
check("enabled=False inaktif", (not r.active) and r.command is None, f"-> {r.command}")

print("\n== SOLA_DONULMEZ: min_hits + geometri (yaw=0) ==")
# çapa=(10,0); merkez=çapa+4*fwd=(14,0); sol nokta=merkez+5*sol=(14,5)
m = LevhaKisitManager(base_params())
r1 = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.0)
r2 = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.1)
check("min_hits öncesi komut yok", r1.command is None and r2.command is None,
      f"-> {r1.command}, {r2.command}")
r3 = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.2)
check("3. hit'te kenar_blok",
      r3.command == "kenar_blok;sol;14.00;5.00;levha_sola_donulmez;2.50",
      f"-> {r3.command}")
check("aktif", r3.active)
check("event blok", r3.event is not None and r3.event[0] == "blok")

print("\n== geometri yaw=90° (kuzey) ==")
# pose=(0,0,pi/2); çapa=(0,10); fwd=(0,1); sol=(-1,0); merkez=(0,14); sol nokta=(-5,14)
m = LevhaKisitManager(base_params())
PN = (0.0, 0.0, math.pi / 2)
for i in range(3):
    r = gor(m, "SOLA_DONULMEZ", 10.0, PN, i * 0.1)
check("yaw=90° sol nokta (-5,14)",
      r.command == "kenar_blok;sol;-5.00;14.00;levha_sola_donulmez;2.50",
      f"-> {r.command}")

print("\n== SAGA_DONULMEZ sağ kol ==")
# merkez=(14,0); sag nokta=(14,-5)
m = LevhaKisitManager(base_params())
for i in range(3):
    r = gor(m, "SAGA_DONULMEZ", 10.0, P0, i * 0.1)
check("sag nokta (14,-5)",
      r.command == "kenar_blok;sag;14.00;-5.00;levha_saga_donulmez;2.50",
      f"-> {r.command}")

print("\n== ILERI_MECBURI_YON: iki blok, tick başına tek komut ==")
m = LevhaKisitManager(base_params())
gor(m, "ILERI_MECBURI_YON", 10.0, P0, 0.0)
gor(m, "ILERI_MECBURI_YON", 10.0, P0, 0.1)
r3 = gor(m, "ILERI_MECBURI_YON", 10.0, P0, 0.2)
r4 = gor(m, "ILERI_MECBURI_YON", 10.0, P0, 0.3)
check("1. komut sol", r3.command is not None and ";sol;" in r3.command, f"-> {r3.command}")
check("2. komut sag (kuyruktan)", r4.command is not None and ";sag;" in r4.command,
      f"-> {r4.command}")

print("\n== GIRISI_OLMAYAN_YOL: kavşak merkezine (ofsetsiz) blok ==")
m = LevhaKisitManager(base_params())
for i in range(3):
    r = gor(m, "GIRISI_OLMAYAN_YOL", 10.0, P0, i * 0.1)
check("ileri nokta (14,0)",
      r.command == "kenar_blok;ileri;14.00;0.00;levha_girisi_olmayan_yol;2.50",
      f"-> {r.command}")

print("\n== channel_busy: komut tutulur, sonraki tick çıkar ==")
m = LevhaKisitManager(base_params())
gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.0)
gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.1)
r3 = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.2, busy=True)
check("busy tick'te komut yok ama aktif", r3.command is None and r3.active,
      f"-> {r3.command}")
r4 = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.3)
check("kanal açılınca kuyruktan çıkar",
      r4.command is not None and r4.command.startswith("kenar_blok;sol;"),
      f"-> {r4.command}")

print("\n== refresh: pencere içinde sus, sonra güncel çapayla tazele ==")
m = LevhaKisitManager(base_params())
for i in range(3):
    gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1)
r = gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.5)
check("refresh penceresi içinde komut yok", r.command is None and r.active,
      f"-> {r.command}")
# araç 2m ilerledi, levha 8m önde → çapa yine (10,0); ama levha 9m ölçülsün → çapa (11,0)
r = gor(m, "SOLA_DONULMEZ", 9.0, (2.0, 0.0, 0.0), 1.3)
check("refresh güncel çapayla (15,5)",
      r.command == "kenar_blok;sol;15.00;5.00;levha_sola_donulmez;2.50",
      f"-> {r.command}")

print("\n== geçilme: çapa pass_behind gerisinde → kenar_serbest (son koordinatla) ==")
m = LevhaKisitManager(base_params())
for i in range(3):
    gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1)     # çapa (10,0), nokta (14,5)
# araç 19m'de: çapa 9m geride (>8) — levha artık görünmüyor (NONE)
r = gor(m, "NONE", -1.0, (19.0, 0.0, 0.0), 1.0)
check("geçilince kenar_serbest",
      r.command == "kenar_serbest;sol;14.00;5.00;levha_sola_donulmez;2.50",
      f"-> {r.command}")
check("event serbest/gecildi",
      r.event is not None and r.event[0] == "serbest"
      and r.event[1].get("neden") == "gecildi")
check("kısıt düştü", not r.active)

print("\n== max_s zaman aşımı → kenar_serbest ==")
m = LevhaKisitManager(base_params(max_s=5.0))
for i in range(3):
    gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1)
r = gor(m, "NONE", -1.0, P0, 3.0)                   # ara tick (refresh çıkabilir)
r = gor(m, "NONE", -1.0, P0, 5.5)
check("max_s sonrası kenar_serbest",
      r.command is not None and r.command.startswith("kenar_serbest;"),
      f"-> {r.command}")
check("neden zaman_asimi",
      r.event is not None and r.event[1].get("neden") == "zaman_asimi")

print("\n== acildurus: dondurma (komut yok, yaşlanma yok) ==")
m = LevhaKisitManager(base_params(max_s=5.0))
for i in range(3):
    gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1)
r = gor(m, "NONE", -1.0, P0, 100.0, karar="acildurus")   # max_s çoktan geçti
check("acildurus'ta komut yok + aktif", r.command is None and r.active,
      f"-> {r.command}")

print("\n== bilinmeyen levha (DUR) / NONE → hiçbir şey ==")
m = LevhaKisitManager(base_params())
for i in range(6):
    r = gor(m, "DUR", 5.0, P0, i * 0.1)
check("DUR kısıt açmaz", (not r.active) and r.command is None, f"-> {r.command}")

print("\n== bayat levha / bayat odom / menzil dışı çapalamaz ==")
m = LevhaKisitManager(base_params())
for i in range(6):
    r = gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1, age=1.5)
check("bayat levha çapalamaz", not m.active)
m = LevhaKisitManager(base_params())
for i in range(6):
    r = gor(m, "SOLA_DONULMEZ", 10.0, P0, i * 0.1, odom_age=2.0)
check("bayat odom çapalamaz", not m.active)
m = LevhaKisitManager(base_params())
for i in range(6):
    r = gor(m, "SOLA_DONULMEZ", 20.0, P0, i * 0.1)
check("etki_m dışı çapalamaz", not m.active)

print("\n== hit_gap: kesintili tespit sayacı sıfırlar ==")
m = LevhaKisitManager(base_params())
gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.0)
gor(m, "SOLA_DONULMEZ", 10.0, P0, 0.1)
gor(m, "NONE", -1.0, P0, 0.2)
gor(m, "NONE", -1.0, P0, 0.9)                       # >hit_gap_s boşluk
r = gor(m, "SOLA_DONULMEZ", 10.0, P0, 1.0)          # sayaç 1'den başlamalı
check("kesinti sonrası track açılmaz", not m.active and r.command is None,
      f"-> {r.command}")

print()
if failures:
    print(f"BAŞARISIZ: {len(failures)} test: {failures}")
    sys.exit(1)
print("TÜM TESTLER GEÇTİ ✓")
