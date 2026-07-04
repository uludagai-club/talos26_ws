#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reroute.py (RerouteManager, ÇOKLU KONİ) birim testleri — ROS yok.

    python3 karar/test/test_reroute.py

Cone reroute durum makinesi (§16/E-A,E-B + 2026-07-04 çoklu-koni):
  • bloklu cone → kenar_blok, refresh, temizlenince kenar_serbest (eski davranış)
  • karşı şeritte İKİNCİ koni → AYRI kenar_blok, ilk koninin bloğu KAYBOLMAZ
  • görülmeyen koni max_s sonra düşer (zaman_asimi); görülen koni DÜŞMEZ
  • tick başına tek komut (kuyruk), max_cones tavanı, acildurus'ta koru
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


def base_params(enabled=True, max_cones=6):
    return RerouteParams(enabled=enabled, block_radius_m=1.0, max_s=15.0,
                         refresh_s=1.0, release_clear_ticks=3,
                         match_m=2.0, max_cones=max_cones)


C1 = (3.0, 1.0)
C2 = (9.0, -2.0)          # C1'den >match_m uzakta → ayrı koni
BLOK1 = "kenar_blok;-;3.00;1.00;cone;1.00"
BLOK2 = "kenar_blok;-;9.00;-2.00;cone;1.00"
SERBEST1 = "kenar_serbest;-;3.00;1.00;cone;1.00"
SERBEST2 = "kenar_serbest;-;9.00;-2.00;cone;1.00"


print("== enabled=False → komut yok ==")
m = RerouteManager(base_params(enabled=False))
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
check("enabled=False inaktif", (not r.active) and r.command is None, f"-> {r.command}")

print("\n== aktivasyon: bloklu cone → kenar_blok ==")
m = RerouteManager(base_params())
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
check("kenar_blok komutu", r.command == BLOK1, f"-> {r.command}")
check("aktif", r.active)
check("event blok", r.event is not None and r.event[0] == "blok")
check("event n_koni=1", r.event is not None and r.event[1].get("n_koni") == 1)

print("\n== refresh: pencere içinde komut yok, sonra tazelenir ==")
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.5)
check("refresh penceresi içinde komut yok", r.command is None and r.active, f"-> {r.command}")
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=1.0)
check("refresh sonrası kenar_blok tazelenir", r.command == BLOK1, f"-> {r.command}")

print("\n== aynı koni hafif taşındı (<match_m): YENİ blok AÇILMAZ, konum tazelenir ==")
r = m.update(reroute_request=True, cone_world=(3.4, 1.3), decision_karar="slow", now=1.2)
check("taşınan koni yeni blok açmaz", r.command is None and m.n_cones == 1,
      f"-> {r.command} n={m.n_cones}")
r = m.update(reroute_request=True, cone_world=(3.4, 1.3), decision_karar="slow", now=2.3)
check("refresh güncel konumla çıkar", r.command == "kenar_blok;-;3.40;1.30;cone;1.00",
      f"-> {r.command}")

print("\n== cone temizlendi: debounce sonrası kenar_serbest ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
r = m.update(reroute_request=False, cone_world=C1, decision_karar="normal", now=0.1)
check("1. temiz tick: komut yok, hâlâ aktif", r.command is None and r.active)
r = m.update(reroute_request=False, cone_world=C1, decision_karar="normal", now=0.2)
check("2. temiz tick: komut yok", r.command is None and r.active)
r = m.update(reroute_request=False, cone_world=C1, decision_karar="normal", now=0.3)
check("3. temiz tick (>=release): kenar_serbest", r.command == SERBEST1, f"-> {r.command}")
check("serbest sonrası pasif", not r.active)
check("event serbest", r.event is not None and r.event[0] == "serbest")

print("\n== ÇOKLU KONİ: karşı şeritte 2. koni → ayrı blok, 1. blok KAYBOLMAZ ==")
m = RerouteManager(base_params())
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
check("C1 bloklandı", r.command == BLOK1, f"-> {r.command}")
r = m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=3.0)
check("C2 YENİ kenar_blok", r.command == BLOK2, f"-> {r.command}")
check("iki koni izleniyor", m.n_cones == 2, f"-> n={m.n_cones}")
check("C2 event n_koni=2", r.event is not None and r.event[1].get("n_koni") == 2)
# C2 raporlanırken C1'in refresh'i de dönmeli (hedef TTL=3s beslenmeli)
r = m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=3.1)
check("C1 refresh C2 aktifken de çıkar", r.command == BLOK1, f"-> {r.command}")
r = m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=4.1)
check("sıradaki refresh C2", r.command == BLOK2, f"-> {r.command}")

print("\n== görülmeyen koni zaman aşımı: C2 aktifken C1 max_s sonra düşer ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=1.0)
# C1 artık hiç raporlanmıyor (geçildi); C2 sürekli raporlanıyor
r = m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=15.5)
# 15.5 - 0.0 > 15.0 → C1 serbest kuyruğa girer ve bu tick döner
check("C1 zaman aşımı kenar_serbest", r.command == SERBEST1, f"-> {r.command}")
check("event zaman_asimi", r.event is not None and r.event[0] == "zaman_asimi")
check("C2 hâlâ izleniyor", m.n_cones == 1 and r.active, f"-> n={m.n_cones}")

print("\n== sürekli GÖRÜLEN koni zaman aşımıyla DÜŞMEZ (yeni semantik) ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
r = m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=16.0)
check("görülen koni düşmez (refresh döner)", r.command == BLOK1 and r.active,
      f"-> {r.command}")

print("\n== all-clear: iki koni, tick başına TEK serbest (kuyruk) ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=1.0)
m.update(reroute_request=False, cone_world=None, decision_karar="normal", now=1.1)
m.update(reroute_request=False, cone_world=None, decision_karar="normal", now=1.2)
r = m.update(reroute_request=False, cone_world=None, decision_karar="normal", now=1.3)
check("3. temiz tick: ilk serbest", r.command == SERBEST1, f"-> {r.command}")
r = m.update(reroute_request=False, cone_world=None, decision_karar="normal", now=1.4)
check("4. tick: ikinci serbest (kuyruktan)", r.command == SERBEST2, f"-> {r.command}")
check("hepsi bırakıldı → pasif", not r.active)

print("\n== max_cones tavanı: fazlası izlenmez ==")
m = RerouteManager(base_params(max_cones=2))
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
m.update(reroute_request=True, cone_world=C2, decision_karar="slow", now=0.1)
r = m.update(reroute_request=True, cone_world=(20.0, 5.0), decision_karar="slow", now=0.2)
check("3. koni izlenmedi", m.n_cones == 2, f"-> n={m.n_cones}")
check("3. koni için blok komutu yok", r.command is None, f"-> {r.command}")

print("\n== acildurus: blok KORUNUR (cone hâlâ orada; e-stop control'de) ==")
m = RerouteManager(base_params())
m.update(reroute_request=True, cone_world=C1, decision_karar="slow", now=0.0)
r = m.update(reroute_request=True, cone_world=C1, decision_karar="acildurus", now=0.5)
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
