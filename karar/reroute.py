#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reroute.py — cone (statik engel) = hedef REROUTE durum makinesi (ÇOKLU KONİ).

NE YAPAR (gelistirme_plani §16 — yeni mimari)
    Engel kaçınması artık control'ün açık-döngü yanal-offset'iyle (eski sollama,
    §12.10/§12.13 H-A ile KALDIRILDI) DEĞİL, planlayıcının (hedef) rotayı dubanın
    etrafından çizmesiyle yapılır. Bu modül o kapalı-döngüyü kurar:

      1. Karar bloklu cone gördüğünde (commit bandı) cone'un DÜNYA konumunu
         latch'ler — cone bir tık görüşten çıksa bile referans kaybolmaz.
      2. hedef'e `/hedef_komut` ile "kenar_blok;-;cx;cy;cone;r" yollar (E-A).
         hedef o kenarı bloklayıp KARŞI-ŞERİT bağlantısıyla rota çizer
         (hedef tarafı = Samed: S-A/S-B/S-C, §16.3).
      3. Cone artık bloklamıyorsa (geçildi/temizlendi) `kenar_serbest` der →
         hedef eski şeride dönüş rotasını restore eder.

ÇOKLU KONİ (kullanıcı 2026-07-04: "ters şeritte yeni engelde de waypoint kapat"):
    Eski sürüm TEK koni latch'liyordu; karşı şeritte İKİNCİ koni görülünce latch
    yeni koniye taşınıyor, koniler <2 m ise hedef bunu "aynı blok taşındı" sanıp
    İLK koninin bloğunu düşürüyordu → rota ilk koninin üstünden geri dönebiliyordu.
    Artık koni başına ayrı kayıt tutulur (match_m eşleme yarıçapı, hedef'in
    _yakin_blok_bul esik=2.0 ile AYNI olmalı): her koninin kendi kenar_blok /
    refresh / kenar_serbest yaşam döngüsü vardır → D* iki bloklu koninin arasından
    ESKİ şeride dönüş rotasını waypoint'lerle çizer.

    Yaşam döngüsü (koni başına):
      • İLK görülme → kenar_blok (hedef: yeni blok → kilit_bypass'lı recalc).
      • Görüldükçe → konum tazele + refresh_s periyodunda kenar_blok refresh
        (hedef BLOK_TTL_S=3s: tazelenmeyen blok kendiliğinden düşer → her canlı
        koninin bloğu bizim refresh'imizle ayakta kalır).
      • max_s boyunca GÖRÜLMEZSE (son görülmeden itibaren; başka koni aktifken
        arkada kalan geçilmiş koni) → kenar_serbest (zaman_asimi). NOT: eski
        sürümde zaman aşımı İLK bloktan itibarendi; sürekli görülen koni de
        düşüyordu. Yeni semantik: görülen koni düşmez, görülmeyen max_s'te düşer.
      • HİÇBİR koni bloklamıyorsa release_clear_ticks tick üst üste → TÜM koniler
        kenar_serbest (cone_temiz; eski tek-koni davranışının genellemesi).

    Komut kanalı tek-String olduğundan tick başına EN FAZLA BİR komut yayınlanır;
    fazlası iç kuyrukta sıraya girer (10 Hz tick × refresh_s=1.0 → max_cones=6
    koniyle bile TTL=3s rahat karşılanır).

KOMUT BİÇİMİ (String prototip — plan §3.2/K-C: latch=False, queue_size=1)
    "kenar_blok;-;<cx>;<cy>;cone;<r>"      cone'u blokla (reroute talebi)
    "kenar_serbest;-;<cx>;<cy>;cone;<r>"   cone temizlendi → kenarı geri yükle
    (taraf alanı "-" = yok sayılır; hedef yalnız cx,cy,r kullanır)

    ROS'suz, saf Python → `python3 karar/test/test_reroute.py` ile test edilir.
    karar_bt_node her tick'te update() çağırır; dönen komutu RosBridge yayınlar.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class RerouteParams:
    enabled: bool = True
    block_radius_m: float = 1.0      # hedef'e bildirilen cone blok yarıçapı (m)
    max_s: float = 15.0              # koni SON GÖRÜLMEDEN sonra bu kadar sn → kenar_serbest
    refresh_s: float = 1.0           # aktif kenar_blok'u bu periyotla tazele (hedef TTL=3s'i besler)
    release_clear_ticks: int = 5     # hiçbir koni N tick üst üste bloklamıyorsa → hepsi kenar_serbest
    match_m: float = 2.0             # gelen koniyi izlenen koniye eşleme yarıçapı (m)
                                     # DİKKAT: hedef _yakin_blok_bul(esik=2.0) ile aynı tutulmalı,
                                     # yoksa karar "yeni koni" derken hedef "aynı blok taşındı" sanır.
    max_cones: int = 6               # aynı anda izlenen koni üst sınırı (kaçak birikime fren)

    @classmethod
    def from_cfg(cls, rr: dict, ov: dict | None = None) -> "RerouteParams":
        """`reroute` bölümünden oku; yoksa eski `overtake` anahtarlarına düş."""
        rr = rr or {}
        ov = ov or {}
        def g(key, default):
            if key in rr:
                return rr[key]
            return ov.get(key, default)
        return cls(
            enabled=bool(g("enabled", True)),
            block_radius_m=float(g("block_radius_m", 1.0)),
            max_s=float(g("max_s", 15.0)),
            refresh_s=float(g("refresh_s", 1.0)),
            release_clear_ticks=int(rr.get("release_clear_ticks", 5)),
            match_m=float(rr.get("match_m", 2.0)),
            max_cones=int(rr.get("max_cones", 6)),
        )


@dataclass
class RerouteResult:
    command: Optional[str] = None     # /hedef_komut'a yayınlanacak (yoksa None)
    event: Optional[tuple] = None     # (faz, dict) karar_logger için (yoksa None)
    active: bool = False


class RerouteManager:
    def __init__(self, params: RerouteParams):
        self.p = params
        # İzlenen koniler: {'x','y','first_t','last_seen_t','last_emit_t'}
        self._cones: list[dict] = []
        # Bekleyen (komut, event) çiftleri — tick başına bir komut yayınlanır
        self._pending: list[tuple] = []
        self._clear_streak = 0

    @property
    def active(self) -> bool:
        return bool(self._cones)

    @property
    def cone_world(self):
        """En son eklenen/izlenen koni (geriye dönük uyumluluk; dış okuyucu log amaçlı)."""
        if self._cones:
            c = self._cones[-1]
            return (c['x'], c['y'])
        return (0.0, 0.0)

    @property
    def n_cones(self) -> int:
        return len(self._cones)

    def _cmd(self, verb: str, x: float, y: float) -> str:
        return f"{verb};-;{x:.2f};{y:.2f};cone;{self.p.block_radius_m:.2f}"

    def reset(self):
        self._cones = []
        self._pending = []
        self._clear_streak = 0

    def _match(self, x: float, y: float) -> Optional[dict]:
        """(x,y)'ye match_m içinde en yakın izlenen koniyi döndürür (yoksa None)."""
        best, best_d = None, self.p.match_m
        for c in self._cones:
            d = math.hypot(c['x'] - x, c['y'] - y)
            if d <= best_d:
                best, best_d = c, d
        return best

    def _release(self, cone: dict, faz: str, ev_extra: dict):
        """Koniyi listeden düşür + kenar_serbest'i kuyruğa koy."""
        self._cones.remove(cone)
        ev = {"cone_dunya": [round(cone['x'], 2), round(cone['y'], 2)],
              "kalan_koni": len(self._cones)}
        ev.update(ev_extra)
        self._pending.append((self._cmd("kenar_serbest", cone['x'], cone['y']),
                              (faz, ev)))

    def update(self, *, reroute_request, cone_world, decision_karar, now) -> RerouteResult:
        if not self.p.enabled:
            return RerouteResult(active=False)

        # GÜVENLİK: acil durusta bloğu serbest bırakma (latch korunur; cone hâlâ
        # orada). E-stop control tarafında (§12.13 H-B); komut/refresh/yaşlanma
        # dondurulur, acil çözülünce devam eder. (Uzun e-stop'ta hedef blokları
        # TTL ile düşebilir; ilk refresh'te yeni-blok olarak geri yüklenir.)
        if decision_karar == "acildurus":
            return RerouteResult(active=self.active)

        cone_valid = (cone_world is not None
                      and math.isfinite(cone_world[0]) and math.isfinite(cone_world[1])
                      and (abs(cone_world[0]) > 1e-6 or abs(cone_world[1]) > 1e-6))

        # ---------------- 1) Gözlemi işle ---------------- #
        if reroute_request and cone_valid:
            self._clear_streak = 0
            x, y = float(cone_world[0]), float(cone_world[1])
            mevcut = self._match(x, y)
            if mevcut is not None:
                # Aynı koni: konumu HER ZAMAN en tazeye çek (gürültülü ilk konumda
                # donma; hedef tarafı taşınma>0.5m'de kendisi reroute tetikler).
                mevcut['x'], mevcut['y'] = x, y
                mevcut['last_seen_t'] = now
            elif len(self._cones) < self.p.max_cones:
                # YENİ koni (karşı şeritteki 2. engel dahil) → hemen kenar_blok:
                # hedef yeni blok görür → kilit_bypass'lı recalc → dönüş rotası.
                yeni = {'x': x, 'y': y, 'first_t': now,
                        'last_seen_t': now, 'last_emit_t': now}
                self._cones.append(yeni)
                ev = {"cone_dunya": [round(x, 2), round(y, 2)],
                      "yaricap_m": self.p.block_radius_m,
                      "n_koni": len(self._cones)}
                self._pending.append((self._cmd("kenar_blok", x, y), ("blok", ev)))
            # max_cones doluysa: yeni koni İZLENMEZ (en yakın koni zaten dur/slow
            # kararını üretiyor; blok listesi şişip rotayı tamamen kilitlemesin).
        else:
            # Bu tick hiçbir koni bloklamıyor → debounce sonrası HEPSİNİ serbest bırak
            self._clear_streak += 1
            if self._clear_streak >= self.p.release_clear_ticks and self._cones:
                for cone in list(self._cones):
                    self._release(cone, "serbest",
                                  {"neden": "cone_temiz",
                                   "clear_ticks": self._clear_streak})

        # ---------------- 2) Görülmeyen koni zaman aşımı ---------------- #
        # Başka koni aktifken (clear_streak sıfırlanırken) arkada kalan geçilmiş
        # koni ancak burada düşer: son görülmeden max_s sonra kenar_serbest.
        for cone in list(self._cones):
            if (now - cone['last_seen_t']) > self.p.max_s:
                self._release(cone, "zaman_asimi",
                              {"neden": "tazelenmedi",
                               "gecen_s": round(now - cone['last_seen_t'], 1)})

        # ---------------- 3) Tick başına TEK komut yayınla ---------------- #
        if self._pending:
            cmd, event = self._pending.pop(0)
            return RerouteResult(command=cmd, event=event, active=self.active)

        # Kuyruk boş → en bayat refresh'i tazele (hedef TTL=3s beslenir)
        due = [c for c in self._cones if (now - c['last_emit_t']) >= self.p.refresh_s]
        if due:
            cone = min(due, key=lambda c: c['last_emit_t'])
            cone['last_emit_t'] = now
            return RerouteResult(command=self._cmd("kenar_blok", cone['x'], cone['y']),
                                 active=True)

        return RerouteResult(active=self.active)
