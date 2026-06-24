#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reroute.py — cone (statik engel) = hedef REROUTE durum makinesi.

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
         hedef eski şeride dönüş rotasını restore eder. Zaman aşımı = fallback
         restore (kenar_serbest susmasın diye).

    Eski overtake.py (sollama + Ackermann geri-dönüş) bu modülle değiştirildi:
    cone artık karşı şeride DİREKSİYONLA değil ROTAYLA geçiliyor (§16.2).

    ROS'suz, saf Python → `python3 karar/test/test_reroute.py` ile test edilir.
    karar_bt_node her tick'te update() çağırır; dönen komutu RosBridge yayınlar.

KOMUT BİÇİMİ (String prototip — plan §3.2/K-C: latch=False, queue_size=1)
    "kenar_blok;-;<cx>;<cy>;cone;<r>"      cone'u blokla (reroute talebi)
    "kenar_serbest;-;<cx>;<cy>;cone;<r>"   cone temizlendi → kenarı geri yükle
    (taraf alanı "-" = yok sayılır; hedef yalnız cx,cy,r kullanır)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class RerouteParams:
    enabled: bool = True
    block_radius_m: float = 1.0      # hedef'e bildirilen cone blok yarıçapı (m)
    max_s: float = 15.0              # reroute zaman aşımı → fallback kenar_serbest
    refresh_s: float = 1.0           # aktif kenar_blok'u bu periyotla tazele (geç abone yakalasın)
    release_clear_ticks: int = 5     # cone N tick üst üste bloklamıyorsa → kenar_serbest

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
        )


@dataclass
class RerouteResult:
    command: Optional[str] = None     # /hedef_komut'a yayınlanacak (yoksa None)
    event: Optional[tuple] = None     # (faz, dict) karar_logger için (yoksa None)
    active: bool = False


class RerouteManager:
    def __init__(self, params: RerouteParams):
        self.p = params
        self._active = False
        self._cone = (0.0, 0.0)       # latched cone dünya konumu
        self._start_t = 0.0
        self._last_emit_t = 0.0
        self._clear_streak = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def cone_world(self):
        return self._cone

    def _blok_cmd(self, verb: str) -> str:
        cx, cy = self._cone
        return f"{verb};-;{cx:.2f};{cy:.2f};cone;{self.p.block_radius_m:.2f}"

    def update(self, *, reroute_request, cone_world, decision_karar, now) -> RerouteResult:
        if not self.p.enabled:
            return RerouteResult(active=False)

        # GÜVENLİK: acil durusta bloğu serbest bırakma (latch korunur; cone hâlâ orada).
        # E-stop control tarafında (§12.13 H-B); reroute durumu acil çözülünce devam eder.
        if decision_karar == "acildurus":
            return RerouteResult(active=self._active)

        cone_valid = (cone_world is not None
                      and math.isfinite(cone_world[0]) and math.isfinite(cone_world[1])
                      and (abs(cone_world[0]) > 1e-6 or abs(cone_world[1]) > 1e-6))

        # ---------------- AKTİF DEĞİL: blok talebini bekle ---------------- #
        if not self._active:
            if not (reroute_request and cone_valid):
                return RerouteResult(active=False)
            self._active = True
            self._cone = (float(cone_world[0]), float(cone_world[1]))
            self._start_t = now
            self._last_emit_t = now
            self._clear_streak = 0
            ev = {"cone_dunya": [round(self._cone[0], 2), round(self._cone[1], 2)],
                  "yaricap_m": self.p.block_radius_m}
            return RerouteResult(command=self._blok_cmd("kenar_blok"),
                                 event=("blok", ev), active=True)

        # ------------------- AKTİF: tazele / serbest bırak / zaman aşımı ----- #
        # Zaman aşımı: kenar_serbest susmasın diye fallback restore
        if (now - self._start_t) > self.p.max_s:
            self._active = False
            ev = {"gecen_s": round(now - self._start_t, 1), "neden": "zaman_asimi"}
            return RerouteResult(command=self._blok_cmd("kenar_serbest"),
                                 event=("zaman_asimi", ev), active=False)

        if reroute_request and cone_valid:
            # Hâlâ bloklu (cone önümüzde). En güncel konuma latch'i tazele + komutu refresh.
            self._cone = (float(cone_world[0]), float(cone_world[1]))
            self._clear_streak = 0
            cmd = None
            if (now - self._last_emit_t) >= self.p.refresh_s:
                self._last_emit_t = now
                cmd = self._blok_cmd("kenar_blok")
            return RerouteResult(command=cmd, event=None, active=True)

        # Cone artık bloklamıyor (geçildi/temizlendi). Debounce → kenar_serbest.
        self._clear_streak += 1
        if self._clear_streak >= self.p.release_clear_ticks:
            self._active = False
            ev = {"neden": "cone_temiz", "clear_ticks": self._clear_streak}
            return RerouteResult(command=self._blok_cmd("kenar_serbest"),
                                 event=("serbest", ev), active=False)
        return RerouteResult(command=None, event=None, active=True)

    def reset(self):
        self._active = False
        self._clear_streak = 0
