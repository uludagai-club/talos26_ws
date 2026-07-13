#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""levha_kisit.py — yön-kısıt levhaları (dönülmez/mecburi/giriş-yok) = hedef KISIT
durum makinesi.

NE YAPAR (reroute.py'nin levha ikizi)
    "Sola dönülmez" gibi levhalar bir karar çıkışı (dur/slow) değil, ROTA
    kısıtıdır: kavşağın yasak koluna girilmemeli. Karar bunu hedef'in zaten
    desteklediği kenar_blok mekanizmasıyla kurar (yeni hedef komutu YOK):

      1. Levha yeterince yakında ve üst üste min_hits tick görülünce levhanın
         DÜNYA konumu çapalanır (araç pozu + ileri mesafe).
      2. Kavşağın yasak koluna denk gelen noktaya /hedef_komut ile
         "kenar_blok;<taraf>;px;py;levha_<isim>;r" bırakılır. hedef o bölgedeki
         kenarlara TOPLAMSAL ceza uygular (BLOK_EK_CEZA, SOFT) → D* öbür kolu
         seçer; yanlış yerleşmiş blok rotayı KİLİTLEMEZ, pahalılaştırır.
      3. Kavşak geçilince (çapa pass_behind_m gerisinde) veya max_s dolunca
         kenar_serbest → hedef bloğu düşürür (kenar_serbest reroute TETİKLEMEZ,
         committed path korunur — hedef tarafındaki off-road fix).

BLOK YERLEŞİMİ (araç yönüne göre, çapa = levha dünya konumu):
      fwd = levha görüldüğü andaki araç yönü; kavşak merkezi ≈ çapa + ileri_ofset_m
      sol   → merkez + yan_ofset_m SOL   (sol kol girişi)
      sag   → merkez + yan_ofset_m SAĞ   (sağ kol girişi)
      ileri → merkez                      (yasak yolun ağzı — girişi olmayan yol)

LEVHA → YERLEŞİM TABLOSU (yeni levha eklemek = tabloya satır eklemek):
      SOLA_DONULMEZ      → (sol,)
      SAGA_DONULMEZ      → (sag,)
      ILERI_MECBURI_YON  → (sol, sag)      yalnız düz
      GIRISI_OLMAYAN_YOL → (ileri,)

SINIRLAR (v1, bilerek):
    • Levha topic'i tek levha taşır (en yakın); DUR + DONULMEZ aynı karede ise
      eşleştirme-tablolu levha önceliklidir → DONULMEZ o karede görünmez.
    • Levha yanal offset'i algıda abs() → levhanın yolun hangi tarafında
      olduğu bilinmez; kısıtın BİZE ait olduğu varsayılır (karşı yönün
      levhasını da üstümüze alabiliriz — soft ceza bunu tolere eder).
    • Monoküler mesafe kaba (K/bbox_h); çapa görüldükçe en taze değere çekilir,
      hedef taşınmayı (>0.5 m) kendi reroute'uyla düzeltir.

    Komut kanalı (/hedef_komut) RerouteManager ile PAYLAŞILIR: karar_bt_node
    cone komutu yayınladığı tick'te channel_busy=True verir → bu manager o tick
    susar (iç kuyruk korunur). Tick başına EN FAZLA BİR komut yayınlanır.

    ROS'suz, saf Python → `python3 karar/test/test_levha_kisit.py` ile test edilir.
    karar_bt_node her tick'te update() çağırır; dönen komutu RosBridge yayınlar.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


# Levha adı → blok yerleşimleri. Adlar levha node passthrough'unun ürettiği
# ham YOLO sınıf adlarının UPPER halidir (algi/levha SINIF_ESLESTIRME dışı).
YASAK_YERLESIM = {
    "SOLA_DONULMEZ":      ("sol",),
    "SAGA_DONULMEZ":      ("sag",),
    "ILERI_MECBURI_YON":  ("sol", "sag"),
    "GIRISI_OLMAYAN_YOL": ("ileri",),
}


@dataclass
class LevhaKisitParams:
    enabled: bool = True
    etki_m: float = 15.0          # levha bu ileri mesafeden yakınsa çapalanır
    min_hits: int = 3             # üst üste bu kadar tick görülmeden track açılmaz (misdetect freni)
    hit_gap_s: float = 0.5        # hit sayacı bu süreden uzun kesintide sıfırlanır
    ileri_ofset_m: float = 4.0    # çapa → kavşak merkezi (levha kavşaktan önce dikilir)
    yan_ofset_m: float = 5.0      # kavşak merkezi → yasak kol girişi (yanal)
    block_radius_m: float = 2.5   # hedef'e bildirilen blok yarıçapı; yan_ofset - r
                                  # düz şeridin dışında kalmalı (5.0-2.5=2.5m marj)
    refresh_s: float = 1.0        # aktif blokları bu periyotla tazele (hedef TTL=3s'i besler)
    pass_behind_m: float = 8.0    # çapa bu kadar GERİDE kalınca kısıt biter; ileri_ofset_m'den
                                  # BÜYÜK tut (kavşak merkezi geçilmeden serbest bırakma)
    max_s: float = 45.0           # mutlak tavan (kırmızıda beklerken kısıt düşmesin diye uzun;
                                  # yanlış çapalı soft blok en fazla bu kadar yaşar)
    levha_max_age_s: float = 0.6  # freshness.levha_max_age_s aynası
    odom_max_age_s: float = 0.5   # freshness.odom_max_age_s aynası

    @classmethod
    def from_cfg(cls, lk: dict, fresh: dict | None = None) -> "LevhaKisitParams":
        lk = lk or {}
        fresh = fresh or {}
        d = cls()
        return cls(
            enabled=bool(lk.get("enabled", d.enabled)),
            etki_m=float(lk.get("etki_m", d.etki_m)),
            min_hits=int(lk.get("min_hits", d.min_hits)),
            hit_gap_s=float(lk.get("hit_gap_s", d.hit_gap_s)),
            ileri_ofset_m=float(lk.get("ileri_ofset_m", d.ileri_ofset_m)),
            yan_ofset_m=float(lk.get("yan_ofset_m", d.yan_ofset_m)),
            block_radius_m=float(lk.get("block_radius_m", d.block_radius_m)),
            refresh_s=float(lk.get("refresh_s", d.refresh_s)),
            pass_behind_m=float(lk.get("pass_behind_m", d.pass_behind_m)),
            max_s=float(lk.get("max_s", d.max_s)),
            levha_max_age_s=float(fresh.get("levha_max_age_s", d.levha_max_age_s)),
            odom_max_age_s=float(fresh.get("odom_max_age_s", d.odom_max_age_s)),
        )


@dataclass
class LevhaKisitResult:
    command: Optional[str] = None   # /hedef_komut'a yayınlanacak (yoksa None)
    event: Optional[tuple] = None   # (faz, dict) log için (yoksa None)
    active: bool = False


class LevhaKisitManager:
    def __init__(self, params: LevhaKisitParams):
        self.p = params
        # Aktif kısıtlar: {isim: {'ax','ay','yaw','first_t','last_seen_t',
        #                         'noktalar': {taraf: {'x','y','last_emit_t'}}}}
        self._aktif: dict[str, dict] = {}
        # Track öncesi hit sayacı: {isim: {'n': int, 't': float}}
        self._hits: dict[str, dict] = {}
        # Bekleyen (komut, event) çiftleri — tick başına bir komut yayınlanır
        self._pending: list[tuple] = []

    @property
    def active(self) -> bool:
        return bool(self._aktif)

    @property
    def n_aktif(self) -> int:
        return len(self._aktif)

    # ------------------------------------------------------------
    def _nokta_hesapla(self, tr: dict, taraf: str) -> tuple:
        """Track çapası + yönünden blok noktasının dünya koordinatı."""
        fx, fy = math.cos(tr['yaw']), math.sin(tr['yaw'])
        lx, ly = -fy, fx                                   # sol birim vektör
        px = tr['ax'] + self.p.ileri_ofset_m * fx
        py = tr['ay'] + self.p.ileri_ofset_m * fy
        if taraf == "sol":
            px += self.p.yan_ofset_m * lx
            py += self.p.yan_ofset_m * ly
        elif taraf == "sag":
            px -= self.p.yan_ofset_m * lx
            py -= self.p.yan_ofset_m * ly
        # "ileri" → kavşak merkezi (ofsetsiz)
        return px, py

    def _cmd(self, verb: str, isim: str, taraf: str, x: float, y: float) -> str:
        return (f"{verb};{taraf};{x:.2f};{y:.2f};"
                f"levha_{isim.lower()};{self.p.block_radius_m:.2f}")

    def _release(self, isim: str, neden: str):
        """Kısıtı düşür: yerleşen HER noktaya son yayınlanan koordinatla
        kenar_serbest kuyruğa (hedef _yakin_blok_bul 2m eşleşmesi için son
        emit koordinatı ŞART — çapa güncellenmiş olabilir)."""
        tr = self._aktif.pop(isim)
        for taraf, n in tr['noktalar'].items():
            ev = {"levha": isim, "taraf": taraf, "neden": neden,
                  "nokta": [round(n['x'], 2), round(n['y'], 2)],
                  "kalan_kisit": len(self._aktif)}
            self._pending.append(
                (self._cmd("kenar_serbest", isim, taraf, n['x'], n['y']),
                 ("serbest", ev)))

    def reset(self):
        self._aktif = {}
        self._hits = {}
        self._pending = []

    # ------------------------------------------------------------
    def update(self, *, levha_isim, levha_ileri_m, levha_age_s,
               pose, odom_age_s, decision_karar, channel_busy, now) -> LevhaKisitResult:
        if not self.p.enabled:
            return LevhaKisitResult(active=False)

        # Acil duruşta durumu DONDUR (reroute.py ile aynı sözleşme): komut,
        # tazeleme ve yaşlanma durur; acil çözülünce kaldığı yerden sürer.
        if decision_karar == "acildurus":
            return LevhaKisitResult(active=self.active)

        x, y, yaw = pose
        odom_taze = odom_age_s <= self.p.odom_max_age_s

        # ---------------- 1) Gözlemi işle ---------------- #
        isim = (levha_isim or "").upper()
        if (isim in YASAK_YERLESIM
                and levha_age_s <= self.p.levha_max_age_s
                and odom_taze
                and levha_ileri_m is not None
                and 0.0 < levha_ileri_m <= self.p.etki_m):
            ax = x + math.cos(yaw) * levha_ileri_m
            ay = y + math.sin(yaw) * levha_ileri_m
            tr = self._aktif.get(isim)
            if tr is not None:
                # Görüldükçe çapayı EN TAZE değere çek (reroute ile aynı ilke);
                # blok noktaları refresh'te güncel çapayla yeniden hesaplanır,
                # hedef taşınmayı (>0.5m) kendisi reroute'lar.
                tr['ax'], tr['ay'], tr['yaw'] = ax, ay, yaw
                tr['last_seen_t'] = now
            else:
                h = self._hits.get(isim)
                if h is None or (now - h['t']) > self.p.hit_gap_s:
                    h = {'n': 0, 't': now}
                h['n'] += 1
                h['t'] = now
                self._hits[isim] = h
                if h['n'] >= self.p.min_hits:
                    del self._hits[isim]
                    tr = {'ax': ax, 'ay': ay, 'yaw': yaw,
                          'first_t': now, 'last_seen_t': now, 'noktalar': {}}
                    self._aktif[isim] = tr
                    for taraf in YASAK_YERLESIM[isim]:
                        px, py = self._nokta_hesapla(tr, taraf)
                        tr['noktalar'][taraf] = {'x': px, 'y': py, 'last_emit_t': now}
                        ev = {"levha": isim, "taraf": taraf,
                              "capa": [round(ax, 2), round(ay, 2)],
                              "nokta": [round(px, 2), round(py, 2)],
                              "yaricap_m": self.p.block_radius_m,
                              "n_kisit": len(self._aktif)}
                        self._pending.append(
                            (self._cmd("kenar_blok", isim, taraf, px, py),
                             ("blok", ev)))

        # ---------------- 2) Geçilme / zaman aşımı ---------------- #
        for isim_a in list(self._aktif.keys()):
            tr = self._aktif[isim_a]
            if odom_taze:
                # Çapanın araç frame'inde ileri bileşeni; yeterince geride → geçildi.
                dx, dy = tr['ax'] - x, tr['ay'] - y
                ileri = math.cos(yaw) * dx + math.sin(yaw) * dy
                if ileri < -self.p.pass_behind_m:
                    self._release(isim_a, "gecildi")
                    continue
            if (now - tr['first_t']) > self.p.max_s:
                self._release(isim_a, "zaman_asimi")

        # ---------------- 3) Tick başına TEK komut (kanal boşsa) ---------------- #
        if channel_busy:
            return LevhaKisitResult(active=self.active)

        if self._pending:
            cmd, event = self._pending.pop(0)
            return LevhaKisitResult(command=cmd, event=event, active=self.active)

        # Kuyruk boş → en bayat noktayı güncel çapayla tazele (hedef TTL beslenir)
        en_bayat = None
        for isim_a, tr in self._aktif.items():
            for taraf, n in tr['noktalar'].items():
                if (now - n['last_emit_t']) >= self.p.refresh_s:
                    if en_bayat is None or n['last_emit_t'] < en_bayat[2]['last_emit_t']:
                        en_bayat = (isim_a, taraf, n, tr)
        if en_bayat is not None:
            isim_a, taraf, n, tr = en_bayat
            px, py = self._nokta_hesapla(tr, taraf)
            n['x'], n['y'], n['last_emit_t'] = px, py, now
            return LevhaKisitResult(
                command=self._cmd("kenar_blok", isim_a, taraf, px, py),
                active=True)

        return LevhaKisitResult(active=self.active)
