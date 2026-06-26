#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_memory.py — duba (statik engel) DÜNYA-konum hafızası.

NEDEN (canlı log teşhisi, 2026-06-26 — `manuel_103648Z` slalom koşusu)
    Detektör (`talos_obstacle_detector`) duba ~3 m önümüzdeyken bile arada bir
    KAREYİ DÜŞÜRÜYOR (boş/eksik PoseArray) → `engel_d_center` boşalıyor →
    `EngelMerkezBlokaj` FAILURE → `RerouteKarar` çağrılmıyor → `reroute_request`
    düşüyor → RerouteManager 5 tick sonra `kenar_serbest` veriyor ("cone_temiz")
    → hedef rotayı restore ediyor → duba tekrar görününce re-blok → FLIP-FLOP.
    Bir kez karar `normal/cruise`'a bile düştü (duba 2.95 m'de!). Yani araç
    dubanın konumunu ~her saniye UNUTUYOR.

NE YAPAR (kullanıcı isteği: "konumunu hafızana al; lidardan veri gelmese bile
o duba orada gibi karar ver")
    1. Her PoseArray tick'inde gelen engelleri DÜNYA çerçevesinde iz (track)
       olarak takip eder (en yakın iz ile eşleştir, EMA ile konumu güncelle).
    2. Bir iz "konfirme" olur: yeterince tick üst üste görüldü (hits) VE araç ona
       yeterince yaklaştı (min menzil ≤ approach). = "yakaladıktan ve yaklaştıktan
       sonra aynı konumda olduğunu tespit etmek".
    3. Konfirme iz O TICK canlı algılanmadıysa (dropout), izin dünya konumunu
       güncel araç pozuyla GÖVDE çerçevesine geri-projekte edip füzyon girdisine
       SENTETİK nokta olarak enjekte eder → BT duba hâlâ oradaymış gibi karar verir.
    4. İz şu durumda DÜŞER: araç dubayı GEÇTİ (gövde ileri < -pass_behind), VEYA
       hiç algılanmadan memory_ttl geçti, VEYA mutlak max_memory aşıldı. Geçince
       düşürmek doğal "serbest bırakma" anıdır (hedef o an kendi şeride döner).

TASARIM
    Saf Python, ROS'suz → `python3 karar/test/test_obstacle_memory.py`. ROS bağı
    yalnız `ros_bridge.py`; o, ham PoseArray'i (fwd, lat) gövde noktalarına çevirip
    + araç pozunu (x, y, yaw — odom) verir. Bu modül dünya↔gövde dönüşümlerini
    kendi yapar (avoidance_geometry.obstacle_world_pos ile aynı konvansiyon).

KONVANSİYON (avoidance_geometry ile uyumlu)
    Gövde: REP-103, fwd=ileri(+), lat=sol(+). Dünya: x doğu, y kuzey, yaw CCW.
        wx = rx + fwd*cos(yaw) - lat*sin(yaw)
        wy = ry + fwd*sin(yaw) + lat*cos(yaw)
    Ters (dünya→gövde):
        fwd =  dx*cos(yaw) + dy*sin(yaw)
        lat = -dx*sin(yaw) + dy*cos(yaw)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

_INF = float("inf")


# ----------------------------------------------------------------------- #
# Çerçeve dönüşümleri
# ----------------------------------------------------------------------- #
def body_to_world(fwd: float, lat: float, rx: float, ry: float, yaw: float) -> Tuple[float, float]:
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (rx + fwd * cy - lat * sy, ry + fwd * sy + lat * cy)


def world_to_body(wx: float, wy: float, rx: float, ry: float, yaw: float) -> Tuple[float, float]:
    dx, dy = wx - rx, wy - ry
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (dx * cy + dy * sy, -dx * sy + dy * cy)


# ----------------------------------------------------------------------- #
# Parametreler
# ----------------------------------------------------------------------- #
@dataclass
class MemParams:
    enabled: bool = True
    assoc_radius_m: float = 1.5       # canlı nokta bu kadar yakınsa aynı iz sayılır
    confirm_hits: int = 3             # bu kadar tick eşleşince (+approach) konfirme
    confirm_approach_m: float = 9.0   # iz en az bir kez bu menzile girmiş olmalı ("yaklaştı")
                                      #   = yavasla bandı (en dış tepki) → reroute(6m)'den
                                      #   ÖNCE konfirme → ani-beliren-dubada bile marj (/incele)
    memory_ttl_s: float = 3.0         # konfirme iz canlı algı olmadan bu süre tutulur
    unconfirmed_ttl_s: float = 0.8    # konfirme OLMAMIŞ iz bu süre algılanmazsa düşer (FP temizliği)
    max_memory_s: float = 12.0        # mutlak tavan — yalnız HAFIZAYLA tutulan ize uygulanır
    inject_forward_min_m: float = 0.3 # yalnız bu kadar ÖNDE iz enjekte edilir
    hold_forward_max_m: float = 14.0  # bu menzilden uzak iz enjekte edilmez
    pass_behind_m: float = 1.5        # gövde ileri < -bu → araç geçti → düş (Bee1 ~2.5m boy; arka tampon payı)
    pos_alpha: float = 0.4            # dünya konumu EMA katsayısı (yeni gözleme doğru)
    max_tracks: int = 12              # iz sayısı tavanı (önce konfirme-olmayanı at)
    max_pose_jump_m: float = 2.0      # araç pozu bir tick'te bundan fazla atlarsa (odom reset) → hafıza sıfırla

    def __post_init__(self):
        # Sahada bozuk config canlı node'u çökertmesin → güvenli sınırlara klemple
        # (/incele: confirm_hits=0/ttl=0 patolojik değerleri).
        self.confirm_hits = max(1, int(self.confirm_hits))
        self.memory_ttl_s = max(0.1, float(self.memory_ttl_s))
        self.unconfirmed_ttl_s = max(0.1, float(self.unconfirmed_ttl_s))
        self.max_memory_s = max(self.memory_ttl_s, float(self.max_memory_s))
        self.pos_alpha = min(1.0, max(0.0, float(self.pos_alpha)))
        self.max_tracks = max(1, int(self.max_tracks))
        self.assoc_radius_m = max(0.0, float(self.assoc_radius_m))
        self.max_pose_jump_m = max(0.0, float(self.max_pose_jump_m))

    @classmethod
    def from_cfg(cls, cfg: Optional[dict]) -> "MemParams":
        # YENİ ALAN EKLERKEN: hem yukarıdaki alana hem buraya ekle (programatik bağ yok).
        cfg = cfg or {}
        d = cls()
        return cls(
            enabled=bool(cfg.get("enabled", d.enabled)),
            assoc_radius_m=float(cfg.get("assoc_radius_m", d.assoc_radius_m)),
            confirm_hits=int(cfg.get("confirm_hits", d.confirm_hits)),
            confirm_approach_m=float(cfg.get("confirm_approach_m", d.confirm_approach_m)),
            memory_ttl_s=float(cfg.get("memory_ttl_s", d.memory_ttl_s)),
            unconfirmed_ttl_s=float(cfg.get("unconfirmed_ttl_s", d.unconfirmed_ttl_s)),
            max_memory_s=float(cfg.get("max_memory_s", d.max_memory_s)),
            inject_forward_min_m=float(cfg.get("inject_forward_min_m", d.inject_forward_min_m)),
            hold_forward_max_m=float(cfg.get("hold_forward_max_m", d.hold_forward_max_m)),
            pass_behind_m=float(cfg.get("pass_behind_m", d.pass_behind_m)),
            pos_alpha=float(cfg.get("pos_alpha", d.pos_alpha)),
            max_tracks=int(cfg.get("max_tracks", d.max_tracks)),
            max_pose_jump_m=float(cfg.get("max_pose_jump_m", d.max_pose_jump_m)),
        )


@dataclass
class _Track:
    x: float                       # dünya konumu (EMA)
    y: float
    first_seen: float
    last_seen: float
    hits: int = 1
    min_range_m: float = _INF      # şimdiye dek görülen en yakın gövde menzili ("ne kadar yaklaştı")
    confirmed: bool = False

    def observe(self, wx: float, wy: float, brange: float, alpha: float, now: float):
        a = max(0.0, min(1.0, alpha))
        self.x += a * (wx - self.x)
        self.y += a * (wy - self.y)
        self.hits += 1
        self.last_seen = now
        if brange < self.min_range_m:
            self.min_range_m = brange


# ----------------------------------------------------------------------- #
# Hafıza
# ----------------------------------------------------------------------- #
class ObstacleMemory:
    """Konfirme dubaların dünya konumunu tutar; dropout'ta gövde-frame'e enjekte eder."""

    def __init__(self, params: MemParams):
        self.p = params
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._last_pose: Optional[Tuple[float, float]] = None  # odom-jump tespiti için
        self.last_stats: dict = {"injected": 0, "tracks": 0, "confirmed": 0}

    # -- yardımcılar -- #
    def _nearest(self, wx: float, wy: float) -> Tuple[Optional[int], float]:
        best_id, best_d = None, _INF
        for tid, t in self._tracks.items():
            d = math.hypot(wx - t.x, wy - t.y)
            if d < best_d:
                best_id, best_d = tid, d
        return best_id, best_d

    def _new_track(self, wx: float, wy: float, brange: float, now: float) -> int:
        if len(self._tracks) >= self.p.max_tracks:
            # Tavan koruması: ÖNCE konfirme-olmayan, sonra en uzun süredir eşleşmeyen
            # (en küçük last_seen) izi çıkar. Konfirme duba (dropout köprüsü aktif)
            # taze FP'ler yüzünden düşmesin (/incele algoritma+güvenlik Yüksek).
            oldest = min(self._tracks,
                         key=lambda k: (self._tracks[k].confirmed, self._tracks[k].last_seen))
            del self._tracks[oldest]
        tid = self._next_id
        self._tracks[tid] = _Track(
            x=wx, y=wy, first_seen=now, last_seen=now, hits=1, min_range_m=brange)
        self._next_id += 1
        return tid

    # -- ana giriş -- #
    def update(self, points: Sequence[Sequence[float]],
               rx: float, ry: float, yaw: float, now: float
               ) -> Tuple[List[Tuple[float, float, float]], dict]:
        """Canlı gövde noktalarını al, izleri güncelle, dropout'taki konfirme
        dubaları enjekte ederek artırılmış nokta listesini döndür.

        points: her biri (fwd, lat[, half_w]) olan canlı algı noktaları.
        Döner: (artirilmis_noktalar, stats). stats: injected/tracks/confirmed.
        """
        out: List[Tuple[float, float, float]] = [
            (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0) for p in points
        ]

        if not self.p.enabled:
            self.last_stats = {"injected": 0, "tracks": 0, "confirmed": 0}
            return out, self.last_stats

        # 0) Odom-sıçraması koruması: araç dünya pozu bir tick'te aşırı atladıysa
        #    (gerçek donanımda GPS/SLAM reset → /incele güvenlik Yüksek) tüm dünya
        #    izleri şüpheli → sıfırla; bu tick'ten temiz başla.
        if self._last_pose is not None:
            if math.hypot(rx - self._last_pose[0], ry - self._last_pose[1]) > self.p.max_pose_jump_m:
                self._tracks.clear()
        self._last_pose = (rx, ry)

        # 1) Eşleştirme: her canlı noktayı en yakın ize bağla (yoksa yeni iz).
        #    Tek tick'te aynı ize düşen 2. nokta (gürültülü cluster bölünmesi)
        #    hits'i ŞİŞİRMESİN → matched guard ile çift-sayım engeli (/incele algoritma).
        matched: set[int] = set()
        for p in points:
            fwd, lat = float(p[0]), float(p[1])
            brange = math.hypot(fwd, lat)
            wx, wy = body_to_world(fwd, lat, rx, ry, yaw)
            tid, dist = self._nearest(wx, wy)
            if tid is not None and dist <= self.p.assoc_radius_m:
                if tid in matched:
                    continue                      # aynı tike ait 2. nokta — çift sayma
                self._tracks[tid].observe(wx, wy, brange, self.p.pos_alpha, now)
                matched.add(tid)
            else:
                matched.add(self._new_track(wx, wy, brange, now))

        # 2) Konfirmasyon: yeterli hit + yeterince yaklaşıldı
        for t in self._tracks.values():
            if (not t.confirmed and t.hits >= self.p.confirm_hits
                    and t.min_range_m <= self.p.confirm_approach_m):
                t.confirmed = True

        # 3) Enjeksiyon + budama (tek geçişte). ÖNCE ucuz zaman kontrolleri, sonra
        #    hayatta kalan izler için trig (world_to_body) — /incele performans.
        injected = 0
        confirmed_n = 0
        for tid in list(self._tracks.keys()):
            t = self._tracks[tid]
            ttl = self.p.memory_ttl_s if t.confirmed else self.p.unconfirmed_ttl_s
            if (now - t.last_seen) > ttl:                         # uzun süre algılanmadı → düş
                del self._tracks[tid]; continue
            # Mutlak tavan YALNIZ bu tick canlı görülmeyen (hafızayla tutulan) ize
            # uygulanır → sürekli algılanan gerçek engel 12s'te zorla düşmez (aksi
            # halde 0.3s yeniden-konfirme boşluğu doğardı — /incele güvenlik #5).
            if (tid not in matched) and (now - t.first_seen) > self.p.max_memory_s:
                del self._tracks[tid]; continue

            fwd, lat = world_to_body(t.x, t.y, rx, ry, yaw)
            if fwd < -self.p.pass_behind_m:                       # araç geçti → düş
                del self._tracks[tid]; continue

            if t.confirmed:
                confirmed_n += 1
            # --- Enjeksiyon: konfirme + bu tick canlı eşleşmedi + önümüzde ---
            if (t.confirmed and tid not in matched
                    and self.p.inject_forward_min_m < fwd <= self.p.hold_forward_max_m):
                out.append((fwd, lat, 0.0))
                injected += 1

        self.last_stats = {"injected": injected, "tracks": len(self._tracks),
                           "confirmed": confirmed_n}
        return out, self.last_stats

    def reset(self):
        self._tracks.clear()
        self._last_pose = None
        self.last_stats = {"injected": 0, "tracks": 0, "confirmed": 0}
