#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""avoidance_geometry.py — engelden kaçış için saf geometri yardımcıları.

ROS'suz, saf Python → `python3 karar/test/test_avoidance_geometry.py` ile
test edilebilir. Hem `behaviors/conditions.py` (yol-bilinçli sol/sağ seçimi)
hem `overtake.py` (sollama/geri-dönüş durum makinesi) buradan beslenir.

KONVANSİYON
    Dünya çerçevesi: REP-103, x ileri/doğu, y sol/kuzey, yaw CCW (sol pozitif).
    Engel füzyon açısı (`engel_angle_deg`): SAĞ POZİTİF (eski engel_node ile
    uyumlu) → fusion: angle_deg = -degrees(atan2(lat, fwd)).
    "Yanal" (signed lateral): bir doğrunun SOLU pozitif (sağ el kuralı, z yukarı).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple


# ----------------------------------------------------------------------- #
# Temel
# ----------------------------------------------------------------------- #
def _hypot(ax, ay, bx, by) -> float:
    return math.hypot(ax - bx, ay - by)


def obstacle_world_pos(rx: float, ry: float, yaw: float,
                       d_range: float, angle_deg: float) -> Tuple[float, float]:
    """Engelin gövde-çerçevesi (menzil, açı) ölçümünü dünya konumuna çevirir.

    angle_deg SAĞ POZİTİF (füzyon konvansiyonu). Gövde çerçevesinde:
        fwd =  d * cos(angle)   (araç önü +)
        lat = -d * sin(angle)   (araç solu +, sağ açı negatif lat verir)
    Dünya: araç yaw'ı kadar döndür + araç konumuna taşı.
    """
    a = math.radians(angle_deg)
    fwd = d_range * math.cos(a)
    lat = -d_range * math.sin(a)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    ox = rx + fwd * cos_y - lat * sin_y
    oy = ry + fwd * sin_y + lat * cos_y
    return ox, oy


def project_to_segment(px: float, py: float,
                       ax: float, ay: float,
                       bx: float, by: float) -> Tuple[float, float, float]:
    """(px,py) noktasını a→b doğru parçasına projekte eder.

    Döner: (t, s_long, lateral_signed)
        t            : 0=a, 1=b (kırpılmamış; <0 veya >1 segment dışıdır)
        s_long       : a'dan itibaren a→b yönünde uzunlamasına mesafe (m)
        lateral_signed: noktanın doğruya işaretli dik uzaklığı; doğrunun
                        SOLU pozitif (sağ el kuralı). |lateral| = gerçek uzaklık.

    Segment dejenere (a==b, ör. rota sonunda wp1==wp2 padding'i) ise yön TANIMSIZ
    → (0, 0, 0.0) döner. lateral=0 → side_to_avoid deadband'e düşer → yan_sektor
    fallback (en açık taraf) devreye girer. işaretsiz hypot dönmek HER ZAMAN "sag"
    seçtirirdi — yanlış yön riski (/incele algoritma bulgusu, 2026-06-24).
    """
    abx, aby = bx - ax, by - ay
    seg_len2 = abx * abx + aby * aby
    apx, apy = px - ax, py - ay
    if seg_len2 <= 1e-9:
        return 0.0, 0.0, 0.0
    seg_len = math.sqrt(seg_len2)
    t = (apx * abx + apy * aby) / seg_len2
    s_long = t * seg_len
    # cross(ab, ap) / |ab|  → sol pozitif
    cross = abx * apy - aby * apx
    lateral_signed = cross / seg_len
    return t, s_long, lateral_signed


def side_to_avoid(obs_x: float, obs_y: float,
                  ax: float, ay: float, bx: float, by: float,
                  deadband_m: float = 0.4) -> Tuple[Optional[str], float]:
    """Engelin rota doğrusuna (a→b) göre konumundan kaçış yönünü seçer.

    Engel rotanın SOLUNDAysa → SAĞA kaç ("sag"); SAĞINDAysa → SOLA kaç ("sol").
    |yanal| < deadband (engel ~rota üzerinde, tipik koni) → None: çağıran
    varsayılan sollama yönüne / yan-sektör fallback'ine düşer.

    Döner: (taraf|None, lateral_signed). lateral_signed sol-pozitif.
    """
    _, _, lateral = project_to_segment(obs_x, obs_y, ax, ay, bx, by)
    if abs(lateral) < deadband_m:
        return None, lateral
    # engel solda (lateral>0) → sağa kaç; engel sağda → sola kaç
    return ("sag" if lateral > 0.0 else "sol"), lateral


def longitudinal_gap(robot_x: float, robot_y: float,
                     obs_x: float, obs_y: float,
                     dir_x: float, dir_y: float) -> float:
    """Robotun engeli rota yönünde NE KADAR GEÇTİĞİ (m, pozitif = robot önde).

    dir: rota ilerleme yön vektörü (birim olması gerekmez). Robotun engele göre
    bağıl konumunun bu yöndeki izdüşümü.
    """
    dn = math.hypot(dir_x, dir_y)
    if dn <= 1e-9:
        return 0.0
    ux, uy = dir_x / dn, dir_y / dn
    return (robot_x - obs_x) * ux + (robot_y - obs_y) * uy


# ----------------------------------------------------------------------- #
# Ackermann
# ----------------------------------------------------------------------- #
def ackermann_radius(steer_deg: float, wheelbase_m: float) -> float:
    """Direksiyon açısına karşılık dönüş yarıçapı R = L / tan(δ).

    steer_deg ~0 → inf (düz gidiş). Negatif/iş yok: mutlak değer alınır.
    """
    a = abs(math.radians(steer_deg))
    if a < 1e-4:
        return float("inf")
    return wheelbase_m / math.tan(a)


def lane_change_longitudinal(radius_m: float, lateral_offset_m: float) -> float:
    """Tek yay ile `lateral_offset` kadar yanal kayma için gereken uzunlamasına
    yol: x = sqrt(2*R*Δ - Δ²)  (Δ ≤ R).

    R = inf (düz) → 0 yanal kayma imkânsız sayılır, çok büyük döndürülür değil:
    fiziksel olarak düz giderken yanal kayma 0; bu durumda gereken uzunluk
    tanımsız → güvenli tarafta büyük bir sayı yerine 0 (çağıran clearance ekler).
    """
    if not math.isfinite(radius_m):
        return 0.0
    delta = min(abs(lateral_offset_m), radius_m)
    return math.sqrt(max(0.0, 2.0 * radius_m * delta - delta * delta))


def ackermann_return_distance(lane_offset_m: float, return_steer_deg: float,
                              wheelbase_m: float, clearance_m: float) -> float:
    """Sollamadan sonra ÇARPMADAN şeride dönmek için engeli uzunlamasına ne kadar
    GEÇMİŞ olmak gerektiği (m).

    Mantık: araç, `return_steer_deg` ile (yarıçap R) bir dönüş yayı çizerek
    `lane_offset` kadar geri kayar. Bu yay uzunlamasına `arc_long` yol kaplar.
    Yayın TAMAMI engelin ilerisinde kalsın diye dönüşe ancak engeli
    (arc_long + clearance) kadar geçince başlanır → iç süpürme engele değmez.
    clearance: araç ön çıkıntısı + engel yarıçapı + emniyet payı.
    """
    R = ackermann_radius(return_steer_deg, wheelbase_m)
    arc_long = lane_change_longitudinal(R, lane_offset_m)
    return arc_long + max(0.0, clearance_m)


def required_steer_deg(lateral_offset_m: float, longitudinal_m: float,
                       wheelbase_m: float) -> float:
    """`longitudinal` yolda `lateral_offset` kaymak için gereken direksiyon (derece).

    Tek-yay tersi: x²+Δ² = 2RΔ → R = (x²+Δ²)/(2Δ); δ = atan(L/R).
    Fizibilite kontrolü için (δ ≤ MAX_STEER mı?). Δ veya x ~0 → 90° (imkânsıza
    yakın) döner.
    """
    dx = abs(longitudinal_m)
    dy = abs(lateral_offset_m)
    if dy < 1e-6:
        return 0.0
    if dx < 1e-6:
        return 90.0
    R = (dx * dx + dy * dy) / (2.0 * dy)
    return math.degrees(math.atan2(wheelbase_m, R))


def avoidance_feasible(lateral_offset_m: float, longitudinal_m: float,
                       wheelbase_m: float, max_steer_deg: float) -> bool:
    """Verilen uzunlamasına mesafede istenen yanal kaçış, MAX_STEER içinde
    Ackermann ile yapılabilir mi?"""
    return required_steer_deg(lateral_offset_m, longitudinal_m, wheelbase_m) <= max_steer_deg + 1e-6
