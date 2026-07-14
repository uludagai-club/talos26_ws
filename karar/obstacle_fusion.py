"""Engel füzyonu — nokta/kutu listesinden BT'nin beklediği skaler alanlara.

Yeni `talos_obstacle_detector` (DBSCAN + PCA OBB) engelleri `/obstacles/poses`
(geometry_msgs/PoseArray) olarak konum listesi şeklinde yayınlar. BT ise eski
`engel_node` arayüzünden gelen skaler alanları (merkez/sol/sağ minimum mesafe,
açı, present) bekler. Bu modül ikisini köprüler.

ÖNEMLİ: Bu dosya rospy'siz saf Python'dur — `behaviors/` ve `trees/` gibi ROS
olmadan da test edilebilir. ROS bağı yalnız `ros_bridge.py` içindedir; o, ham
PoseArray'i `(forward_m, left_m, half_width_m)` üçlülerine çevirip buraya verir.

Konvansiyon (girdi): araç gövde çerçevesi, REP-103 —
  forward_m > 0 : araç önü
  left_m   > 0 : aracın solu
  half_width_m : engelin yarı yanal genişliği (kutu yoksa 0.0)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

_INF = float("inf")


@dataclass
class ObstacleFusionParams:
    corridor_half_w_m: float = 1.0   # araç koridoru yarı genişliği → merkez sektör
    forward_min_m: float = 0.0       # bu değerden ileri engeller sayılır (önümüz)
    forward_max_m: float = 30.0      # merkez/açı için ileri bakış sınırı
    side_forward_max_m: float = 8.0  # yan sektör (lane-change) için ileri sınır

    @classmethod
    def from_cfg(cls, cfg: dict) -> "ObstacleFusionParams":
        cfg = cfg or {}
        return cls(
            corridor_half_w_m=float(cfg.get("corridor_half_width_m", 1.0)),
            forward_min_m=float(cfg.get("forward_min_m", 0.0)),
            forward_max_m=float(cfg.get("forward_max_m", 30.0)),
            side_forward_max_m=float(cfg.get("side_forward_max_m", 8.0)),
        )


@dataclass
class FusedObstacles:
    present: bool = False
    d_center: float = _INF
    d_left: float = _INF
    d_right: float = _INF
    d_overall: float = _INF
    angle_deg: float = 0.0    # en yakın engelin açısı; sağ pozitif (eski node ile uyumlu)
    count: int = 0            # ileri bakış içindeki engel sayısı (debug)


def fuse_obstacles(points: Iterable[Sequence[float]],
                   p: ObstacleFusionParams) -> FusedObstacles:
    """Engel konum listesini BT skaler alanlarına indirger.

    points: her biri (forward_m, left_m[, half_width_m]) olan dizilerin listesi.

    Sektör tanımı:
      - MERKEZ : önümüzde (forward>min, <max) ve yanal kenarı koridor içinde
                 (|left| - half_w <= corridor_half_w). Bunlar fren/acil kararı.
      - SOL    : yanal olarak SOL şeritte (left - half_w > corridor_half_w) ve
                 ileri bakış içinde → o tarafa kaçış güvenli mi diye bakılır.
      - SAĞ    : simetrik (right tarafı).
    Merkezdeki engel yan sektörleri KİRLETMEZ; böylece tam ortadaki bir engelden
    boş bir yana kaçış mümkün kalır.
    """
    d_center = _INF
    d_left = _INF
    d_right = _INF
    d_overall = _INF
    nearest_d = _INF
    nearest_angle = 0.0
    count = 0

    chw = p.corridor_half_w_m

    for pt in points:
        fwd = float(pt[0])
        lat = float(pt[1])
        hw = float(pt[2]) if len(pt) > 2 else 0.0

        if fwd <= p.forward_min_m:
            continue  # arkamızda / yanımızda değil → sürüş kararını etkilemez

        rng = math.hypot(fwd, lat)
        # Engelin araç merkez hattına en yakın yanal kenarı
        lat_edge = max(0.0, abs(lat) - hw)

        # --- Merkez sektör + genel/açı (ileri bakış içinde) ---
        if fwd <= p.forward_max_m:
            count += 1
            if rng < d_overall:
                d_overall = rng
                nearest_d = rng
                # Eski engel_node konvansiyonu: sağ pozitif, sol negatif
                nearest_angle = -math.degrees(math.atan2(lat, fwd))
            if lat_edge <= chw:
                if rng < d_center:
                    d_center = rng

        # --- Yan sektörler (yalnız koridor DIŞI; lane-change boşluğu) ---
        if fwd <= p.side_forward_max_m and lat_edge > chw:
            if lat > 0.0:  # sol şerit
                if rng < d_left:
                    d_left = rng
            elif lat < 0.0:  # sağ şerit
                if rng < d_right:
                    d_right = rng

    present = math.isfinite(d_center)
    return FusedObstacles(
        present=present,
        d_center=d_center,
        d_left=d_left,
        d_right=d_right,
        d_overall=d_overall,
        angle_deg=nearest_angle if math.isfinite(nearest_d) else 0.0,
        count=count,
    )


# ============================================================
# Ackermann yay-kapısı (2026-07-15) — acil bandı için
# ------------------------------------------------------------
# Canlı bulgu (run 20260713T173335Z): 41° sağ-yanda 1.12 m'deki bordür/koni,
# araç SOLA dönerken bile düz-koridor d_center eşiğini kırıp acildurus'u
# 2 dakika kilitledi. Bisiklet modeline göre süpürme bandı o nesneyi
# temizliyordu. d_center dur/reroute/yavasla bantları için doğru (planlama
# erken görmeli); ACİL bandı ise "mevcut direksiyonla GERÇEKTEN çarpacak mı"
# sorusudur → aşağıdaki kapı yalnız acil tetik/release'te kullanılır.
# Matematik control.py ackermann_path_clears ile bire bir aynıdır
# (2026-07-04 /incele CONFIRMED); iki taraf senkron tutulmalı.
# ============================================================

@dataclass
class ArcGateParams:
    enabled: bool = True
    wheelbase_m: float = 1.86       # Bee1/golf.urdf dingil (2026-07-04 hizalaması)
    half_width_m: float = 0.75      # araç yarı gen. 0.6 + koni payı 0.15 (= control ESTOP_BANT_YARIM_M)
    sensor_to_ra_m: float = 1.76    # lidar → arka aks (= control LIDAR_ARKA_AKS_M)
    nose_m: float = 2.34            # arka aks → ön tampon (golf.urdf ölçümü: 1.477+0.862)
    steer_full_deg: float = 28.95   # steer=1'in bisiklet açısı (urdf max_steer 0.5053 rad)
    steer_max_age_s: float = 0.5    # direksiyon verisi bundan eskiyse kapı DEVRE DIŞI (fail-safe)

    @classmethod
    def from_cfg(cls, cfg: dict) -> "ArcGateParams":
        cfg = cfg or {}
        return cls(
            enabled=bool(cfg.get("enabled", True)),
            wheelbase_m=float(cfg.get("wheelbase_m", 1.86)),
            half_width_m=float(cfg.get("half_width_m", 0.75)),
            sensor_to_ra_m=float(cfg.get("sensor_to_ra_m", 1.76)),
            nose_m=float(cfg.get("nose_m", 2.34)),
            steer_full_deg=float(cfg.get("steer_full_deg", 28.95)),
            steer_max_age_s=float(cfg.get("steer_max_age_s", 0.5)),
        )


def ackermann_path_clears(fwd: float, lat: float, steer_deg: float,
                          gp: ArcGateParams) -> bool:
    """Araç MEVCUT direksiyonla giderken 2B süpürme bandı (fwd, lat)'ı
    içine alıyor mu? True = geçer (çarpmaz), False = bant içinde.

    (fwd, lat) SENSÖR (lidar) çerçevesindedir; bisiklet modeli arka aks
    referanslı → ICR sensörün sensor_to_ra_m arkasında. Düz gidişte yanal
    ayrım yeterli; dönüşte gövdenin süpürdüğü halka (iç kenar R−w, dış kenar
    hypot(R+w, burun)) ile engelin ICR uzaklığı karşılaştırılır.
    Kaynak: control.py ackermann_path_clears (aynı formüller)."""
    delta = math.radians(steer_deg)
    if abs(delta) < math.radians(1.0):
        return abs(lat) >= gp.half_width_m
    R = gp.wheelbase_m / math.tan(abs(delta))
    cy = R if delta > 0.0 else -R              # + sol dönüş → ICR +y
    d_c = math.hypot(fwd + gp.sensor_to_ra_m, lat - cy)
    r_ic = R - gp.half_width_m
    r_dis = math.hypot(R + gp.half_width_m, gp.nose_m)
    return d_c <= r_ic or d_c >= r_dis


def arc_blocking_distance(points: Iterable[Sequence[float]],
                          steer_deg: float,
                          p: ObstacleFusionParams,
                          gp: ArcGateParams) -> float:
    """Mevcut direksiyonun süpürme bandı İÇİNDE kalan en yakın engelin
    menzili (hypot; d_center ile aynı metrik). Bant hepsini temizliyorsa inf.

    Yalnız önümüzdeki (forward_min < fwd ≤ forward_max) noktalar denetlenir.
    Geri viteste anlamı yoktur (ileri bant varsayılır) — bugünkü d_center
    davranışından daha gevşek değildir. Engelin yarı genişliği (hw) banda
    eklenir: kenarı banda değen engel de bloklayıcı sayılır."""
    best = _INF
    for pt in points:
        fwd = float(pt[0])
        lat = float(pt[1])
        hw = float(pt[2]) if len(pt) > 2 else 0.0
        if fwd <= p.forward_min_m or fwd > p.forward_max_m:
            continue
        eff = ArcGateParams(
            enabled=gp.enabled, wheelbase_m=gp.wheelbase_m,
            half_width_m=gp.half_width_m + hw,
            sensor_to_ra_m=gp.sensor_to_ra_m, nose_m=gp.nose_m,
            steer_full_deg=gp.steer_full_deg,
            steer_max_age_s=gp.steer_max_age_s,
        ) if hw > 0.0 else gp
        if ackermann_path_clears(fwd, lat, steer_deg, eff):
            continue
        rng = math.hypot(fwd, lat)
        if rng < best:
            best = rng
    return best
