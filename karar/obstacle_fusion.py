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
