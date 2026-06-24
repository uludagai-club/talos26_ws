"""Blackboard — tüm BT'nin paylaştığı tek gözlem tablosu.

ROSBridge yalnız buraya yazar; behavior'lar yalnız buradan okur.
Bu ayrım test edilebilirliği sağlıyor: ROS olmadan da blackboard'u
elle doldurup ağacı tick'lemek mümkün.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import Optional


_INF = float("inf")
_NA = -1.0


@dataclass
class Observations:
    # --- Yaya ---
    yaya_present: bool = False
    yaya_x: float = _NA
    yaya_y: float = _NA
    yaya_distance: float = _NA
    yaya_last_seen: float = 0.0      # ROS time saniye

    # --- Trafik levhası ---
    levha_isim: str = "NONE"          # "DUR","SAG","SOL","30","OKUL","YAVAS","KIRMIZI","NONE"
    levha_distance: float = _NA
    levha_x: float = _NA              # ileri (m)
    levha_y: float = _NA              # yan (m)
    levha_last_seen: float = 0.0

    # --- Engel ---
    engel_present: bool = False
    engel_d_center: float = _INF
    engel_d_left:   float = _INF
    engel_d_right:  float = _INF
    engel_d_overall: float = _INF
    engel_angle_deg: float = 0.0
    engel_last_seen: float = 0.0
    engel_left_last_seen:  float = 0.0   # yan sektör ayrı tazelik (lane-change güvenliği)
    engel_right_last_seen: float = 0.0
    engel_source: str = "none"           # "poses" (yeni detektör) | "legacy" | "none" — debug
    engel_count: int = 0                 # ileri bakış içindeki engel sayısı — debug

    # --- Şerit ---
    lane_offset_px: float = 0.0
    line_angle_deg: float = 0.0
    lane_last_seen: float = 0.0

    # --- Localization (yalnız okuma; mission planlamak için değil) ---
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed_kmh: float = 0.0
    odom_last_seen: float = 0.0

    # --- Mission gözlemi (read-only) ---
    hedef_x: Optional[float] = None
    hedef_y: Optional[float] = None
    next_hedef_x: Optional[float] = None
    next_hedef_y: Optional[float] = None
    hedef_last_seen: float = 0.0

    # --- Geçen tick'in ham String input'ları (Decision msg için aynısı dolacak) ---
    raw_yaya: str = "none"
    raw_levha: str = "none"


@dataclass
class StatePersist:
    """Tick'ler arası yaşayan iç durum. Behavior'lar bunu güncelliyor."""
    last_karar: Optional[str] = None
    last_decision_id: Optional[str] = None

    # Emergency latch
    emergency_latched: bool = False
    emergency_clear_streak: int = 0

    # DUR levhası FSM: "idle" | "holding" | "released"
    stop_sign_phase: str = "idle"
    stop_sign_hold_start_s: float = 0.0
    stop_sign_released_s: float = 0.0   # son release zamanı — release_grace_s ile çift duruşu önler

    # Lane change cooldown + manevra kilidi (control.py edge-tetiklemeli, manevrayı
    # kendi LANE_CHANGE_DURATION süresince sürdürür → BT aynı yönü o pencere boyunca
    # tutmalı; aksi halde "dur"/"normal" manevrayı keser).
    last_lane_change_s: float = 0.0
    lane_change_dir: str = ""          # "sol" | "sag" | "" — devam eden manevranın yönü

    # Yol-bilinçli kaçış (KacisYonuSec yazar; KacisKarar + logger okur)
    kacis_yon: str = ""               # "sol" | "sag" | "" — bu tick seçilen kaçış yönü
    kacis_kaynak: str = ""            # "rota" (çapraz-çarpım) | "yan_sektor" (en açık) | ""
    kacis_lateral_m: float = 0.0      # engelin rotaya işaretli yanal uzaklığı (sol+)
    kacis_engel_dunya: tuple = (0.0, 0.0)  # son hesaplanan engel dünya konumu (debug)

    # Cone reroute (RerouteKarar yazar; node RerouteManager ile kenar_blok yönetir) — §16/E-A,E-B
    reroute_request: bool = False           # bu tick bloklu cone var mı (reroute talebi); node tüketince sıfırlar
    reroute_cone_world: tuple = (0.0, 0.0)  # bloklu cone'un dünya konumu (/hedef_komut kenar_blok için)

    # Sollama/reroute aynası (yalnız snapshot/log için)
    overtake_active: bool = False
    overtake_return_dist_m: float = 0.0

    # Debounce sayaçları (key -> ardışık true tick sayısı)
    debounce: dict = field(default_factory=dict)


class Blackboard:
    """Thread-safe gözlem tablosu.

    ROS callback'leri arka iplikten yazar; tick döngüsü ana iplikten okur.
    """

    def __init__(self):
        self.obs = Observations()
        self.state = StatePersist()
        self._lock = Lock()
        # Ağacın son tick'inde üretilen karar (publisher tarafı için)
        self.last_decision: dict = {
            "karar": "normal",
            "reason": "init",
            "phase": "driving",
            "wait_remaining_s": 0.0,
        }

    def write(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self.obs, k, v)

    def snapshot(self) -> dict:
        """Debug/logging için anlık özet (JSON'a çevrilebilir)."""
        o = self.obs
        s = self.state
        return {
            "yaya": {"present": o.yaya_present, "d": o.yaya_distance, "age_s": _age(o.yaya_last_seen)},
            "levha": {"isim": o.levha_isim, "d": o.levha_distance, "age_s": _age(o.levha_last_seen)},
            "engel": {
                "present": o.engel_present,
                "d_center": _fin(o.engel_d_center),
                "d_left":   _fin(o.engel_d_left),
                "d_right":  _fin(o.engel_d_right),
                "angle_deg": o.engel_angle_deg,
                "age_s": _age(o.engel_last_seen),
                "source": o.engel_source,
                "count": o.engel_count,
            },
            "speed_kmh": o.speed_kmh,
            "state": {
                "emergency_latched": s.emergency_latched,
                "stop_sign_phase": s.stop_sign_phase,
            },
            "decision": self.last_decision,
        }


# --- Yardımcılar ---
def _age(last_seen: float) -> float:
    if last_seen <= 0.0:
        return _INF
    import time
    return max(0.0, time.time() - last_seen)


def _fin(v: float) -> float:
    return v if v != _INF else -1.0
