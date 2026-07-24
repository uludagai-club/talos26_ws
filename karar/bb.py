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
    # --- Yaya (adanmış çizgi modeli /yaya_gecidi/model) ---
    yaya_present: bool = False
    yaya_x: float = _NA
    yaya_y: float = _NA
    yaya_distance: float = _NA
    yaya_last_seen: float = 0.0

    # --- Yaya geçidi LEVHASI (levha modelinin yaya_gecidi sınıfı → /yaya_gecidi;
    #     adanmış çizgi modeli /yaya_gecidi/model'DEN AYRI). Çizgi modeli şeritle
    #     karışabildiği için karar ona yalnız bu levha görülünce güvenir (kapı). ---
    yaya_levha_present: bool = False
    yaya_levha_distance: float = _NA   # ileri mesafe (m)
    yaya_levha_last_seen: float = 0.0      # ROS time saniye

    # --- Park alanı modeli (/park_alani; park_durak_node HSV mavi-renk) ---
    #     "mesafe,offset" veya "none". mesafe bbox-proxy (kaba; yalnız "en yakın"
    #     sıralaması için — mutlak güvenilmez), offset işaretli (sol- / sağ+).
    #     Park müsaitlik AND'inin Kapı 2'si (model park alanı gösteriyor mu). ---
    park_alani_present: bool = False
    park_alani_distance: float = _NA   # ileri mesafe (m; bbox-proxy)
    park_alani_offset: float = _NA     # işaretli yatay offset (sol- / sağ+)
    park_alani_last_seen: float = 0.0

    # --- Trafik levhası ---
    levha_isim: str = "NONE"          # "DUR","SAG","SOL","30","OKUL","YAVAS","KIRMIZI","PARK_YERI","PARK_ETMEK_YASAKTIR","NONE"
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
    engel_source: str = "none"           # "poses" (yeni detektör) | "poses+mem" | "legacy" | "none" — debug
    engel_count: int = 0                 # ileri bakış içindeki engel sayısı — debug
    engel_mem_count: int = 0             # bu tick hafızadan enjekte edilen duba sayısı (dropout köprüsü) — debug
    # Yay-kapısı (2026-07-15): mevcut direksiyonun süpürme bandı İÇİNDEKİ en
    # yakın engel menzili. ACİL tetik/release BUNU okur (d_center değil) —
    # yan nesne bisiklet yayının dışındaysa acildurus atılmaz. Direksiyon
    # verisi yok/bayatsa ros_bridge buraya d_center yazar (fail-safe: eski davranış).
    engel_d_arc: float = _INF

    # --- Direksiyon (yay-kapısı girdisi; /cart cart_control.steer × steer_full_deg) ---
    steer_deg: float = 0.0               # bisiklet-modeli komut açısı; + sol
    steer_last_seen: float = 0.0

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
    emergency_clear_streak: int = 0          # her tür temizlik (yokluk dahil) ardışık tick
    emergency_clear_streak_olculu: int = 0   # yalnız ÖLÇÜLÜ temizlik (d_arc≥eşik) ardışık tick (P1 №7)
    # Statik-durum çözme yolu (P0 №3, inceleme 2026-07-16 E8-R1):
    # LatchEmergency kurar, ReleaseEmergencyIfClear okur/günceller.
    emergency_latch_start_s: float = 0.0      # mührün kurulduğu an (time.time)
    emergency_d_arc_ref: float = _INF         # d_arc sabitlik takibi referansı
    emergency_d_arc_stable_ticks: int = 0     # d_arc referans ± tolerans içinde kalınan ardışık tick

    # DUR levhası FSM: "idle" | "holding" | "released"
    stop_sign_phase: str = "idle"
    stop_sign_hold_start_s: float = 0.0
    stop_sign_released_s: float = 0.0   # son release zamanı — release_grace_s ile çift duruşu önler

    # Trafik ışığı FSM (birleşik KIRMIZI/SARI/YEŞİL — TrafikIsigiFSM). Son aksiyon-
    # alan ışık (KIRMIZI→dur / YAVAS→slow) grace ile tutulur → flicker'ı yutar;
    # YEŞİL/ışık yok → geç. hazir: kırmızıdan sonra sarı = yeşile yavaştan hazırlan.
    trafik_isik_last_light: str = ""      # "" | "KIRMIZI" | "YAVAS" (aksiyon-alan son ışık)
    trafik_isik_last_s: float = 0.0       # o ışığın en son görüldüğü an (release grace)
    trafik_isik_hazir: bool = False       # kırmızı→sarı geçişi (yeşile hazırlan; her zaman slow)

    # Yaya geçidi FSM: "idle" | "holding" | "released" (min zorunlu duruş + lidar
    # engel ile yaya-bekleme köprüsü; adanmış model yalnız 'crosswalk' verdiği için).
    yaya_gecidi_phase: str = "idle"
    yaya_gecidi_hold_start_s: float = 0.0
    yaya_gecidi_released_s: float = 0.0   # son release — release_grace_s ile çift duruşu önler

    # Yaya geçidi LEVHA-KAPISI (2026-07-23): çizgi modeline yalnız yaya geçidi
    # LEVHASI görülünce güven (çizgi modeli şeritle karışabiliyor). Kapı levha
    # menzile girince açılır; çizgi FSM release olunca VEYA levha geçilince
    # (dünya-çapası pass_behind gerisinde) kapanır. YayaLevhaKapisi yürütür.
    yaya_kapi_armed: bool = False
    yaya_kapi_anchor: tuple = (0.0, 0.0)   # levha görüldüğü an dünya konumu (geçildi tespiti)
    yaya_kapi_anchored: bool = False       # çapa taze odom ile kuruldu mu (geçildi kapısı için)
    yaya_kapi_arm_s: float = 0.0           # silahlanma anı (fail-safe max TTL)
    yaya_kapi_released_s: float = 0.0      # son kapanma — grace ile hemen yeniden silahlanmayı önler

    # Park müsaitlik FSM (ParkFSM) — üç-kapılı AND, "park tabelası → model → lidar":
    #   Kapı 1  PARK_YERI levhası görüldü mü (kapı arm; PARK_ETMEK_YASAKTIR → yasak)
    #   Kapı 2  /park_alani modeli alan gösteriyor mu (present+taze)
    #   Kapı 3  o alanda lidar engeli yok mu  (2026-07-24: ERTELENDİ — bag analizinden
    #           sonra eklenecek; park.lidar_enabled=false iken True kabul edilir)
    # Kapı, yaya geçidi levha-kapısı (YayaLevhaKapisi) desenini yansıtır: levha
    # görülmeden /park_alani modeli DİNLENMEZ. "idle"|"armed"|"released".
    park_phase: str = "idle"
    park_kapi_arm_s: float = 0.0            # silahlanma anı (fail-safe TTL)
    park_kapi_anchor: tuple = (0.0, 0.0)   # PARK_YERI levhası görüldüğü an dünya konumu (geçildi tespiti)
    park_kapi_anchored: bool = False       # çapa taze odom ile kuruldu mu
    park_kapi_released_s: float = 0.0      # son kapanma — grace ile yeniden silahlanmayı önler

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

    # Engel DUR→REROUTE→DEVAM fazı (RerouteKarar yürütür): engel yolu bloklayınca
    # önce sınırlı süre gerçek DUR (planlayıcı replan yapsın), sonra reroute'u
    # takip için SLOW'a geç. "" = boş/yeni karşılaşma | "stop" = duraklama sürüyor
    # | "follow" = reroute takibi (slow). Branch birkaç tick sessiz kalınca (engel
    # banttan çıktı) reset_gap ile "" ye döner → sonraki engelde yeniden durulur.
    reroute_phase: str = ""
    reroute_stop_start_s: float = 0.0       # "stop" fazının başlangıcı (bekleme ölçümü)
    reroute_last_tick_s: float = 0.0        # RerouteKarar'ın en son çalıştığı tick (dormant reset)

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

    def read_pose(self) -> tuple:
        """(x, y, yaw, odom_last_seen) — lock altında TUTARLI okuma.

        _on_odom (arka iplik) bu dört alanı birlikte write() ile yazar; başka bir
        callback (ör. ros_bridge memory köprüsü) lock'suz üç ayrı okursa yarı-yazılmış
        poz (x_eski, yaw_yeni) yakalayabilir → yanlış dünya konumu (/incele ROS+güvenlik).
        """
        with self._lock:
            o = self.obs
            return (o.x, o.y, o.yaw, o.odom_last_seen)

    def snapshot(self) -> dict:
        """Debug/logging için anlık özet (JSON'a çevrilebilir)."""
        o = self.obs
        s = self.state
        return {
            "yaya": {"present": o.yaya_present, "d": o.yaya_distance, "age_s": _age(o.yaya_last_seen)},
            "levha": {"isim": o.levha_isim, "d": o.levha_distance, "age_s": _age(o.levha_last_seen)},
            "park": {"model": o.park_alani_present, "d": _fin(o.park_alani_distance),
                     "off": o.park_alani_offset, "age_s": _age(o.park_alani_last_seen)},
            "engel": {
                "present": o.engel_present,
                "d_arc":    _fin(o.engel_d_arc),     # ACİL tetiği bunu okur (yay-kapısı)
                "d_center": _fin(o.engel_d_center),
                "d_left":   _fin(o.engel_d_left),
                "d_right":  _fin(o.engel_d_right),
                "angle_deg": o.engel_angle_deg,
                "age_s": _age(o.engel_last_seen),
                "source": o.engel_source,
                "count": o.engel_count,
                "mem": o.engel_mem_count,
            },
            "speed_kmh": o.speed_kmh,
            "state": {
                "emergency_latched": s.emergency_latched,
                "stop_sign_phase": s.stop_sign_phase,
                "trafik_isik": s.trafik_isik_last_light or "-",
                "trafik_isik_hazir": s.trafik_isik_hazir,
                "yaya_gecidi_phase": s.yaya_gecidi_phase,
                "yaya_kapi_armed": s.yaya_kapi_armed,
                "park_phase": s.park_phase,
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
