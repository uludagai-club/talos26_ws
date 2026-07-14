"""Condition node'ları.

Hepsi yan etkisiz; yalnız blackboard'u okur, SUCCESS/FAILURE döner.
"""
from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Status

from bb import Blackboard
from avoidance_geometry import obstacle_world_pos, side_to_avoid

_INF = float("inf")


# ============================================================
# Yardımcı bazsınıf
# ============================================================
class _Cond(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, bb: Blackboard):
        super().__init__(name=name)
        self.bb = bb


# ============================================================
# Sensor freshness
# ============================================================
def _is_fresh(last_seen: float, max_age_s: float) -> bool:
    if last_seen <= 0.0:
        return False
    return (time.time() - last_seen) <= max_age_s


# ============================================================
# Hıza-duyarlı eşik yardımcıları
# ============================================================
def _speed_mps(bb: Blackboard, odom_max_age_s: float) -> float:
    """Taze odom varsa hız (m/s); yoksa 0.0 (güvenli: eşik tabana iner)."""
    if not _is_fresh(bb.obs.odom_last_seen, odom_max_age_s):
        return 0.0
    return max(0.0, bb.obs.speed_kmh / 3.6)


def _adaptive_esik(base_m: float, bb: Blackboard, gain_s: float,
                   max_extra_m: float, odom_max_age_s: float) -> float:
    """Hıza göre genişleyen eşik. gain_s<=0 → taban eşik (değişiklik yok)."""
    if gain_s <= 0.0:
        return base_m
    extra = min(gain_s * _speed_mps(bb, odom_max_age_s), max_extra_m)
    return base_m + extra


class YayaFresh(_Cond):
    def __init__(self, bb, max_age_s):
        super().__init__("YayaFresh?", bb)
        self.max_age_s = max_age_s

    def update(self):
        return Status.SUCCESS if _is_fresh(self.bb.obs.yaya_last_seen, self.max_age_s) else Status.FAILURE


class LevhaFresh(_Cond):
    def __init__(self, bb, max_age_s):
        super().__init__("LevhaFresh?", bb)
        self.max_age_s = max_age_s

    def update(self):
        return Status.SUCCESS if _is_fresh(self.bb.obs.levha_last_seen, self.max_age_s) else Status.FAILURE


class EngelFresh(_Cond):
    def __init__(self, bb, max_age_s):
        super().__init__("EngelFresh?", bb)
        self.max_age_s = max_age_s

    def update(self):
        return Status.SUCCESS if _is_fresh(self.bb.obs.engel_last_seen, self.max_age_s) else Status.FAILURE


class OdomFresh(_Cond):
    def __init__(self, bb, max_age_s):
        super().__init__("OdomFresh?", bb)
        self.max_age_s = max_age_s

    def update(self):
        return Status.SUCCESS if _is_fresh(self.bb.obs.odom_last_seen, self.max_age_s) else Status.FAILURE


# ============================================================
# Yaya
# ============================================================
class YayaVarMi(_Cond):
    """Yaya geçidi mesajı 'none' değil mi?"""
    def __init__(self, bb):
        super().__init__("YayaVar?", bb)

    def update(self):
        return Status.SUCCESS if self.bb.obs.yaya_present and self.bb.obs.yaya_distance > 0 else Status.FAILURE


class YayaCokYakin(_Cond):
    """Acil durus eşiği."""
    def __init__(self, bb, esik_m):
        super().__init__(f"YayaCokYakin(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.yaya_distance
        return Status.SUCCESS if (d is not None and 0 < d < self.esik_m) else Status.FAILURE


class YayaYakin(_Cond):
    """Tam duruş eşiği (hıza-duyarlı)."""
    def __init__(self, bb, esik_m, gain_s=0.0, max_extra_m=0.0, odom_max_age_s=0.5):
        super().__init__(f"YayaYakin(<{esik_m}m+v)?", bb)
        self.esik_m = esik_m
        self.gain_s = gain_s
        self.max_extra_m = max_extra_m
        self.odom_max_age_s = odom_max_age_s

    def update(self):
        d = self.bb.obs.yaya_distance
        esik = _adaptive_esik(self.esik_m, self.bb, self.gain_s, self.max_extra_m, self.odom_max_age_s)
        return Status.SUCCESS if (d is not None and 0 < d < esik) else Status.FAILURE


class YayaOrtaMesafe(_Cond):
    """Yavaşlama eşiği (hıza-duyarlı)."""
    def __init__(self, bb, esik_m, gain_s=0.0, max_extra_m=0.0, odom_max_age_s=0.5):
        super().__init__(f"YayaOrta(<{esik_m}m+v)?", bb)
        self.esik_m = esik_m
        self.gain_s = gain_s
        self.max_extra_m = max_extra_m
        self.odom_max_age_s = odom_max_age_s

    def update(self):
        d = self.bb.obs.yaya_distance
        esik = _adaptive_esik(self.esik_m, self.bb, self.gain_s, self.max_extra_m, self.odom_max_age_s)
        return Status.SUCCESS if (d is not None and 0 < d < esik) else Status.FAILURE


# ============================================================
# Engel
# ============================================================
class EngelVar(_Cond):
    def __init__(self, bb):
        super().__init__("EngelVar?", bb)

    def update(self):
        return Status.SUCCESS if self.bb.obs.engel_present else Status.FAILURE


class EngelCokYakin(_Cond):
    """Mevcut direksiyonun süpürme bandı İÇİNDE çok yakın engel — acil durus.

    2026-07-15: d_center yerine engel_d_arc okur (yay-kapısı). Bisiklet
    modeline göre yayın DIŞINDA kalan yan nesne (örn. 41°'de bordür, canlı
    173335) artık acildurus tetiklemez; dur/reroute/yavasla bantları
    d_center ile aynen sürer. Direksiyon verisi yoksa ros_bridge d_arc'a
    d_center yazar → davranış eskisiyle aynı (fail-safe)."""
    def __init__(self, bb, esik_m):
        super().__init__(f"EngelCokYakin(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.engel_d_arc
        if d is None or not math.isfinite(d):
            return Status.FAILURE   # inf = bant temiz (veya veri yok) → acil değil
        return Status.SUCCESS if d < self.esik_m else Status.FAILURE


class EngelMerkezBlokaj(_Cond):
    """Merkez sektörde sürüş engelleyici mesafede engel var mı? (hıza-duyarlı)"""
    def __init__(self, bb, esik_m, gain_s=0.0, max_extra_m=0.0, odom_max_age_s=0.5):
        super().__init__(f"EngelMerkezBlokaj(<{esik_m}m+v)?", bb)
        self.esik_m = esik_m
        self.gain_s = gain_s
        self.max_extra_m = max_extra_m
        self.odom_max_age_s = odom_max_age_s

    def update(self):
        d = self.bb.obs.engel_d_center
        if d is None or not math.isfinite(d):
            return Status.FAILURE
        esik = _adaptive_esik(self.esik_m, self.bb, self.gain_s, self.max_extra_m, self.odom_max_age_s)
        return Status.SUCCESS if d < esik else Status.FAILURE


class YanSektorBos(_Cond):
    """Sol veya sağ sektörde belirli mesafeden uzakta engel yok mu?

    Güvenlik: yan sektör sensörü `max_age_s` içinde veri vermediyse FAILURE
    döner — bilinmeyen yöne şerit değiştirmeyiz (eski davranış: veri yokken
    'boş' kabul ediyordu, bu riskliydi).
    """
    def __init__(self, bb, taraf: str, esik_m: float, max_age_s: float):
        assert taraf in ("sol", "sag")
        super().__init__(f"{taraf.upper()}Bos(>{esik_m}m)?", bb)
        self.taraf = taraf
        self.esik_m = esik_m
        self.max_age_s = max_age_s

    def update(self):
        if self.taraf == "sol":
            d = self.bb.obs.engel_d_left
            last_seen = self.bb.obs.engel_left_last_seen
        else:
            d = self.bb.obs.engel_d_right
            last_seen = self.bb.obs.engel_right_last_seen

        # Yan sektör tazeliği yoksa güvenli tarafta kal: kaçış yapma
        if not _is_fresh(last_seen, self.max_age_s):
            return Status.FAILURE

        # inf veya eşikten büyük → boş
        if d is None or not math.isfinite(d):
            return Status.SUCCESS
        return Status.SUCCESS if d >= self.esik_m else Status.FAILURE


# ============================================================
# Yol-bilinçli kaçış yönü seçimi (waypoint tabanlı)
# ============================================================
class KacisYonuSec(_Cond):
    """Engelden kaçış yönünü WAYPOINT'lere göre seçer ve bb.state'e yazar.

    Mantık (kullanıcı isteği: "sol/sağ kararını waypointlere göre ver"):
      1. /hedef taze ve WP'ler varsa → engelin DÜNYA konumunu hesapla, rota
         doğrusuna göre işaretli yanal konumunu çıkar (çapraz-çarpım). Engel
         rotanın SOLUNDAysa SAĞA, SAĞINDAysa SOLA kaç → kaynak="rota".
         Bu seçim aracın kendi şerit ofsetinden BAĞIMSIZDIR (sadece engel↔rota).
      2. Engel ~rota üzerinde (deadband içinde, tipik koni) veya rota verisi
         yoksa → en açık yan sektöre düş (engel_d_left vs d_right) → kaynak="yan_sektor";
         beraberlikte `varsayilan_yon` (ters/karşı şerit = genelde "sol").

    AKTİF SEGMENT SEÇİMİ (control.py WP_NEAR_DISTANCE ± histerezis — Q1):
      Araç wp1'e (wp_near + hyst) kadar yaklaştıysa onu "geçilmiş" sayıp rota
      doğrusunu wp1→wp2 alır; aksi halde robot→wp1 alır. Böylece karar, control'ün
      WP geçiş eşiğiyle aynı "aktif segment" anlayışını paylaşır.

    Her zaman SUCCESS döner (bir yön daima seçilir); o yönün GERÇEKTEN boş olup
    olmadığını YanSektorBosSecilen denetler.
    """

    def __init__(self, bb, deadband_m: float, wp_near_m: float, wp_hyst_m: float,
                 hedef_max_age_s: float, varsayilan_yon: str = "sol"):
        super().__init__("KacisYonuSec", bb)
        self.deadband_m = float(deadband_m)
        self.wp_near_m = float(wp_near_m)
        self.wp_hyst_m = float(wp_hyst_m)
        self.hedef_max_age_s = float(hedef_max_age_s)
        self.varsayilan_yon = varsayilan_yon if varsayilan_yon in ("sol", "sag") else "sol"

    def _yan_sektor_yon(self) -> str:
        o = self.bb.obs
        dl = o.engel_d_left if o.engel_d_left is not None else _INF
        dr = o.engel_d_right if o.engel_d_right is not None else _INF
        if dl > dr:
            return "sol"
        if dr > dl:
            return "sag"
        return self.varsayilan_yon

    def update(self):
        o = self.bb.obs
        s = self.bb.state
        taraf = None
        kaynak = "yan_sektor"
        lateral = 0.0
        ox = oy = 0.0

        hedef_taze = _is_fresh(o.hedef_last_seen, self.hedef_max_age_s) \
            and o.hedef_x is not None and o.hedef_y is not None
        # Engel menzili: nearest overall, yoksa center
        rng = o.engel_d_overall
        if rng is None or not math.isfinite(rng):
            rng = o.engel_d_center

        route_taze = hedef_taze and rng is not None and math.isfinite(rng)
        if route_taze:
            ox, oy = obstacle_world_pos(o.x, o.y, o.yaw, rng, o.engel_angle_deg or 0.0)
            # Aktif segmenti WP_NEAR ± histerezis ile seç
            d_wp1 = math.hypot(o.hedef_x - o.x, o.hedef_y - o.y)
            if (o.next_hedef_x is not None and o.next_hedef_y is not None
                    and d_wp1 <= (self.wp_near_m + self.wp_hyst_m)):
                ax, ay = o.hedef_x, o.hedef_y          # wp1 geçildi → wp1→wp2
                bx, by = o.next_hedef_x, o.next_hedef_y
            else:
                ax, ay = o.x, o.y                       # robot→wp1
                bx, by = o.hedef_x, o.hedef_y
            taraf, lateral = side_to_avoid(ox, oy, ax, ay, bx, by, self.deadband_m)
            if taraf is not None:
                kaynak = "rota"                        # engel rotanın bir yanında → karşı tarafa
            else:
                # Engel ~rota ÜZERİNDE (deadband içinde = tipik blok eden koni). Geometri
                # belirsiz → KARŞI/geçiş şeridine (varsayilan_yon) geç. yan_sektor'e DÜŞME:
                # araca-göre d_left/d_right gürültülü/simetrik (CANLI BUG 2026-06-24:
                # merkezi koni → yan_sektor → yanlış "sag" → araç koniye girip acildurus).
                taraf = self.varsayilan_yon
                kaynak = "rota_merkez"

        if taraf is None:
            # Rota HİÇ yok (tazelik/WP eksik) → son çare en açık yan sektör
            taraf = self._yan_sektor_yon()
            kaynak = "yan_sektor"

        s.kacis_yon = taraf
        s.kacis_kaynak = kaynak
        s.kacis_lateral_m = lateral
        s.kacis_engel_dunya = (ox, oy)
        return Status.SUCCESS


class YanSektorBosSecilen(_Cond):
    """KacisYonuSec'in seçtiği yön (bb.state.kacis_yon) gerçekten boş + taze mi?

    YanSektorBos ile aynı güvenlik kapısı; ama tarafı state'ten dinamik okur
    (sabit "sol"/"sag" değil). Yön seçilmemişse FAILURE.
    """

    def __init__(self, bb, esik_m: float, max_age_s: float):
        super().__init__("YanSektorBosSecilen", bb)
        self.esik_m = float(esik_m)
        self.max_age_s = float(max_age_s)

    def update(self):
        taraf = self.bb.state.kacis_yon
        if taraf == "sol":
            d = self.bb.obs.engel_d_left
            last_seen = self.bb.obs.engel_left_last_seen
        elif taraf == "sag":
            d = self.bb.obs.engel_d_right
            last_seen = self.bb.obs.engel_right_last_seen
        else:
            return Status.FAILURE

        if not _is_fresh(last_seen, self.max_age_s):
            return Status.FAILURE
        if d is None or not math.isfinite(d):
            return Status.SUCCESS
        return Status.SUCCESS if d >= self.esik_m else Status.FAILURE


# ============================================================
# Levha
# ============================================================
class LevhaIs(_Cond):
    """Belirtilen sınıflardan herhangi biri mi?"""
    def __init__(self, bb, hedef_isimler: tuple, max_mesafe_m: float):
        super().__init__(f"Levha in {hedef_isimler} (<{max_mesafe_m}m)?", bb)
        self.hedef_isimler = tuple(h.upper() for h in hedef_isimler)
        self.max_mesafe_m = max_mesafe_m

    def update(self):
        if self.bb.obs.levha_isim not in self.hedef_isimler:
            return Status.FAILURE
        d = self.bb.obs.levha_distance
        if d is None or d <= 0:
            return Status.FAILURE
        return Status.SUCCESS if d <= self.max_mesafe_m else Status.FAILURE


class LevhaIcindeMesafe(_Cond):
    """Belirli sınıf hem doğru hem eşik içinde mi?"""
    def __init__(self, bb, isim: str, esik_m: float):
        super().__init__(f"Levha=={isim} & d<{esik_m}m?", bb)
        self.isim = isim.upper()
        self.esik_m = esik_m

    def update(self):
        if self.bb.obs.levha_isim != self.isim:
            return Status.FAILURE
        d = self.bb.obs.levha_distance
        return Status.SUCCESS if (d is not None and 0 < d < self.esik_m) else Status.FAILURE


# ============================================================
# Emergency latch (state okuma)
# ============================================================
class EmergencyLatched(_Cond):
    def __init__(self, bb):
        super().__init__("EmergencyLatched?", bb)

    def update(self):
        return Status.SUCCESS if self.bb.state.emergency_latched else Status.FAILURE


# ============================================================
# Lane-change cooldown
# ============================================================
class LaneChangeCooldownOk(_Cond):
    def __init__(self, bb, cooldown_s: float):
        super().__init__(f"LaneChangeCooldown(>{cooldown_s}s)?", bb)
        self.cooldown_s = cooldown_s

    def update(self):
        last = self.bb.state.last_lane_change_s
        if last <= 0.0:
            return Status.SUCCESS
        return Status.SUCCESS if (time.time() - last) >= self.cooldown_s else Status.FAILURE


class LaneChangeInProgress(_Cond):
    """Bir şerit değişimi başlatıldı ve manevra penceresi (hold_s) henüz dolmadı mı?

    control.py şerit değişimini kenar-tetiklemeli başlatıp LANE_CHANGE_DURATION
    süresince kendi sürer. Bu pencerede BT aynı yön komutunu tutmalı — yoksa
    "dur" (fren) veya "normal" (manevrayı iptal) komutu manevrayı keser.
    """
    def __init__(self, bb, hold_s: float):
        super().__init__(f"LaneChangeInProgress(<{hold_s}s)?", bb)
        self.hold_s = hold_s

    def update(self):
        s = self.bb.state
        if not s.lane_change_dir:
            return Status.FAILURE
        if s.last_lane_change_s <= 0.0:
            return Status.FAILURE
        return Status.SUCCESS if (time.time() - s.last_lane_change_s) < self.hold_s else Status.FAILURE
