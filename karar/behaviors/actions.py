"""Action node'ları.

Hepsi blackboard'a/iç duruma yazar; gerçek ROS publish'i tick döngüsü sonunda
RosBridge yapar (ağacın son tick'inde üretilen son karar yayınlanır).

Karar üretimi: bir action tick'te `bb.last_decision`'u günceller ve SUCCESS döner.
Üst selector (memory'siz) bu SUCCESS'i yakalayıp ağacı keser.
"""
from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Status

from bb import Blackboard
from avoidance_geometry import obstacle_world_pos


# ============================================================
# Karar yayını (ağacın yaprak action'ı)
# ============================================================
class SetKarar(py_trees.behaviour.Behaviour):
    """Verilen kararı blackboard.last_decision'a yazar ve SUCCESS döner."""

    def __init__(self, name: str, bb: Blackboard, karar: str, reason: str,
                 phase: str = "driving", wait_remaining_s: float = 0.0):
        super().__init__(name=name)
        self.bb = bb
        self.karar = karar
        self.reason = reason
        self.phase = phase
        self.wait_remaining_s = wait_remaining_s

    def update(self):
        self.bb.last_decision = {
            "karar": self.karar,
            "reason": self.reason,
            "phase": self.phase,
            "wait_remaining_s": float(self.wait_remaining_s),
        }
        return Status.SUCCESS


# ============================================================
# Emergency latch yönetimi
# ============================================================
class LatchEmergency(py_trees.behaviour.Behaviour):
    """Tetiklenince mührü kilitler ve SUCCESS döner.

    Bu action'a yalnız bir alttaki koşulların biri SUCCESS olduğunda gelinir
    (sequence içinde). Mührü açma işini ReleaseEmergencyIfClear yapar.
    """

    def __init__(self, bb: Blackboard, reason: str = "trigger"):
        super().__init__("LatchEmergency")
        self.bb = bb
        self.reason = reason

    def update(self):
        # Bu node yalnız mühür KAPALIYKEN tick'lenir (mühür açıkken üstteki
        # ReleaseEmergencyIfClear SUCCESS döner ve selector keser) → burası
        # daima "yeni mühür" anıdır: statik-çözme takibi sıfırdan başlar (P0 №3).
        self.bb.state.emergency_latch_start_s = time.time()
        self.bb.state.emergency_d_arc_ref = float("inf")
        self.bb.state.emergency_d_arc_stable_ticks = 0
        self.bb.state.emergency_latched = True
        self.bb.state.emergency_clear_streak = 0
        self.bb.state.emergency_clear_streak_olculu = 0
        self.bb.last_decision = {
            "karar": "acildurus",
            "reason": f"emergency_latch:{self.reason}",
            "phase": "emergency",
            "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS


class ReleaseEmergencyIfClear(py_trees.behaviour.Behaviour):
    """Mühür kapalıyken NoOp. Açıkken: tüm tehlikeler temiz mi diye bakar;
    N tick üst üste temizse mührü çözer.

    Statik-durum çözme yolu (P0 №3, inceleme 2026-07-16 E8-R1, param-kapılı):
    Bırakma eşiği (d_arc ≥ engel_esik) STATİK yakın engelde yapısal olarak
    sağlanamaz — acil'de control steer=0 yollar, d_arc /cart steer'inden
    hesaplandığı için düz-koridor mesafesine donar ve araç kımıldayamadığı
    için mesafe hiç büyümez (9 koşunun 5'i kalıcı kilit, sürenin ~%50'si).
    Mühür uzun süredir kapalı + araç hareketsiz + d_arc stabil + güvenli
    tabanın ÜSTÜNDEyse: mühür AÇIK KALIR (tam çözülme yok → EngelCokYakin+
    Debounce'un anında yeniden mühürleme döngüsü kurulmaz) ama karar
    'acildurus' yerine 'dur'a İNER (reason: muhur_statik_dur) → control'ün
    reason-filtreli DUR-kaçışı (P0 №1) aracı geri çekebilir; kalıcı çözülme
    araç uzaklaşınca normal d_arc yolundan gelir."""

    def __init__(self, bb: Blackboard, release_clear_ticks: int, yaya_esik: float, engel_esik: float,
                 statik_cozme: dict = None, odom_max_age_s: float = 0.5,
                 release_yokluk_ticks: int = None):
        super().__init__("ReleaseEmergencyIfClear")
        self.bb = bb
        self.release_clear_ticks = int(release_clear_ticks)
        # P1 №7 (E5-O3): "temiz"in kaynağı ayrık — yokluk-temizliği (engel_present=0;
        # detektör dropout'u da olabilir) için daha uzun eşik. None → eski davranış.
        self.release_yokluk_ticks = int(release_yokluk_ticks
                                        if release_yokluk_ticks is not None
                                        else release_clear_ticks)
        self.yaya_esik = yaya_esik
        self.engel_esik = engel_esik
        sc = statik_cozme or {}
        self.statik_enabled = bool(sc.get("enabled", False))
        self.statik_min_muhur_s = float(sc.get("min_muhur_s", 15.0))
        self.statik_max_speed_kmh = float(sc.get("max_speed_kmh", 0.3))
        self.statik_d_arc_ticks = int(sc.get("d_arc_sabit_ticks", 30))
        self.statik_d_arc_tol_m = float(sc.get("d_arc_sabit_tol_m", 0.4))
        self.statik_d_arc_min_m = float(sc.get("d_arc_min_m", 1.0))
        self.odom_max_age_s = float(odom_max_age_s)

    def _statik_dur_kosullari(self) -> bool:
        """Statik-durum inişinin tüm kapıları (mühür açıkken her tick çağrılır).
        d_arc sabitlik sayacını da burada ilerletir/sıfırlar."""
        o = self.bb.obs
        s = self.bb.state
        d = o.engel_d_arc
        # Sabitlik takibi: referans ± tolerans içinde kal → say; kır → referansı tazele.
        # Tolerans, duran araçta gözlenen detektör jitter'ına göre geniş (±0.2-0.4 m);
        # gerçek hareketli nesne (yaya ~1 m/s) 3 s pencerede bandı kesin kırar.
        # DROPOUT TOLERANSI (canlı doğrulama 2026-07-17, ros 263-357 mühürü):
        # tek-tick algı dropout'u (d_arc=inf, E3 flicker'ı ~1-2 Hz) sayacı
        # SIFIRLAMAZ — dropout "sahne değişti" kanıtı değildir; sıfırlasaydı
        # 30 ardışık tick hiç birikmez, statik yol hiç açılmazdı. Engelin
        # gerçekten temizlenmesi normal release yolunun işi (8 temiz tick).
        if math.isfinite(d):
            if (math.isfinite(s.emergency_d_arc_ref)
                    and abs(d - s.emergency_d_arc_ref) <= self.statik_d_arc_tol_m):
                s.emergency_d_arc_stable_ticks += 1
            else:
                s.emergency_d_arc_ref = d
                s.emergency_d_arc_stable_ticks = 0
        if not self.statik_enabled:
            return False
        now = time.time()
        odom_taze = (now - o.odom_last_seen) <= self.odom_max_age_s
        return (now - s.emergency_latch_start_s >= self.statik_min_muhur_s
                and odom_taze
                and abs(o.speed_kmh) <= self.statik_max_speed_kmh
                and s.emergency_d_arc_stable_ticks >= self.statik_d_arc_ticks
                and math.isfinite(d)
                and d >= self.statik_d_arc_min_m)

    def update(self):
        if not self.bb.state.emergency_latched:
            return Status.FAILURE  # mühür yok → bu dal devam etmesin

        o = self.bb.obs
        yaya_clear = (not o.yaya_present) or (o.yaya_distance < 0) or (o.yaya_distance >= self.yaya_esik)
        # Engel present ama d_center inf ise sensör verisi eksik → güvenli tarafta kal.
        # Temizlik ölçüsü d_arc (yay-kapısı, 2026-07-15): araç direksiyonu engeli
        # temizleyen bir yaya çevirdiyse (d_arc=inf ya da ≥ eşik) mühür çözülür;
        # ölü-merkez engel her direksiyonda bant içinde kalır → mühür durur.
        engel_d_valid = math.isfinite(o.engel_d_center)
        # P1 №7 (E5-O3): iki AYRI temizlik kaynağı, iki ayrı sayaç.
        #   ölçülü  = engel VAR ama d_arc ≥ eşik (gerçek geometrik kanıt) → 8 tick
        #   yokluk  = engel_present=0 (temiz de olabilir, dropout da!)     → 20 tick
        # 20260716 vakası: 6.6-7 s'lik detektör dropout'u mührü 8 tick'te YANLIŞ
        # anda çözdü → araç koniye ilerledi → yeniden mühür (E5 flip döngüsü).
        engel_olculu_clear = o.engel_present and engel_d_valid and o.engel_d_arc >= self.engel_esik
        engel_yokluk = not o.engel_present

        if yaya_clear and (engel_olculu_clear or engel_yokluk):
            self.bb.state.emergency_clear_streak += 1
        else:
            self.bb.state.emergency_clear_streak = 0
        if yaya_clear and engel_olculu_clear:
            self.bb.state.emergency_clear_streak_olculu += 1
        else:
            self.bb.state.emergency_clear_streak_olculu = 0

        if (self.bb.state.emergency_clear_streak_olculu >= self.release_clear_ticks
                or self.bb.state.emergency_clear_streak >= self.release_yokluk_ticks):
            self.bb.state.emergency_latched = False
            self.bb.state.emergency_clear_streak = 0
            self.bb.state.emergency_clear_streak_olculu = 0
            self.bb.state.emergency_d_arc_ref = float("inf")
            self.bb.state.emergency_d_arc_stable_ticks = 0
            # Mühür çözüldü ama bu tick hâlâ "acildurus" değil — alt dallar konuşsun.
            return Status.FAILURE  # üst selector bir sonraki dalı denesin

        # P0 №3: statik-durum inişi — mühür açık kalır, karar 'dur'a iner.
        # Yaya temiz DEĞİLSE inilmez (yaya bağlamında tam fren korunur).
        if yaya_clear and self._statik_dur_kosullari():
            self.bb.last_decision = {
                "karar": "dur",
                "reason": "muhur_statik_dur",
                "phase": "emergency",
                "wait_remaining_s": 0.0,
            }
            return Status.SUCCESS

        # Mühür hâlâ kapalı → acildurus yay
        self.bb.last_decision = {
            "karar": "acildurus",
            "reason": "emergency_latched",
            "phase": "emergency",
            "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS


# ============================================================
# DUR levhası FSM
# ============================================================
class DurLevhasiFSM(py_trees.behaviour.Behaviour):
    """3 fazlı DUR levhası mantığı.

    - APPROACH (mesafe > stop_esik): "slow"
    - HOLD (mesafe < stop_esik, bekleme süresi dolmadı): "dur"
    - RELEASED: SUCCESS dön ki üst selector ileri gitsin; levha görüşten çıkana
      kadar yeniden tetiklenmesin (FSM 'released' kalır).

    Yeniden silahlanma: levha "NONE" olur veya mesafe >> esik → 'idle'.

    release_grace_s: Bekleme bittikten (release) sonra bu süre boyunca aynı DUR
    levhasının (algı titremesi ya da araç hâlâ levhaya yakınken yeniden görünmesi)
    yeniden tetiklenip İKİNCİ bir duruşa yol açması engellenir.
    """

    def __init__(self, bb: Blackboard, stop_esik_m: float, oku_esik_m: float,
                 bekleme_s: float, release_grace_s: float):
        super().__init__("DurLevhasiFSM")
        self.bb = bb
        self.stop_esik_m = stop_esik_m
        self.oku_esik_m = oku_esik_m
        self.bekleme_s = bekleme_s
        self.release_grace_s = release_grace_s

    def update(self):
        o = self.bb.obs
        s = self.bb.state

        # Yeniden silahlanma: levha görünmüyor veya çok uzaksa idle'a dön
        levha_uzakta = (o.levha_isim != "DUR") or (o.levha_distance < 0) or (o.levha_distance > self.oku_esik_m + 2.0)
        if levha_uzakta and s.stop_sign_phase != "idle":
            s.stop_sign_phase = "idle"
            return Status.FAILURE  # bu tick'te bir karar üretme; üst selector default cruise'a düşsün

        if o.levha_isim != "DUR":
            return Status.FAILURE

        d = o.levha_distance

        # APPROACH
        if s.stop_sign_phase == "idle":
            # Release grace: yeni durulan levhanın çift tetiklenmesini önle
            if (self.release_grace_s > 0.0 and s.stop_sign_released_s > 0.0
                    and (time.time() - s.stop_sign_released_s) < self.release_grace_s):
                return Status.FAILURE
            if d >= self.stop_esik_m and d <= self.oku_esik_m:
                self.bb.last_decision = {
                    "karar": "slow",
                    "reason": "dur_levhasi_yaklasma",
                    "phase": "approach",
                    "wait_remaining_s": 0.0,
                }
                return Status.SUCCESS
            elif d < self.stop_esik_m:
                # Doğrudan HOLD'a geç
                s.stop_sign_phase = "holding"
                s.stop_sign_hold_start_s = time.time()
            else:
                return Status.FAILURE

        # HOLD
        if s.stop_sign_phase == "holding":
            gecen = time.time() - s.stop_sign_hold_start_s
            kalan = max(0.0, self.bekleme_s - gecen)
            if gecen < self.bekleme_s:
                self.bb.last_decision = {
                    "karar": "dur",
                    "reason": "dur_levhasi_bekleme",
                    "phase": "waiting_at_stop",
                    "wait_remaining_s": kalan,
                }
                return Status.SUCCESS
            else:
                s.stop_sign_phase = "released"
                s.stop_sign_released_s = time.time()

        # RELEASED: bu tick'te SUCCESS dönmeyelim ki üst selector
        # cruise'a düşsün; levha görüşten çıkınca 'idle'a sıfırlanır.
        return Status.FAILURE


# ============================================================
# Şerit değiştirme bildirimi (cooldown güncelle)
# ============================================================
class LaneChangeStamp(py_trees.behaviour.Behaviour):
    """Lane change tetiklendi — cooldown sayacını başlat, yönü kilitle, SUCCESS dön.

    `direction` ("sol"/"sag") manevra penceresi boyunca LaneChangeHold dalı
    tarafından yeniden yayınlanır (control.py manevrayı kesmesin diye).
    """

    def __init__(self, bb: Blackboard, direction: str):
        super().__init__(f"LaneChangeStamp({direction})")
        assert direction in ("sol", "sag")
        self.bb = bb
        self.direction = direction

    def update(self):
        self.bb.state.last_lane_change_s = time.time()
        self.bb.state.lane_change_dir = self.direction
        return Status.SUCCESS


class KacisKarar(py_trees.behaviour.Behaviour):
    """Yol-bilinçli kaçış kararı: bb.state.kacis_yon yönünde "sol"/"sag" üretir
    ve cooldown/yön kilidini damgalar (LaneChangeStamp işini de yapar).

    Statik [avoid_left, avoid_right] (sol-öncelikli) sırasının yerine geçer:
    yön artık KacisYonuSec tarafından waypoint'lere göre seçilmiştir. Reason'a
    seçim kaynağı (rota/yan_sektor) gömülür → karar logundan izlenebilir.
    """

    def __init__(self, bb: Blackboard):
        super().__init__("KacisKarar")
        self.bb = bb

    def update(self):
        d = self.bb.state.kacis_yon
        if d not in ("sol", "sag"):
            return Status.FAILURE
        self.bb.state.last_lane_change_s = time.time()
        self.bb.state.lane_change_dir = d
        kaynak = self.bb.state.kacis_kaynak or "?"
        self.bb.last_decision = {
            "karar": d,
            "reason": f"engel_kacis_{d}({kaynak})",
            "phase": "driving",
            "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS


class RerouteKarar(py_trees.behaviour.Behaviour):
    """Engel (cone/lidar) blokajı kararı: DUR → yeniden planla → devam.

    Kullanıcı gereksinimi: "önümüze engel girdisi gelince DURMAK, ardından
    yeniden rota planlayıp DEVAM etmek." Bunu §16 mimarisiyle (control offset
    YOK — H-A; planlayıcı rotayı dubanın etrafından çizer) kilitlenmeden kurar:

      1) STOP fazı — engel commit bandına ilk girdiğinde `pause_s` boyunca
         GERÇEK 'dur' basar ve her tick reroute talebini (kenar_blok) yeniler.
         Böylece araç durur, planlayıcı (hedef) yeni rotayı bu duraklamada çizer.
      2) FOLLOW fazı — bekleme dolunca 'slow'a geçer (talep sürüyor). control
         yeni rotayı düşük hızda takip eder → dubanın etrafından kıvrılır →
         engel banttan çıkınca ağaç bu dala uğramaz → default 'normal'.

    KİLİTLENME NOTU: sürekli 'dur' verilseydi control hareket etmez, rerouteu
    TAKİP edemez, engel merkezde kalır, sonsuza 'dur' olurduk. Bu yüzden 'dur'
    SINIRLIDIR; sonrası 'slow' ile reroute takibi. En yakında (d_arc<acil)
    acildurus üst dalda mutlak güvenlik ağıdır (reroute gerçekten başarısızsa).

    Dormant reset: branch `reset_gap_s` boyunca çalışmazsa (engel banttan çıktı)
    faz "" ye döner → SONRAKİ engelde yeniden tam bir DUR yapılır.

    TEK-SEFERLİK DUR (2026-07-22 karar-kararsızlığı fix): reset_gap KISA olunca
    (eski 0.5s) tünel duvarı / duba detektör titremesi her >0.5s boşlukta fazı
    sıfırlıyor → araç DURDUKTAN sonra bile her titreşimde YENİDEN 1.5s 'dur'
    basılıyordu (canlı 160358Z: 382 tekrarlı engel_dur_reroute, araç kilitli).
    Çözüm: reset_gap büyütülür (~3s) → kısa boşluklar fazı KORUR; engel dönünce
    doğrudan FOLLOW ('slow'), yeni DUR yok. DUR yalnızca gerçekten yeni bir
    karşılaşmada (engel reset_gap'ten uzun süre banttan çıkıp geri gelince) tekrar
    yapılır. Yapışkan engel-kapısı (Debounce hold_ticks, main_tree) ile birlikte
    çalışır: boşlukta dal düşmez, 'slow' tutulur.

    Her zaman SUCCESS (commit bandındaki engele daima bir tepki üretilir).
    """

    def __init__(self, bb: Blackboard, pause_s: float, reset_gap_s: float = 0.5):
        super().__init__("RerouteKarar")
        self.bb = bb
        self.pause_s = max(0.0, float(pause_s))
        self.reset_gap_s = max(0.0, float(reset_gap_s))

    def update(self):
        o = self.bb.obs
        s = self.bb.state
        # Cone menzili: nearest overall, yoksa center
        rng = o.engel_d_overall
        if rng is None or not math.isfinite(rng):
            rng = o.engel_d_center
        if rng is None or not math.isfinite(rng):
            # Konum yok → reroute talebi AÇMA (yanlış (0,0) blok riski); güvenli slow.
            s.reroute_request = False
            s.reroute_phase = ""
            self.bb.last_decision = {
                "karar": "slow", "reason": "engel_reroute_nopos",
                "phase": "approach", "wait_remaining_s": 0.0,
            }
            return Status.SUCCESS

        ox, oy = obstacle_world_pos(o.x, o.y, o.yaw, rng, o.engel_angle_deg or 0.0)
        s.reroute_request = True
        s.reroute_cone_world = (ox, oy)
        s.kacis_engel_dunya = (ox, oy)   # trace log sürekliliği (eski alan)

        now = time.time()
        # Dormant reset: engel banttan çıkıp geri geldiyse yeni karşılaşma say.
        # reset_gap_s KISA olursa titreşim = "yeni karşılaşma" sanılır → tekrarlı
        # DUR (kilit). Büyük gap → kısa boşluk fazı korur; yalnız uzun temizlikte
        # yeni DUR (tek-seferlik DUR fix).
        if s.reroute_last_tick_s <= 0.0 or (now - s.reroute_last_tick_s) > self.reset_gap_s:
            s.reroute_phase = ""
        s.reroute_last_tick_s = now

        # STOP fazı — ilk girişte başlat, pause_s dolana kadar gerçek dur
        if s.reroute_phase == "":
            s.reroute_phase = "stop"
            s.reroute_stop_start_s = now
        if s.reroute_phase == "stop":
            kalan = self.pause_s - (now - s.reroute_stop_start_s)
            if kalan > 0.0:
                self.bb.last_decision = {
                    "karar": "dur", "reason": "engel_dur_reroute",
                    "phase": "reroute_stop", "wait_remaining_s": kalan,
                }
                return Status.SUCCESS
            s.reroute_phase = "follow"

        # FOLLOW fazı — reroute'u düşük hızda takip et (control yeni rotayı sürer)
        self.bb.last_decision = {
            "karar": "slow", "reason": "engel_reroute_follow",
            "phase": "reroute_follow", "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS


class HoldLaneChange(py_trees.behaviour.Behaviour):
    """Devam eden şerit değişiminin yön komutunu yeniden yayınlar.

    LaneChangeInProgress koşulu SUCCESS verdiğinde çağrılır; kilitli yönü
    (`bb.state.lane_change_dir`) aynen "sol"/"sag" olarak basar. Böylece
    control.py'nin başlattığı manevra (LANE_CHANGE_DURATION) kesintisiz tamamlanır.
    """

    def __init__(self, bb: Blackboard):
        super().__init__("HoldLaneChange")
        self.bb = bb

    def update(self):
        d = self.bb.state.lane_change_dir
        if d not in ("sol", "sag"):
            return Status.FAILURE
        self.bb.last_decision = {
            "karar": d,
            "reason": f"lane_change_hold:{d}",
            "phase": "lane_change",
            "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS
