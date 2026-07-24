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
# Trafik ışığı FSM (KIRMIZI → yeşile kadar DUR; DUR levhasından AYRI)
# ============================================================
class TrafikIsigiFSM(py_trees.behaviour.Behaviour):
    """Trafik ışığı kararı — KIRMIZI/SARI/YEŞİL birleşik, DUR levhasından AYRI.

    DUR levhası (DurLevhasiFSM) zaman-sınırlı bir duruştur: 3 sn bekle → devam.
    Trafik ışığı KOŞULLUDUR: renk döngüsüne göre sürülür. Eskiden algı kırmızıyı
    'DUR'a mapliyordu → kırmızı DUR levhası FSM'ine düşüp 3 sn sonra KALKIYORDU
    (kırmızıda geçme). 2026-07-23: algı lamba_kirmizi→KIRMIZI, lamba_sari→YAVAS,
    lamba_yesil→YESIL verir; bu FSM üçünü tek yerde sürer.

    `oku_esik_m` = IŞIK tepki mesafesi (traffic_light.isik_oku_m, ~40m) — DUR/yön
    levhalarının dar `levha_oku_m`'inden AYRI ve GENİŞ: ışık çok daha uzakta aksiyon
    ister (saha 2026-07-23: kırmızı 33–40m'de görülüyor, 10m hiç tetiklemiyordu).

    Renkler (algı okuma menzili `oku_esik_m` içinde):
      • KIRMIZI → 'dur' (yeşile kadar; zaman-sınırlı DEĞİL)
      • YAVAS (sarı) → 'slow':
          - kırmızıdan SONRA görülen sarı → yeşile YAVAŞTAN HAZIRLAN (her zaman
            slow; kullanıcı isteği 2026-07-23) → reason 'trafik_sari_hazir'
          - yaklaşırken (kırmızısız) sarı → `yellow_action` (slow|dur, KTR §7.5.5)
            → reason 'trafik_sari'
      • YESIL / ışık yok → FAILURE (geç; üst selector alt dallara iner)

    Son aksiyon-alan ışık (KIRMIZI/YAVAS) `release_grace_s` boyunca TUTULUR → kısa
    algı flicker'ı (levha 1-tick NONE) kararı bozmaz (kırmızıda öne seğirme /
    sarıda gaz kesme yok). Model YEŞİL sınıfı vermiyorsa: ışık grace boyunca hiç
    görülmeyince 'geçildi' sayılıp geçilir (süre-fallback).

    Güvenlik: emergency/yaya geçidi/DUR levhası bu dalın ÜSTÜNDE → onlar önceliklidir.
    Perception ölürse (LevhaFresh FAIL) dal düşer; alt dallar + emergency güvenlik ağı.
    """

    def __init__(self, bb: Blackboard, oku_esik_m: float, release_grace_s: float,
                 yellow_action: str = "slow"):
        super().__init__("TrafikIsigiFSM")
        self.bb = bb
        self.oku_esik_m = float(oku_esik_m)
        self.release_grace_s = float(release_grace_s)
        ya = str(yellow_action).lower()
        self.yellow_action = ya if ya in ("slow", "dur") else "slow"

    def update(self):
        o = self.bb.obs
        s = self.bb.state
        now = time.time()
        isim = o.levha_isim
        in_range = (o.levha_distance is not None
                    and 0.0 < o.levha_distance <= self.oku_esik_m)

        # YEŞİL pozitif → ışık bitti, geç (anında; üst selector ilerlesin)
        if isim == "YESIL":
            s.trafik_isik_last_light = ""
            s.trafik_isik_hazir = False
            return Status.FAILURE

        if isim in ("KIRMIZI", "YAVAS") and in_range:
            prev = s.trafik_isik_last_light
            if isim == "YAVAS" and prev == "KIRMIZI":
                s.trafik_isik_hazir = True    # kırmızı→sarı → yeşile hazırlan
            elif isim == "KIRMIZI":
                s.trafik_isik_hazir = False
            s.trafik_isik_last_light = isim
            s.trafik_isik_last_s = now
        elif s.trafik_isik_last_light:
            # Aksiyon-alan ışık şu an görünmüyor → grace içinde TUT, sonra temizle
            if (now - s.trafik_isik_last_s) >= self.release_grace_s:
                s.trafik_isik_last_light = ""
                s.trafik_isik_hazir = False

        if s.trafik_isik_last_light == "KIRMIZI":
            self.bb.last_decision = {
                "karar": "dur",
                "reason": "trafik_kirmizi",
                "phase": "waiting_red_light",
                "wait_remaining_s": 0.0,
            }
            return Status.SUCCESS
        if s.trafik_isik_last_light == "YAVAS":
            if s.trafik_isik_hazir:
                karar, reason = "slow", "trafik_sari_hazir"   # yeşile yavaştan hazırlan
            else:
                karar, reason = self.yellow_action, "trafik_sari"
            self.bb.last_decision = {
                "karar": karar,
                "reason": reason,
                "phase": "approach",
                "wait_remaining_s": 0.0,
            }
            return Status.SUCCESS

        return Status.FAILURE   # ışık yok / yeşil → üst selector diğer dallara


# ============================================================
# Yaya geçidi FSM (min zorunlu duruş + lidar engel ile yaya-bekleme)
# ============================================================
class YayaGecidiFSM(py_trees.behaviour.Behaviour):
    """Yaya geçidi kararı — adanmış crosswalk modeli sinyaliyle (GEÇİCİ köprü).

    Adanmış model YALNIZ geçit ÇİZGİSİNİ tespit ediyor ('crosswalk'), yayanın
    kendisini ('object') değil (algi/yaya_gecidi_node.py). Bu yüzden yaya olsun
    olmasın geçitte MİNİMAL zorunlu duruş yapılır; sonra "geçitte yaya var mı"
    sorusu LİDAR engel verisiyle yanıtlanır: engel (yaya) varsa geçene kadar
    (bir üst sınıra dek) beklenir, yoksa min bekleme dolunca DEVAM edilir.
    (Görüntü-işleme ekibi 'object' sınıfını çözene dek — sonra bu FSM engel
    yerine gerçek yaya tespitini okuyacak şekilde daraltılır.)

    SUSTAIN-HOLDING (2026-07-23, kullanıcı isteği): minimal zorunlu duruş bir kez
    başlayınca ÇİZGİ DÜŞSE DE `min_bekleme_s` tamamlanır — crosswalk modeli tam
    geçitte tespiti düşürdüğü için (cizgi:none flicker) eski yapıda 1-tick sonra
    duruş kesiliyordu. Bunun için bu dal artık sequence'te YayaFresh/YayaVarMi'ye
    bağlı DEĞİL; çizgi tazelik/present kontrolü FSM İÇİNDE (yaya_max_age_s) ve yalnız
    idle→tetik + approach için kullanılır. Kapı (YayaLevhaKapisi) armed olduğu sürece
    FSM her tick ticklenir; holding zamanlayıcıyla sürer.

    Fazlar:
      - IDLE→tetik: çizgi taze+present ise; mesafe < dur_esik → HOLDING, dur_esik ≤
        mesafe ≤ yavas_esik → "slow" (approach). Çizgi yoksa FAILURE (cruise).
      - HOLDING (çizgiden bağımsız sürer): "dur"
          • gecen < min_bekleme_s                    → zorunlu duruş
          • min doldu + lidar engel (yaya) var       → max_bekleme_s'e dek bekle
          • temiz VEYA max doldu                      → RELEASED
      - RELEASED: FAILURE. Kapı-enable ise gate kapanır (yeni tabelada idle reset);
        kapı-disable ise çizgi gidince idle'a döner (yeniden silahlanma).

    Kilitlenme önlemi (eski bug): mesafe geçidin ÜSTÜNDE ~0.6m'de donuyor ve hiç
    büyümüyordu → sonsuz 'dur'. Çözüm: (a) RELEASED mesafeyle değil zamanla/engelle
    verilir, (b) yeniden silahlanma yalnız `yaya_present`e bakar (yakında mesafe
    0.6↔17.2 zıpladığı için mesafe eşiği güvenilmez), (c) max_bekleme_s üst sınırı
    statik/yanlış engelde bile kalıcı kilidi keser. Erken release olsa da alttaki
    obstacle_avoidance + üstteki emergency dalları güvenlik ağı olarak kalır.
    """

    def __init__(self, bb: Blackboard, dur_esik_m: float, yavas_esik_m: float,
                 min_bekleme_s: float, max_bekleme_s: float, engel_bekle_m: float,
                 release_grace_s: float, yaya_max_age_s: float = 0.6):
        super().__init__("YayaGecidiFSM")
        self.bb = bb
        self.dur_esik_m = dur_esik_m
        self.yavas_esik_m = yavas_esik_m
        self.min_bekleme_s = min_bekleme_s
        self.max_bekleme_s = max_bekleme_s
        self.engel_bekle_m = engel_bekle_m
        self.release_grace_s = release_grace_s
        self.yaya_max_age_s = float(yaya_max_age_s)

    @staticmethod
    def _fresh(last_seen: float, max_age_s: float) -> bool:
        return last_seen > 0.0 and (time.time() - last_seen) <= max_age_s

    def _yaya_engeli_var(self) -> bool:
        """Geçit bölgesinde lidar engeli (yaya proxy'si) var mı? Merkez sektörde
        engel_bekle_m içinde engel → 'yaya geçiyor' say. (Görüntü-işleme 'object'
        sınıfını verince burası gerçek yaya tespitiyle değişir.)

        KONİ ÇAKIŞMASI (2026-07-22 canlı): lidar koni ile yayayı ayıramaz. Bir koni
        geçidin yanındaysa engel_present=True olur ve geçit FSM'i onu "yaya" sanıp
        beklerdi → reroute'la çakışıp salınım + acildurus (araç takılır). RerouteManager
        o koniyi zaten izliyorsa (overtake_active) engel KONİDİR, yaya değil → yaya
        sayma; min duruştan sonra bırak, koniyi obstacle_avoidance/reroute geçsin.
        Gerçek yaya (ortada izlenen koni yokken) hâlâ beklenir; çok yakın yaya için
        emergency/obstacle dalları güvenlik ağı olarak kalır."""
        o = self.bb.obs
        if not o.engel_present:
            return False
        if self.bb.state.overtake_active:   # engel izlenen bir koni → yaya değil
            return False
        d = o.engel_d_center
        return d is not None and math.isfinite(d) and 0.0 < d < self.engel_bekle_m

    def update(self):
        o = self.bb.obs
        s = self.bb.state
        now = time.time()
        d = o.yaya_distance
        # Çizgi (crosswalk LINE) taze + present mi (freshness FSM içinde — bu dal
        # artık sequence'te YayaFresh/YayaVarMi'ye bağlı DEĞİL, çünkü holding çizgi
        # düşse de sürmeli). Kapı (YayaLevhaKapisi) armed olduğu sürece FSM ticklenir.
        line = (o.yaya_present and d is not None and d > 0
                and self._fresh(o.yaya_last_seen, self.yaya_max_age_s))

        # ============================================================
        # HOLDING — minimal duruş ÇİZGİDEN BAĞIMSIZ sürer. Kullanıcı isteği:
        # "geçit varken engel olmasa bile minimal dur." Çizgi 1-tick flicker'ı
        # (crosswalk modeli tam geçitte tespiti düşürüyor) 3s duruşu BÖLMEZ; duruş
        # kapı-armed + zamanlayıcıyla sürer. Araç durunca kapı da armed kalır
        # (tabela çapası geçilmez, FSM release olmaz).
        # ============================================================
        if s.yaya_gecidi_phase == "holding":
            gecen = now - s.yaya_gecidi_hold_start_s
            if gecen < self.min_bekleme_s:                     # 1) zorunlu min duruş
                self.bb.last_decision = {
                    "karar": "dur", "reason": "yaya_gecidi_min_dur",
                    "phase": "waiting_at_crosswalk",
                    "wait_remaining_s": max(0.0, self.min_bekleme_s - gecen),
                }
                return Status.SUCCESS
            if self._yaya_engeli_var() and gecen < self.max_bekleme_s:  # 2) yaya (lidar) → bekle
                self.bb.last_decision = {
                    "karar": "dur", "reason": "yaya_gecidi_yaya_bekle",
                    "phase": "waiting_at_crosswalk",
                    "wait_remaining_s": max(0.0, self.max_bekleme_s - gecen),
                }
                return Status.SUCCESS
            s.yaya_gecidi_phase = "released"                   # 3) temiz/max → devam
            s.yaya_gecidi_released_s = now
            return Status.FAILURE

        # RELEASED — bu tick karar yok. Kapı ENABLE ise gate release'i görüp kapanır
        # (yeni tabelada idle'a resetler). Kapı DEVRE DIŞI (pass-through) ise çizgi
        # gidince burada idle'a döneriz (yeniden silahlanma).
        if s.yaya_gecidi_phase == "released":
            if not line:
                s.yaya_gecidi_phase = "idle"
            return Status.FAILURE

        # IDLE — çizgi ile tetik
        if (self.release_grace_s > 0.0 and s.yaya_gecidi_released_s > 0.0
                and (now - s.yaya_gecidi_released_s) < self.release_grace_s):
            return Status.FAILURE
        if not line:
            return Status.FAILURE   # kapı açık ama çizgi henüz yok → cruise
        if d < self.dur_esik_m:
            s.yaya_gecidi_phase = "holding"
            s.yaya_gecidi_hold_start_s = now
            self.bb.last_decision = {
                "karar": "dur", "reason": "yaya_gecidi_min_dur",
                "phase": "waiting_at_crosswalk",
                "wait_remaining_s": self.min_bekleme_s,
            }
            return Status.SUCCESS
        if d <= self.yavas_esik_m:
            self.bb.last_decision = {
                "karar": "slow", "reason": "yaya_gecidi_yaklasma",
                "phase": "approach", "wait_remaining_s": 0.0,
            }
            return Status.SUCCESS
        return Status.FAILURE


# ============================================================
# Yaya geçidi LEVHA-KAPISI (çizgi modeline yalnız levha görülünce güven)
# ============================================================
class YayaLevhaKapisi(py_trees.behaviour.Behaviour):
    """Yaya geçidi kararı için ÖN-KAPI: adanmış çizgi modeline (/yaya_gecidi/model)
    yalnız yaya geçidi LEVHASI (/yaya_gecidi, levha modeli) görülmüşse güvenilir.

    NEDEN: Çizgi modeli geçit çizgisini ŞERİT çizgisiyle karıştırabiliyor →
    levhasız sahte 'crosswalk' → gereksiz duruş. Levha (yol kenarındaki yaya
    geçidi tabelası) görülünce kapı AÇILIR; kapı kapalıyken çizgi modeli TÜMDEN
    yok sayılır (bu node FAILURE döner → pedestrian sequence düşer).

    `arm_menzil_m` GENİŞ tutulur (~45m): geçit tabelası geçidi UZAKTAN duyurur (saha
    2026-07-23: tabela 41–49m'de görülüyordu; dar 10m yüzünden kapı açılmıyor, araç
    gerçek zebradan geçiyordu). DURUŞ mesafesini çizgi FSM'i (yaya_dur_m/yaya_yavas_m)
    belirler; bu değer yalnız "geçit duyuruldu → çizgiye güven" kapısını açar.

    Kapı YAŞAM DÖNGÜSÜ (kullanıcı: "release olunca kapansın, levhayı geçince de"):
      • AÇ   — tabela taze + `arm_menzil_m` içinde görülünce. Yeni epizot: çizgi
               FSM'i sıfırlanır (takılı 'released' fazı yeni geçidi bloklamasın).
      • KAPAN— (a) çizgi FSM 'released' oldu (geçit işlendi), VEYA
               (b) levha geçildi (dünya-çapası `pass_behind_m` gerisinde), VEYA
               (c) fail-safe: kapı `arm_max_s`'ten uzun açık kaldı.
      • grace— kapandıktan sonra `grace_s` boyunca yeniden silahlanmaz (aynı
               levhanın algı titremesiyle kapıyı flip-flop yapmasını önler).

    `enabled=False` → pass-through (daima SUCCESS): eski davranış (çizgiye hep güven).
    Dünya-çapası/geçildi tespiti taze odom ister; odom yoksa kapı yalnız release
    ve fail-safe TTL ile kapanır (geçit işleyişi odomdan bağımsız sürer).
    """

    def __init__(self, bb: Blackboard, enabled: bool, levha_max_age_s: float,
                 odom_max_age_s: float, arm_menzil_m: float, pass_behind_m: float,
                 arm_max_s: float, grace_s: float):
        super().__init__("YayaLevhaKapisi")
        self.bb = bb
        self.enabled = bool(enabled)
        self.levha_max_age_s = float(levha_max_age_s)
        self.odom_max_age_s = float(odom_max_age_s)
        self.arm_menzil_m = float(arm_menzil_m)
        self.pass_behind_m = float(pass_behind_m)
        self.arm_max_s = float(arm_max_s)
        self.grace_s = float(grace_s)

    @staticmethod
    def _fresh(last_seen: float, max_age_s: float) -> bool:
        return last_seen > 0.0 and (time.time() - last_seen) <= max_age_s

    def _kapat(self, s):
        s.yaya_kapi_armed = False
        s.yaya_kapi_anchored = False
        s.yaya_kapi_released_s = time.time()

    def update(self):
        # Devre dışı → pass-through: çizgi modeline eskisi gibi hep güven.
        if not self.enabled:
            return Status.SUCCESS

        o = self.bb.obs
        s = self.bb.state
        now = time.time()

        levha_taze = self._fresh(o.yaya_levha_last_seen, self.levha_max_age_s)
        odom_taze = self._fresh(o.odom_last_seen, self.odom_max_age_s)
        grace_ic = (self.grace_s > 0.0 and s.yaya_kapi_released_s > 0.0
                    and (now - s.yaya_kapi_released_s) < self.grace_s)

        # --- Silahlanma: levha taze + menzil içinde (grace dışında) ---
        if (levha_taze and o.yaya_levha_present
                and o.yaya_levha_distance is not None
                and 0.0 < o.yaya_levha_distance <= self.arm_menzil_m
                and not grace_ic):
            if not s.yaya_kapi_armed:
                # Yeni epizot: kapıyı aç + çizgi FSM'ini sıfırdan başlat.
                s.yaya_kapi_armed = True
                s.yaya_kapi_arm_s = now
                s.yaya_kapi_anchored = False
                s.yaya_gecidi_phase = "idle"
                s.yaya_gecidi_released_s = 0.0
            if odom_taze:
                # Çapayı en taze levha konumuna çek (geçildi tespiti için)
                s.yaya_kapi_anchor = (
                    o.x + math.cos(o.yaw) * o.yaya_levha_distance,
                    o.y + math.sin(o.yaw) * o.yaya_levha_distance,
                )
                s.yaya_kapi_anchored = True

        if not s.yaya_kapi_armed:
            return Status.FAILURE   # kapı kapalı → çizgi modeli tümden yok sayılır

        # --- Kapatma koşulları ---
        # (a) çizgi FSM release oldu → geçit işlendi
        if s.yaya_gecidi_phase == "released":
            self._kapat(s)
            return Status.FAILURE
        # (b) levhayı geçtik → çapa pass_behind_m gerisinde (odom + geçerli çapa şart)
        if odom_taze and s.yaya_kapi_anchored:
            dx = s.yaya_kapi_anchor[0] - o.x
            dy = s.yaya_kapi_anchor[1] - o.y
            ileri = math.cos(o.yaw) * dx + math.sin(o.yaw) * dy
            if ileri < -self.pass_behind_m:
                self._kapat(s)
                return Status.FAILURE
        # (c) fail-safe: kapı çok uzun açık kaldı (release/geçildi gelmedi)
        if self.arm_max_s > 0.0 and (now - s.yaya_kapi_arm_s) > self.arm_max_s:
            self._kapat(s)
            return Status.FAILURE

        return Status.SUCCESS   # kapı açık → alt dallar (YayaFresh→FSM) çalışsın


# ============================================================
# Park müsaitlik FSM (2026-07-24) — "park tabelası → model → lidar" üç-kapılı AND
# ============================================================
class ParkFSM(py_trees.behaviour.Behaviour):
    """Park alanı MÜSAİTLİK kararı — yaya geçidi levha-kapısı desenini yansıtır.

    AKIŞ (kullanıcı isteği): park tabelası görülünce park alanı modeli DEVREYE
    girer; müsaitlik üç koşulun AND'idir:
      Kapı 1  PARK_YERI levhası görüldü mü?  (kapıyı arm eder; model ancak bundan
              sonra dinlenir — /park_alani'ya levhasız güvenilmez). PARK_ETMEK_
              YASAKTIR görülürse park YASAK → doğrudan "müsait değil".
      Kapı 2  /park_alani modeli park alanı gösteriyor mu?  (present + taze)
      Kapı 3  o alanda lidar engeli YOK mu?  → 2026-07-24: ERTELENDİ. Bag analizinden
              sonra eklenecek; `lidar_enabled=False` iken True kabul edilir.
    Üçü de olumlu → "park müsait"; biri bile yoksa → "park müsait değil".

    KAPI YAŞAM DÖNGÜSÜ (YayaLevhaKapisi ile aynı mantık):
      • ARM   — PARK_YERI levhası taze + `arm_menzil_m` içinde görülünce (tabela
                park cebini UZAKTAN duyurur; bbox-proxy mesafe kaba olduğu için
                menzil geniş tutulur). Levha 1-tick düşse de epizot sürer (sustain).
      • KAPAN — (a) levhayı geçtik (dünya-çapası `pass_behind_m` gerisinde), VEYA
                (b) fail-safe: kapı `arm_max_s`'ten uzun açık kaldı.
      • grace — kapandıktan sonra `grace_s` boyunca yeniden silahlanmaz (flip-flop).

    ÇIKTI: motion komutu epizot boyunca 'slow' (park cebine yaklaşma/tarama hızı;
    asıl park manevrasını mission/hedef yürütür — karar yalnız MÜSAİTLİĞİ raporlar).
    Müsaitlik `reason`/`phase` ile taşınır: park_musait / park_musait_degil /
    park_yasak. `enabled=False` → dal tümden kapalı (FAILURE → cruise). control
    tarafı park motion'ını henüz ele almadı (2026-07-24 kullanıcı: sonra) — bu yüzden
    'slow' güvenli köprü; mission reason/phase'i okur.
    """

    def __init__(self, bb: Blackboard, enabled: bool, arm_menzil_m: float,
                 levha_max_age_s: float, park_max_age_s: float, odom_max_age_s: float,
                 pass_behind_m: float, arm_max_s: float, grace_s: float,
                 lidar_enabled: bool = False):
        super().__init__("ParkFSM")
        self.bb = bb
        self.enabled = bool(enabled)
        self.arm_menzil_m = float(arm_menzil_m)
        self.levha_max_age_s = float(levha_max_age_s)
        self.park_max_age_s = float(park_max_age_s)
        self.odom_max_age_s = float(odom_max_age_s)
        self.pass_behind_m = float(pass_behind_m)
        self.arm_max_s = float(arm_max_s)
        self.grace_s = float(grace_s)
        self.lidar_enabled = bool(lidar_enabled)

    @staticmethod
    def _fresh(last_seen: float, max_age_s: float) -> bool:
        return last_seen > 0.0 and (time.time() - last_seen) <= max_age_s

    def _kapat(self, s):
        s.park_phase = "released"
        s.park_kapi_anchored = False
        s.park_kapi_released_s = time.time()

    def _emit(self, reason: str, phase: str):
        self.bb.last_decision = {
            "karar": "slow", "reason": reason,
            "phase": phase, "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS

    def update(self):
        if not self.enabled:
            return Status.FAILURE

        o = self.bb.obs
        s = self.bb.state
        now = time.time()

        levha_taze = self._fresh(o.levha_last_seen, self.levha_max_age_s)
        odom_taze = self._fresh(o.odom_last_seen, self.odom_max_age_s)
        d_levha = o.levha_distance
        menzilde = (d_levha is not None and 0.0 < d_levha <= self.arm_menzil_m)
        is_park_yeri = levha_taze and o.levha_isim == "PARK_YERI" and menzilde
        is_yasak = levha_taze and o.levha_isim == "PARK_ETMEK_YASAKTIR" and menzilde
        grace_ic = (self.grace_s > 0.0 and s.park_kapi_released_s > 0.0
                    and (now - s.park_kapi_released_s) < self.grace_s)

        # --- Silahlanma: PARK_YERI levhası → kapıyı aç (grace dışında) ---
        if is_park_yeri and not grace_ic:
            if s.park_phase != "armed":
                s.park_phase = "armed"
                s.park_kapi_arm_s = now
                s.park_kapi_anchored = False
            if odom_taze:
                s.park_kapi_anchor = (
                    o.x + math.cos(o.yaw) * d_levha,
                    o.y + math.sin(o.yaw) * d_levha,
                )
                s.park_kapi_anchored = True

        # --- Kapı kapalı (idle/released): park epizodu yok ---
        if s.park_phase != "armed":
            if s.park_phase == "released" and not is_park_yeri:
                s.park_phase = "idle"
            if is_yasak:   # tabela park yasağı → müsait değil (epizottan bağımsız uyarı)
                return self._emit("park_yasak", "park_yasak")
            return Status.FAILURE   # cruise (park tabelası yok)

        # --- armed: kapatma koşulları ---
        # (a) levhayı geçtik → çapa pass_behind_m gerisinde (odom + geçerli çapa şart)
        if odom_taze and s.park_kapi_anchored:
            dx = s.park_kapi_anchor[0] - o.x
            dy = s.park_kapi_anchor[1] - o.y
            ileri = math.cos(o.yaw) * dx + math.sin(o.yaw) * dy
            if ileri < -self.pass_behind_m:
                self._kapat(s)
                return Status.FAILURE
        # (b) fail-safe: kapı çok uzun açık kaldı
        if self.arm_max_s > 0.0 and (now - s.park_kapi_arm_s) > self.arm_max_s:
            self._kapat(s)
            return Status.FAILURE

        # --- armed: üç-kapılı AND ile müsaitlik ---
        # Kapı 1 (PARK_YERI) armed olmakla sağlandı. Yasak levha araya girdiyse müsait değil.
        if is_yasak:
            return self._emit("park_yasak", "park_yasak")
        # Kapı 2: /park_alani modeli park alanı gösteriyor mu (present + taze)
        model_ok = (o.park_alani_present
                    and self._fresh(o.park_alani_last_seen, self.park_max_age_s))
        # Kapı 3: lidar engel yok mu — 2026-07-24 ERTELENDİ (bag analizi sonrası)
        lidar_ok = True if not self.lidar_enabled else True  # TODO(kapı3): lidar engel taraması
        if model_ok and lidar_ok:
            return self._emit("park_musait", "park_musait")
        # Model henüz alan göstermiyor → epizot içinde tarama; verdict: müsait değil
        return self._emit("park_musait_degil", "park_tarama")


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
