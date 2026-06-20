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
        self.bb.state.emergency_latched = True
        self.bb.state.emergency_clear_streak = 0
        self.bb.last_decision = {
            "karar": "acildurus",
            "reason": f"emergency_latch:{self.reason}",
            "phase": "emergency",
            "wait_remaining_s": 0.0,
        }
        return Status.SUCCESS


class ReleaseEmergencyIfClear(py_trees.behaviour.Behaviour):
    """Mühür kapalıyken NoOp. Açıkken: tüm tehlikeler temiz mi diye bakar;
    N tick üst üste temizse mührü çözer."""

    def __init__(self, bb: Blackboard, release_clear_ticks: int, yaya_esik: float, engel_esik: float):
        super().__init__("ReleaseEmergencyIfClear")
        self.bb = bb
        self.release_clear_ticks = int(release_clear_ticks)
        self.yaya_esik = yaya_esik
        self.engel_esik = engel_esik

    def update(self):
        if not self.bb.state.emergency_latched:
            return Status.FAILURE  # mühür yok → bu dal devam etmesin

        o = self.bb.obs
        yaya_clear = (not o.yaya_present) or (o.yaya_distance < 0) or (o.yaya_distance >= self.yaya_esik)
        # Engel present ama mesafe inf ise sensör verisi eksik → güvenli tarafta kal
        engel_d_valid = math.isfinite(o.engel_d_center)
        engel_clear = (not o.engel_present) or (engel_d_valid and o.engel_d_center >= self.engel_esik)

        if yaya_clear and engel_clear:
            self.bb.state.emergency_clear_streak += 1
        else:
            self.bb.state.emergency_clear_streak = 0

        if self.bb.state.emergency_clear_streak >= self.release_clear_ticks:
            self.bb.state.emergency_latched = False
            self.bb.state.emergency_clear_streak = 0
            # Mühür çözüldü ama bu tick hâlâ "acildurus" değil — alt dallar konuşsun.
            return Status.FAILURE  # üst selector bir sonraki dalı denesin

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
