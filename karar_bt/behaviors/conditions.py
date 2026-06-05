"""Condition node'ları.

Hepsi yan etkisiz; yalnız blackboard'u okur, SUCCESS/FAILURE döner.
"""
from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Status

from bb import Blackboard


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
    """Tam duruş eşiği."""
    def __init__(self, bb, esik_m):
        super().__init__(f"YayaYakin(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.yaya_distance
        return Status.SUCCESS if (d is not None and 0 < d < self.esik_m) else Status.FAILURE


class YayaOrtaMesafe(_Cond):
    """Yavaşlama eşiği."""
    def __init__(self, bb, esik_m):
        super().__init__(f"YayaOrta(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.yaya_distance
        return Status.SUCCESS if (d is not None and 0 < d < self.esik_m) else Status.FAILURE


# ============================================================
# Engel
# ============================================================
class EngelVar(_Cond):
    def __init__(self, bb):
        super().__init__("EngelVar?", bb)

    def update(self):
        return Status.SUCCESS if self.bb.obs.engel_present else Status.FAILURE


class EngelCokYakin(_Cond):
    """Merkez sektörde çok yakın engel — acil durus."""
    def __init__(self, bb, esik_m):
        super().__init__(f"EngelCokYakin(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.engel_d_center
        if d is None or not math.isfinite(d):
            return Status.FAILURE
        return Status.SUCCESS if d < self.esik_m else Status.FAILURE


class EngelMerkezBlokaj(_Cond):
    """Merkez sektörde sürüş engelleyici mesafede engel var mı?"""
    def __init__(self, bb, esik_m):
        super().__init__(f"EngelMerkezBlokaj(<{esik_m}m)?", bb)
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.engel_d_center
        if d is None or not math.isfinite(d):
            return Status.FAILURE
        return Status.SUCCESS if d < self.esik_m else Status.FAILURE


class YanSektorBos(_Cond):
    """Sol veya sağ sektörde belirli mesafeden uzakta engel yok mu?"""
    def __init__(self, bb, taraf: str, esik_m: float):
        assert taraf in ("sol", "sag")
        super().__init__(f"{taraf.upper()}Bos(>{esik_m}m)?", bb)
        self.taraf = taraf
        self.esik_m = esik_m

    def update(self):
        d = self.bb.obs.engel_d_left if self.taraf == "sol" else self.bb.obs.engel_d_right
        # inf veya eşikten büyük → boş
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
