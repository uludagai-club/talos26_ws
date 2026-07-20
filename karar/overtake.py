#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""overtake.py — (DEPRECATED, 2026-06-24) sollama + Ackermann geri-dönüş makinesi.

    ⚠ ARTIK KULLANILMIYOR. Cone kaçınması §16/§12.13 mimarisiyle değişti: cone
    artık karşı şeride DİREKSİYONLA (sollama + control offset) değil, planlayıcının
    rotayı dubanın etrafından çizmesiyle (ROTAYLA) geçiliyor. Yerini `reroute.py`
    (RerouteManager, /hedef_komut kenar_blok) aldı; karar_bt_node onu kullanır.
    Bu dosya + test_overtake.py geri-uyum/referans için bırakıldı, bir sonraki
    temizlikte silinebilir.
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
sollama (ters şeride geç) + güvenli geri-dönüş durum makinesi.

NE YAPAR
    Behavior Tree engelden kaçış için "sol"/"sag" verdiğinde, bu yön anlık
    direksiyon manevrasını (control.py) tetikler; AMA "tekrar ne zaman kendi
    şeridime döneceğim?" sorusu planlayıcıyı (hedef) ilgilendirir. Bu modül o
    kapalı-döngüyü kurar:

      1. Kaçışa çıkınca engelin DÜNYA konumunu sabitler (latch) — engel bir süre
         görüşten çıksa bile referans kaybolmaz.
      2. hedef'e `/hedef_komut` ile "sollama;taraf;ox;oy;..." gönderir
         (gelistirme_plani §3.2). hedef bu engeli bloklayıp karşı şeritten
         rota çizer (hedef tarafı Samed'in işi — bkz README/plan).
      3. Aracın engeli rota yönünde NE KADAR geçtiğini izler. Ackermann ile
         hesaplanan "çarpmadan dönebilme mesafesini" geçince `kenar_serbest`
         komutuyla hedef'e dönüşü bildirir. Erken dönüş = engele sürtme.

    ROS'suz, saf Python → `python3 karar/test/test_overtake.py` ile test edilir.
    karar_bt_node her tick'te update() çağırır; dönen komutu RosBridge yayınlar.

KOMUT BİÇİMİ (String prototip — plan §3.2: latch=False, queue_size=1)
    "sollama;<taraf>;<ox>;<oy>;engel;<yaricap>"     kaçışa çık
    "kenar_serbest;<taraf>;<ox>;<oy>;don;<yaricap>" şeride dön
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from avoidance_geometry import (
    obstacle_world_pos, longitudinal_gap, ackermann_return_distance,
)


@dataclass
class OvertakeParams:
    enabled: bool = True
    lane_offset_m: float = 1.8       # karşı şeride yanal kayma (geri-dönüş için de bu)
    return_steer_deg: float = 20.0   # geri-dönüş yayının direksiyonu (Ackermann R)
    clearance_m: float = 2.0         # araç ön çıkıntısı + engel yarıçapı + emniyet
    block_radius_m: float = 1.0      # hedef'e bildirilen engel blok yarıçapı
    max_s: float = 15.0              # sollama zaman aşımı (fallback dönüş)
    refresh_s: float = 1.0           # aktif komutu bu periyotla tazele (geç abone yakalasın)
    wheelbase_m: float = 1.78
    max_steer_deg: float = 30.0

    @classmethod
    def from_cfg(cls, ov: dict, ack: dict) -> "OvertakeParams":
        ov = ov or {}
        ack = ack or {}
        return cls(
            enabled=bool(ov.get("enabled", True)),
            lane_offset_m=float(ov.get("lane_offset_m", 1.8)),
            # >=5°: 0'a yakın direksiyon yanal kayma üretemez ama return_dist'i
            # clearance'a indirip ERKEN dönüş yaptırırdı (/incele algoritma bulgusu).
            return_steer_deg=max(5.0, float(ov.get("return_steer_deg", 20.0))),
            clearance_m=float(ov.get("clearance_m", 2.0)),
            block_radius_m=float(ov.get("block_radius_m", 1.0)),
            max_s=float(ov.get("max_s", 15.0)),
            refresh_s=float(ov.get("refresh_s", 1.0)),
            wheelbase_m=float(ack.get("wheelbase_m", 1.78)),
            max_steer_deg=float(ack.get("max_steer_deg", 30.0)),
        )


@dataclass
class OvertakeResult:
    command: Optional[str] = None     # /hedef_komut'a yayınlanacak (yoksa None)
    event: Optional[dict] = None      # karar_logger.log_overtake için (yoksa None)
    active: bool = False
    return_dist_m: float = 0.0
    gap_m: float = 0.0


class OvertakeManager:
    def __init__(self, params: OvertakeParams):
        self.p = params
        self._active = False
        self._dir = ""
        self._obs = (0.0, 0.0)        # latched engel dünya konumu
        self._route_dir = (1.0, 0.0)  # latched rota ilerleme yönü (dünya)
        self._return_dist = 0.0
        self._start_t = 0.0
        self._last_emit_t = 0.0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def obstacle_world(self):
        return self._obs

    # ------------------------------------------------------------------ #
    def _route_direction(self, rx, ry, yaw, hx, hy, nx, ny, hedef_fresh):
        """Rota ilerleme yönü (dünya). wp1→wp2 > robot→wp1 > yaw."""
        if hedef_fresh and hx is not None and hy is not None:
            if nx is not None and ny is not None:
                dx, dy = nx - hx, ny - hy
                if math.hypot(dx, dy) > 1e-3:
                    return dx, dy
            dx, dy = hx - rx, hy - ry
            if math.hypot(dx, dy) > 1e-3:
                return dx, dy
        return math.cos(yaw), math.sin(yaw)

    def update(self, *, rx, ry, yaw,
               engel_present, d_overall, d_center, angle_deg,
               hedef_x, hedef_y, next_hedef_x, next_hedef_y, hedef_fresh,
               decision_karar, now) -> OvertakeResult:
        if not self.p.enabled:
            return OvertakeResult(active=False)

        # ---------------- AKTİF DEĞİL: commit'i bekle ----------------- #
        if not self._active:
            if decision_karar not in ("sol", "sag"):
                return OvertakeResult(active=False)
            # Engel menzili: önce overall (en yakın), yoksa center
            rng = d_overall if (d_overall is not None and math.isfinite(d_overall)) else d_center
            if not engel_present or rng is None or not math.isfinite(rng):
                # Yön komutu var ama güvenilir engel konumu yok → latch'leme
                return OvertakeResult(active=False)

            ox, oy = obstacle_world_pos(rx, ry, yaw, rng, angle_deg or 0.0)
            self._route_dir = self._route_direction(
                rx, ry, yaw, hedef_x, hedef_y, next_hedef_x, next_hedef_y, hedef_fresh)
            self._return_dist = ackermann_return_distance(
                self.p.lane_offset_m, self.p.return_steer_deg,
                self.p.wheelbase_m, self.p.clearance_m)
            self._active = True
            self._dir = decision_karar
            self._obs = (ox, oy)
            self._start_t = now
            self._last_emit_t = now

            cmd = (f"sollama;{self._dir};{ox:.2f};{oy:.2f};engel;"
                   f"{self.p.block_radius_m:.2f}")
            ev = {
                "taraf": self._dir, "engel_dunya": [round(ox, 2), round(oy, 2)],
                "menzil_m": round(rng, 2), "aci_deg": round(angle_deg or 0.0, 1),
                "return_dist_m": round(self._return_dist, 2),
                "rota_yon": [round(self._route_dir[0], 2), round(self._route_dir[1], 2)],
                "kaynak_rota": bool(hedef_fresh),
            }
            return OvertakeResult(command=cmd, event=("basla", ev), active=True,
                                  return_dist_m=self._return_dist, gap_m=0.0)

        # ------------------- AKTİF: dönüşü/zaman aşımını yönet -------------- #
        ox, oy = self._obs
        gap = longitudinal_gap(rx, ry, ox, oy, self._route_dir[0], self._route_dir[1])

        # Zaman aşımı: engeli geçtiğimizi tespit edemedik → güvenli fallback dönüş
        if (now - self._start_t) > self.p.max_s:
            self._active = False
            cmd = f"kenar_serbest;{self._dir};{ox:.2f};{oy:.2f};don;{self.p.block_radius_m:.2f}"
            ev = {"taraf": self._dir, "gecen_s": round(now - self._start_t, 1),
                  "gap_m": round(gap, 2), "neden": "zaman_asimi"}
            return OvertakeResult(command=cmd, event=("zaman_asimi", ev), active=False,
                                  return_dist_m=self._return_dist, gap_m=gap)

        # Engeli Ackermann-güvenli mesafe kadar geçtik mi? → şeride dön
        if gap >= self._return_dist:
            self._active = False
            cmd = f"kenar_serbest;{self._dir};{ox:.2f};{oy:.2f};don;{self.p.block_radius_m:.2f}"
            ev = {"taraf": self._dir, "gap_m": round(gap, 2),
                  "return_dist_m": round(self._return_dist, 2), "neden": "engel_gecildi"}
            return OvertakeResult(command=cmd, event=("donus", ev), active=False,
                                  return_dist_m=self._return_dist, gap_m=gap)

        # Hâlâ sollamada: komutu düşük frekansta tazele (geç abone yakalasın)
        cmd = None
        if (now - self._last_emit_t) >= self.p.refresh_s:
            self._last_emit_t = now
            cmd = (f"sollama;{self._dir};{ox:.2f};{oy:.2f};engel;"
                   f"{self.p.block_radius_m:.2f}")
        return OvertakeResult(command=cmd, event=None, active=True,
                              return_dist_m=self._return_dist, gap_m=gap)

    def reset(self):
        self._active = False
        self._dir = ""
