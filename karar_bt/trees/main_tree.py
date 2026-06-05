"""Kök Behavior Tree birleşimi.

Öncelik (yukarıdan aşağı):
  0. Emergency latch / yeniden silahlanma (Safety)
  1. Emergency tetik (yaya/engel çok yakın → mührü kilitle)
  2. Yaya geçidi (yakın → dur, orta → slow)
  3. DUR levhası (approach → slow, hold → dur 3s, released → cruise'a düşer)
  4. Trafik ışığı (KIRMIZI/YAVAS)
  5. Engelden kaçınma (lane change varsa, yoksa dur)
  6. Yön levhası (SAG/SOL)
  7. Hız sınırı (30/OKUL)
  8. Cruise (default: normal)

Ağaç memory'siz selector: ilk SUCCESS dönen dal kararı sahiplenir.
"""
from __future__ import annotations

import py_trees
from py_trees.composites import Selector, Sequence

from bb import Blackboard
from behaviors.conditions import (
    YayaFresh, LevhaFresh, EngelFresh,
    YayaVarMi, YayaCokYakin, YayaYakin, YayaOrtaMesafe,
    EngelVar, EngelCokYakin, EngelMerkezBlokaj, YanSektorBos,
    LevhaIs, LevhaIcindeMesafe,
    EmergencyLatched,
    LaneChangeCooldownOk,
)
from behaviors.actions import (
    SetKarar, LatchEmergency, ReleaseEmergencyIfClear,
    DurLevhasiFSM, LaneChangeStamp,
)
from behaviors.decorators import Debounce


def build_root(bb: Blackboard, p: dict) -> py_trees.behaviour.Behaviour:
    """Ağacı kur. `p` config/params.yaml içeriğidir."""
    fresh = p["freshness"]
    dist = p["distances"]
    timer = p["timers"]
    deb = p["debounce"]
    emer = p["emergency"]
    lc = p["lane_change"]

    # ============================================================
    # 0. Safety: önce latch'i çözmeyi dene; çözüldüyse FAILURE
    #    döner ve aşağıdaki dallar konuşur. Hâlâ kapalıysa SUCCESS
    #    döner ve "acildurus" basılır.
    # ============================================================
    safety_release = ReleaseEmergencyIfClear(
        bb,
        release_clear_ticks=emer["release_clear_ticks"],
        yaya_esik=dist["yaya_acil_durus_m"] * 1.5,   # mührün çözülmesi için biraz daha geniş
        engel_esik=dist["engel_acil_m"] * 1.5,
    )

    # ============================================================
    # 1. Emergency tetik
    # ============================================================
    emergency_trigger = Sequence("EmergencyTrigger", memory=False, children=[
        Selector("EmergencyTriggers", memory=False, children=[
            # Engel: tazelik + çok yakın + debounce
            Sequence("EngelEmergency", memory=False, children=[
                EngelFresh(bb, fresh["engel_max_age_s"]),
                Debounce(
                    "EngelCokYakinDeb",
                    EngelCokYakin(bb, dist["engel_acil_m"]),
                    bb, key="engel_emergency",
                    min_consecutive=deb["engel_min_consecutive"],
                ),
            ]),
            # Yaya: tazelik + çok yakın + debounce
            Sequence("YayaEmergency", memory=False, children=[
                YayaFresh(bb, fresh["yaya_max_age_s"]),
                Debounce(
                    "YayaCokYakinDeb",
                    YayaCokYakin(bb, dist["yaya_acil_durus_m"]),
                    bb, key="yaya_emergency",
                    min_consecutive=deb["yaya_min_consecutive"],
                ),
            ]),
        ]),
        LatchEmergency(bb, reason="threshold_breach"),
    ])

    # ============================================================
    # 2. Yaya geçidi (acil değil)
    # ============================================================
    pedestrian = Sequence("Pedestrian", memory=False, children=[
        YayaFresh(bb, fresh["yaya_max_age_s"]),
        YayaVarMi(bb),
        Selector("YayaAksiyon", memory=False, children=[
            Sequence("YayaDur", memory=False, children=[
                Debounce("YayaYakinDeb",
                         YayaYakin(bb, dist["yaya_dur_m"]),
                         bb, key="yaya_yakin",
                         min_consecutive=deb["yaya_min_consecutive"]),
                SetKarar("Karar=DUR(yaya)", bb, karar="dur",
                         reason="yaya_dur", phase="driving"),
            ]),
            Sequence("YayaSlow", memory=False, children=[
                Debounce("YayaOrtaDeb",
                         YayaOrtaMesafe(bb, dist["yaya_yavas_m"]),
                         bb, key="yaya_orta",
                         min_consecutive=deb["yaya_min_consecutive"]),
                SetKarar("Karar=SLOW(yaya)", bb, karar="slow",
                         reason="yaya_yavas", phase="driving"),
            ]),
        ]),
    ])

    # ============================================================
    # 3. DUR levhası FSM
    # ============================================================
    stop_sign = Sequence("StopSign", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        DurLevhasiFSM(
            bb,
            stop_esik_m=dist["levha_dur_m"],
            oku_esik_m=dist["levha_oku_m"],
            bekleme_s=timer["dur_levhasi_bekleme_s"],
            release_grace_s=timer["dur_levhasi_release_grace_s"],
        ),
    ])

    # ============================================================
    # 4. Trafik ışığı
    # ============================================================
    traffic_light_red = Sequence("TrafficLightRed", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIs(bb, ("KIRMIZI",), max_mesafe_m=dist["levha_oku_m"]),
        SetKarar("Karar=DUR(kirmizi)", bb, karar="dur", reason="trafik_kirmizi"),
    ])
    traffic_light_yellow = Sequence("TrafficLightYellow", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIs(bb, ("YAVAS",), max_mesafe_m=dist["levha_oku_m"]),
        SetKarar("Karar=SLOW(sari)", bb, karar="slow", reason="trafik_sari"),
    ])

    # ============================================================
    # 5. Engelden kaçınma
    # ============================================================
    avoid_left = Sequence("AvoidLeft", memory=False, children=[
        YanSektorBos(bb, "sol", dist["engel_yan_clear_m"]),
        LaneChangeCooldownOk(bb, lc["cooldown_s"]),
        LaneChangeStamp(bb),
        SetKarar("Karar=SOL(engel)", bb, karar="sol", reason="engel_sol_kacis"),
    ])
    avoid_right = Sequence("AvoidRight", memory=False, children=[
        YanSektorBos(bb, "sag", dist["engel_yan_clear_m"]),
        LaneChangeCooldownOk(bb, lc["cooldown_s"]),
        LaneChangeStamp(bb),
        SetKarar("Karar=SAG(engel)", bb, karar="sag", reason="engel_sag_kacis"),
    ])
    obstacle_avoidance = Sequence("ObstacleAvoidance", memory=False, children=[
        EngelFresh(bb, fresh["engel_max_age_s"]),
        Debounce("EngelMerkezDeb",
                 EngelMerkezBlokaj(bb, dist["engel_block_m"]),
                 bb, key="engel_blokaj",
                 min_consecutive=deb["engel_min_consecutive"]),
        Selector("AvoidOrStop", memory=False, children=(
            [avoid_left, avoid_right] if lc["enabled"] else []
        ) + [
            SetKarar("Karar=DUR(engel)", bb, karar="dur", reason="engel_blokaj"),
        ]),
    ])

    # ============================================================
    # 6. Yön levhaları (SAG/SOL)
    # ============================================================
    direction_right = Sequence("DirectionRight", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIcindeMesafe(bb, "SAG", dist["yon_levha_max_m"]),
        LaneChangeCooldownOk(bb, lc["cooldown_s"]),
        LaneChangeStamp(bb),
        SetKarar("Karar=SAG(levha)", bb, karar="sag", reason="yon_levhasi_sag"),
    ])
    direction_left = Sequence("DirectionLeft", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIcindeMesafe(bb, "SOL", dist["yon_levha_max_m"]),
        LaneChangeCooldownOk(bb, lc["cooldown_s"]),
        LaneChangeStamp(bb),
        SetKarar("Karar=SOL(levha)", bb, karar="sol", reason="yon_levhasi_sol"),
    ])

    # ============================================================
    # 7. Hız sınırı levhası
    # ============================================================
    speed_limit = Sequence("SpeedLimit", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIs(bb, ("30", "OKUL"), max_mesafe_m=dist["levha_oku_m"]),
        SetKarar("Karar=SLOW(hiz)", bb, karar="slow", reason="hiz_siniri"),
    ])

    # ============================================================
    # 8. Cruise (default)
    # ============================================================
    cruise = SetKarar("Karar=NORMAL(cruise)", bb, karar="normal", reason="cruise")

    # ============================================================
    # Kök
    # ============================================================
    root = Selector("Root", memory=False, children=[
        safety_release,        # mühür çözülürse FAILURE → diğer dallar
        emergency_trigger,
        pedestrian,
        stop_sign,
        traffic_light_red,
        traffic_light_yellow,
        obstacle_avoidance,
        direction_right,
        direction_left,
        speed_limit,
        cruise,
    ])

    return root
