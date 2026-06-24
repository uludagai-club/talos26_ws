"""Kök Behavior Tree birleşimi.

Öncelik (yukarıdan aşağı):
  0. Emergency latch / yeniden silahlanma (Safety)
  1. Emergency tetik (yaya/engel çok yakın → mührü kilitle)
  2. Yaya geçidi (yakın → dur, orta → slow)
  3. DUR levhası (approach → slow, hold → dur 3s, released → cruise'a düşer)
  4. Trafik ışığı (KIRMIZI/YAVAS)
  5a. Şerit değişimi kilidi (devam eden manevrayı control.py senkronuyla tut)
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
    EngelCokYakin, EngelMerkezBlokaj, YanSektorBos,
    KacisYonuSec, YanSektorBosSecilen,
    LevhaIs, LevhaIcindeMesafe,
    LaneChangeCooldownOk, LaneChangeInProgress,
)
from behaviors.actions import (
    SetKarar, LatchEmergency, ReleaseEmergencyIfClear,
    DurLevhasiFSM, LaneChangeStamp, KacisKarar, HoldLaneChange,
)
from behaviors.decorators import Debounce


def build_root(bb: Blackboard, p: dict) -> py_trees.behaviour.Behaviour:
    """Ağacı kur. `p` config/params.yaml içeriğidir."""
    # ===================================================================
    # AYAR BLOĞU — ENGEL / ŞERİT MESAFELERİ  (sahada hızlı tune için TEK YER)
    # -------------------------------------------------------------------
    # Tüm engel tepki bantları control.py WP_NEAR_DISTANCE referansından
    # ± delta ile türetilir. Bir mesafeyi değiştirmek için: config/params.yaml
    # içindeki ilgili anahtarı düzenle (yoksa buradaki default geçerli).
    # Bant sırası (yakın → uzak):  acil < dur < block(commit) < yavasla
    #   acil    ≈ 1.2 m  → acil durus (emniyetli son çare)
    #   dur     ≈ 2.0 m  → kaçış yoksa tam dur
    #   block   ≈ 6.0 m  → SLALOM/kaçışa (sol/sag) COMMIT menzili (§12.12: 3.5→6.0)
    #   yavasla ≈ 9.0 m  → en dış: tepki başlar, yavaşla (commit üstü ~3m slow-zone)
    # NOT: lane-change'in FİİLEN tamamlanması için block menzili Ackermann
    #      yayına yetmeli. Golf-cart (L=1.78m, R≈3.08m) 1.79m yanalı GÜVENLE
    #      açmak ~5-6m gerektirir (doc §12.12, run 165604) → commit 6m'ye çekildi.
    #      Araç hâlâ engele giriyorsa engel_block_delta_m'i daha da BÜYÜT.
    # ===================================================================
    wp   = p.get("wp_planlama", {}) or {}
    lc   = p["lane_change"]
    dist = p["distances"]

    wp_near        = float(wp.get("control_wp_near_m",     1.5))   # = control.py WP_NEAR_DISTANCE (senkron tut!)
    wp_hyst        = float(wp.get("aktif_wp_histerezis_m", 0.5))   # aktif WP segment geçiş histerezisi
    kacis_deadband = float(wp.get("kacis_deadband_m",      0.7))   # rotaya bu kadar yakın engel = "merkez koni"
    varsayilan_yon = str(  wp.get("varsayilan_kacis_yon", "sol"))  # merkez koni → bu yöne kaç (ters/karşı şerit)

    engel_acil_m      = max(0.3, wp_near + float(wp.get("engel_acil_delta_m",    -0.3)))  # acil durus
    engel_dur_m       =          wp_near + float(wp.get("engel_dur_delta_m",      0.5))   # kaçış yoksa dur
    engel_block_m     =          wp_near + float(wp.get("engel_block_delta_m",    4.5))   # sol/sag SLALOM commit (§12.12: →6.0m)
    engel_yavasla_m   =          wp_near + float(wp.get("engel_yavasla_delta_m",  7.5))   # en dış: yavasla (→9.0m)
    engel_yan_clear_m = float(dist.get("engel_yan_clear_m", 3.0))  # yan sektör bu kadar boşsa lane-change makul

    lc_cooldown_s = float(lc.get("cooldown_s",      4.0))   # ardışık şerit değişimi arası bekleme
    lc_hold_s     = float(lc.get("maneuver_hold_s", 2.0))   # = control.py LANE_CHANGE_DURATION (manevra tut süresi)
    # ===================================================================

    fresh = p["freshness"]
    timer = p["timers"]
    deb = p["debounce"]
    emer = p["emergency"]
    sa = p.get("speed_adaptive", {})
    # Hız-uyumu kapalıysa gain'leri 0'la → eşikler tabanda kalır
    sa_on = bool(sa.get("enabled", False))
    sa_max_extra = sa.get("max_extra_m", 0.0)
    odom_age = fresh["odom_max_age_s"]

    def _gain(key: str) -> float:
        return sa.get(key, 0.0) if sa_on else 0.0

    # ============================================================
    # 0. Safety: önce latch'i çözmeyi dene; çözüldüyse FAILURE
    #    döner ve aşağıdaki dallar konuşur. Hâlâ kapalıysa SUCCESS
    #    döner ve "acildurus" basılır.
    # ============================================================
    safety_release = ReleaseEmergencyIfClear(
        bb,
        release_clear_ticks=emer["release_clear_ticks"],
        yaya_esik=dist["yaya_acil_durus_m"] * 1.5,   # mührün çözülmesi için biraz daha geniş
        engel_esik=engel_acil_m * 1.5,
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
                    EngelCokYakin(bb, engel_acil_m),
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
                         YayaYakin(bb, dist["yaya_dur_m"],
                                   gain_s=_gain("yaya_dur_gain_s"),
                                   max_extra_m=sa_max_extra, odom_max_age_s=odom_age),
                         bb, key="yaya_yakin",
                         min_consecutive=deb["yaya_min_consecutive"]),
                SetKarar("Karar=DUR(yaya)", bb, karar="dur",
                         reason="yaya_dur", phase="driving"),
            ]),
            Sequence("YayaSlow", memory=False, children=[
                Debounce("YayaOrtaDeb",
                         YayaOrtaMesafe(bb, dist["yaya_yavas_m"],
                                        gain_s=_gain("yaya_yavas_gain_s"),
                                        max_extra_m=sa_max_extra, odom_max_age_s=odom_age),
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
    # Sarı (YAVAS) ışık aksiyonu paramla seçilir: "slow" (varsayılan) | "dur".
    _yellow = str(p.get("traffic_light", {}).get("yellow_action", "slow")).lower()
    _yellow = _yellow if _yellow in ("slow", "dur") else "slow"
    traffic_light_yellow = Sequence("TrafficLightYellow", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIs(bb, ("YAVAS",), max_mesafe_m=dist["levha_oku_m"]),
        SetKarar(f"Karar={_yellow.upper()}(sari)", bb, karar=_yellow, reason="trafik_sari"),
    ])

    # ============================================================
    # 5. Engelden kaçınma — KATMANLI MESAFE (kullanıcı isteği)
    #    Dış kapı: engel yavasla bandında (en geniş) → en az "yavasla".
    #    İçeride öncelik:
    #      a) block menzilinde + yola göre seçilen taraf BOŞ → sol/sag (kaçış)
    #      b) dur menzilinde + kaçış yok → dur (son çare)
    #      c) aksi halde (orta menzil, henüz net taraf yok) → yavasla
    #    "Ne dur ne sol/sag diyemiyorsan yavaşla" → dur artık yalnız yakın+çaresiz.
    # ============================================================
    road_aware_avoid = Sequence("RoadAwareAvoid", memory=False, children=[
        EngelMerkezBlokaj(bb, engel_block_m),                 # commit menzilinde mi?
        KacisYonuSec(bb, deadband_m=kacis_deadband, wp_near_m=wp_near,
                     wp_hyst_m=wp_hyst, hedef_max_age_s=fresh["hedef_max_age_s"],
                     varsayilan_yon=varsayilan_yon),          # yönü yola göre seç (hep SUCCESS)
        YanSektorBosSecilen(bb, engel_yan_clear_m, fresh["engel_max_age_s"]),
        LaneChangeCooldownOk(bb, lc_cooldown_s),
        KacisKarar(bb),                                       # sol/sag + cooldown/yön damgala
    ])
    engel_dur = Sequence("EngelDur", memory=False, children=[
        EngelMerkezBlokaj(bb, engel_dur_m),                  # dur menzilinde + kaçış yok
        SetKarar("Karar=DUR(engel)", bb, karar="dur", reason="engel_blokaj"),
    ])
    engel_yavasla = SetKarar("Karar=SLOW(engel)", bb, karar="slow",
                             reason="engel_yavasla", phase="approach")

    obstacle_avoidance = Sequence("ObstacleAvoidance", memory=False, children=[
        EngelFresh(bb, fresh["engel_max_age_s"]),
        Debounce("EngelTepkiDeb",
                 EngelMerkezBlokaj(bb, engel_yavasla_m,        # en dış: yavasla bandı
                                   gain_s=_gain("engel_block_gain_s"),
                                   max_extra_m=sa_max_extra, odom_max_age_s=odom_age),
                 bb, key="engel_blokaj",
                 min_consecutive=deb["engel_min_consecutive"]),
        Selector("EngelTepki", memory=False, children=(
            [road_aware_avoid] if lc["enabled"] else []
        ) + [
            engel_dur,
            engel_yavasla,
        ]),
    ])

    # ============================================================
    # 5b. Şerit değişimi kilidi (control.py manevra senkronu)
    #     Bir kaçış/yön-levhası şerit değişimi başlatıldıysa, control.py o
    #     manevrayı LANE_CHANGE_DURATION (~2s) boyunca kendi sürer. Bu pencerede
    #     aynı yön komutunu tutarız; aksi halde "dur"/"normal" manevrayı keser.
    #     Emergency/yaya/dur-levhası/kırmızı bu dalın ÜZERİNDE → her zaman önceliklidir.
    # ============================================================
    lane_change_hold = Sequence("LaneChangeHold", memory=False, children=[
        LaneChangeInProgress(bb, lc_hold_s),
        HoldLaneChange(bb),
    ])

    # ============================================================
    # 6. Yön levhaları (SAG/SOL)
    # ============================================================
    direction_right = Sequence("DirectionRight", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIcindeMesafe(bb, "SAG", dist["yon_levha_max_m"]),
        LaneChangeCooldownOk(bb, lc_cooldown_s),
        LaneChangeStamp(bb, "sag"),
        SetKarar("Karar=SAG(levha)", bb, karar="sag", reason="yon_levhasi_sag"),
    ])
    direction_left = Sequence("DirectionLeft", memory=False, children=[
        LevhaFresh(bb, fresh["levha_max_age_s"]),
        LevhaIcindeMesafe(bb, "SOL", dist["yon_levha_max_m"]),
        LaneChangeCooldownOk(bb, lc_cooldown_s),
        LaneChangeStamp(bb, "sol"),
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
        lane_change_hold,      # devam eden manevrayı tut (control.py senkronu)
        obstacle_avoidance,
        direction_right,
        direction_left,
        speed_limit,
        cruise,
    ])

    return root
