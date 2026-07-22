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
    YayaVarMi, YayaCokYakin,
    EngelCokYakin, EngelMerkezBlokaj,
    LevhaIs, LevhaIcindeMesafe,
    LaneChangeCooldownOk, LaneChangeInProgress,
)
from behaviors.actions import (
    SetKarar, LatchEmergency, ReleaseEmergencyIfClear,
    DurLevhasiFSM, YayaGecidiFSM, LaneChangeStamp, RerouteKarar, HoldLaneChange,
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
    # Bant sırası (yakın → uzak):  acil < block(reroute) < yavasla
    #   acil    ≈ 1.2 m  → acil durus (emniyetli son çare; e-stop control'de de var)
    #   block   ≈ 6.0 m  → cone REROUTE tetiği (§16 E-A/E-B): RerouteKarar DUR→takip
    #                      FSM'i → önce GERÇEK 'dur' (engel_dur_reroute_pause_s;
    #                      planlayıcı replan yapsın) + hedef'e kenar_blok, sonra
    #                      'slow' ile reroute takibi. (Kullanıcı: dur→planla→devam.)
    #   yavasla ≈ 9.0 m  → en dış: tepki başlar, yavaşla (reroute talebi yok, sadece yaklaş)
    # MİMARİ (§16/§12.13): cone artık sol/sag direksiyon manevrasıyla DEĞİL,
    #   planlayıcının (hedef) rotayı dubanın etrafından çizmesiyle geçilir. Karar
    #   commit bandında 'slow' verir + cone'u kenar_blok ile hedef'e bildirir;
    #   control offset YAPMAZ (H-A kaldırıldı), yalnız rerouted rotayı takip eder.
    #   block menzili = hedef'in replan + control'ün tepki payı (6m kalibre, §12.12).
    # ===================================================================
    wp   = p.get("wp_planlama", {}) or {}
    lc   = p["lane_change"]
    dist = p["distances"]

    wp_near        = float(wp.get("control_wp_near_m",     1.5))   # = control.py WP_NEAR_DISTANCE (senkron tut!)

    engel_acil_m      = max(0.3, wp_near + float(wp.get("engel_acil_delta_m",    -0.3)))  # acil durus
    engel_dur_m       =          wp_near + float(wp.get("engel_dur_delta_m",      0.5))   # (bilgi amaçlı: DUR artık mesafe eşiğiyle değil RerouteKarar bekleme fazıyla)
    engel_block_m     =          wp_near + float(wp.get("engel_block_delta_m",    4.5))   # cone REROUTE commit (§12.12: →6.0m)
    engel_yavasla_m   =          wp_near + float(wp.get("engel_yavasla_delta_m",  7.5))   # en dış: yavasla (→9.0m)
    # NOT: kacis_deadband/varsayilan_yon/wp_hyst/engel_yan_clear_m artık kullanılmıyor
    #      (sol/sag yön seçimi §16/E-B ile kaldırıldı; cone reroute yön istemez).

    lc_cooldown_s = float(lc.get("cooldown_s",      4.0))   # ardışık şerit değişimi arası bekleme (levha SAG/SOL için)
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
        # P0 №3 (E8-R1): statik engelde mühürden 'dur'a iniş — control'ün
        # DUR-kaçışıyla (P0 №1) birlikte kilit kırıcı; params.yaml'dan kapılı.
        statik_cozme=emer.get("statik_cozme", {}),
        odom_max_age_s=odom_age,
        # P1 №7 (E5-O3): yokluk-temizliği (dropout olabilir) için ayrı uzun eşik
        release_yokluk_ticks=emer.get("release_yokluk_ticks"),
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
    # 2. Yaya geçidi FSM (acil değil) — 2026-07-22
    #    Adanmış model yalnız GEÇİT ÇİZGİSİNİ ('crosswalk') veriyor, yayayı değil.
    #    Bu yüzden geçit görülünce MİNİMAL zorunlu duruş → lidar engel ile "geçitte
    #    yaya var mı" → varsa (max'a dek) bekle, yoksa min dolunca DEVAM. Eski
    #    YayaDur/YayaSlow (mesafe eşiği) geçidin üstünde ~0.6m'de donup sonsuz 'dur'
    #    yapıyordu → FSM zaman/engel tabanlı release ile kilidi keser.
    # ============================================================
    yg = p.get("yaya_gecidi", {}) or {}
    pedestrian = Sequence("YayaGecidi", memory=False, children=[
        YayaFresh(bb, fresh["yaya_max_age_s"]),
        YayaVarMi(bb),
        YayaGecidiFSM(
            bb,
            dur_esik_m=dist["yaya_dur_m"],
            yavas_esik_m=dist["yaya_yavas_m"],
            min_bekleme_s=float(yg.get("min_bekleme_s", 3.0)),
            max_bekleme_s=float(yg.get("max_bekleme_s", 20.0)),
            engel_bekle_m=float(yg.get("engel_bekle_m", 8.0)),
            release_grace_s=float(yg.get("release_grace_s", 8.0)),
        ),
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
    # 5. Engelden kaçınma — CONE REROUTE (§16/§12.13 yeni mimari)
    #    Dış kapı: engel yavasla bandında (en geniş) → en az "yavasla".
    #    İçeride öncelik:
    #      a) commit (block) bandında → REROUTE: 'slow' + hedef'e kenar_blok
    #         (cone çok yakınsa/dur bandında RerouteKarar 'dur' güvenlik-ağına düşer)
    #      b) aksi halde (block↔yavasla arası) → yavasla (yaklaşıyor, henüz blok yok)
    #    sol/sag KALDIRILDI (E-B): cone artık control offset'iyle değil planlayıcı
    #    rerouteu ile geçiliyor → karar yalnız 'slow'/'dur'; control rerouted rotayı
    #    düz takip eder (H-A). acildurus/dur safety-net üstte+RerouteKarar içinde.
    # ============================================================
    engel_reroute_pause_s = float(timer.get("engel_dur_reroute_pause_s", 1.5))
    # TEK-SEFERLİK DUR: kısa algı boşlukları fazı sıfırlamasın (eski 0.5s → tekrarlı
    # DUR/kilit). Titreşim süresinden büyük tutulur; yapışkan kapı (hold_ticks) ile
    # birlikte kararı stabil kılar.
    engel_reroute_reset_gap_s = float(timer.get("engel_reroute_reset_gap_s", 3.0))
    road_reroute = Sequence("RoadReroute", memory=False, children=[
        EngelMerkezBlokaj(bb, engel_block_m),                 # commit (reroute) bandında mı?
        RerouteKarar(bb, pause_s=engel_reroute_pause_s,       # DUR(pause) → reroute → slow-takip
                     reset_gap_s=engel_reroute_reset_gap_s),
    ])
    engel_yavasla = SetKarar("Karar=SLOW(engel)", bb, karar="slow",
                             reason="engel_yavasla", phase="approach")

    # YAPIŞKAN ENGEL-KAPISI (çıkış histerezisi): engel bir kez angaje olunca kısa
    # detektör boşluklarında dal düşmesin → karar 'normal'e flip-flop yapmasın.
    # hold_ticks = engel_blokaj_hold_ticks (tick; 10Hz'de 15 ≈ 1.5s). 0 → eski.
    engel_hold_ticks = int(deb.get("engel_blokaj_hold_ticks", 15))
    obstacle_avoidance = Sequence("ObstacleAvoidance", memory=False, children=[
        EngelFresh(bb, fresh["engel_max_age_s"]),
        Debounce("EngelTepkiDeb",
                 EngelMerkezBlokaj(bb, engel_yavasla_m,        # en dış: yavasla bandı
                                   gain_s=_gain("engel_block_gain_s"),
                                   max_extra_m=sa_max_extra, odom_max_age_s=odom_age),
                 bb, key="engel_blokaj",
                 min_consecutive=deb["engel_min_consecutive"],
                 hold_ticks=engel_hold_ticks),
        Selector("EngelTepki", memory=False, children=[
            road_reroute,       # commit bandı → slow + kenar_blok reroute (≤dur → dur)
            engel_yavasla,      # block↔yavasla arası → slow
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
