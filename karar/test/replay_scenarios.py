"""Offline scenario harness — ROS yok, blackboard'u elle dolduruyoruz.

Çalıştır:
    cd talos26_ws/karar_bt && python3 -m test.replay_scenarios

Her senaryo: gözlem set -> tick -> beklenen karar.
Beklenenle uyuşmazsa exit 1.
"""
from __future__ import annotations

import os
import sys
import time

import yaml
import py_trees

# Paket köküne path
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from bb import Blackboard
from trees.main_tree import build_root
from obstacle_fusion import ObstacleFusionParams, fuse_obstacles


def apply_fused(bb: Blackboard, points):
    """Yeni detektör boru hattını taklit et: noktalar → füzyon → blackboard."""
    f = fuse_obstacles(points, ObstacleFusionParams())
    t = time.time()
    bb.obs.engel_present = f.present
    bb.obs.engel_d_center = f.d_center
    bb.obs.engel_d_overall = f.d_overall
    bb.obs.engel_d_left = f.d_left
    bb.obs.engel_d_right = f.d_right
    bb.obs.engel_angle_deg = f.angle_deg
    bb.obs.engel_source = "poses"
    bb.obs.engel_last_seen = t
    bb.obs.engel_left_last_seen = t
    bb.obs.engel_right_last_seen = t


def load_cfg():
    with open(os.path.join(ROOT, "config", "params.yaml")) as f:
        return yaml.safe_load(f)


def fresh_now(bb: Blackboard):
    t = time.time()
    bb.obs.yaya_last_seen = t
    bb.obs.levha_last_seen = t
    bb.obs.engel_last_seen = t
    bb.obs.engel_left_last_seen = t
    bb.obs.engel_right_last_seen = t
    bb.obs.odom_last_seen = t


def tick_n(tree, n: int):
    for _ in range(n):
        tree.tick()


def run_scenarios():
    cfg = load_cfg()
    bb = Blackboard()
    root = build_root(bb, cfg)
    tree = py_trees.trees.BehaviourTree(root)

    deb = cfg["debounce"]
    n_yaya = deb["yaya_min_consecutive"]
    n_engel = deb["engel_min_consecutive"]

    failures = []

    def assert_karar(name: str, expected: str):
        got = bb.last_decision.get("karar")
        if got != expected:
            failures.append(f"[{name}] beklenen={expected} ama={got} reason={bb.last_decision.get('reason')}")
            print(f"  ✗ {name}: beklenen={expected} ama={got}")
        else:
            print(f"  ✓ {name}: {got}  (reason: {bb.last_decision.get('reason')})")

    # -----------------------------------------------------------------
    # S1: Hiçbir şey yok → normal
    # -----------------------------------------------------------------
    print("\nS1: Boş ortam")
    bb.obs.__init__()  # reset
    bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    tick_n(tree, 1)
    assert_karar("S1", "normal")

    # -----------------------------------------------------------------
    # S2: Yaya çok yakın → acil durus (debounce sonrası)
    # -----------------------------------------------------------------
    print("\nS2: Yaya 1.5m önde → acildurus")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 1.5
    bb.obs.yaya_x = 1.5; bb.obs.yaya_y = 0.0
    for _ in range(n_yaya):
        fresh_now(bb); tree.tick()
    assert_karar("S2", "acildurus")

    # -----------------------------------------------------------------
    # S3: Yaya 3m → dur
    # -----------------------------------------------------------------
    print("\nS3: Yaya 3m")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 3.0
    for _ in range(n_yaya):
        fresh_now(bb); tree.tick()
    assert_karar("S3", "dur")

    # -----------------------------------------------------------------
    # S4: Yaya 8m → slow
    # -----------------------------------------------------------------
    print("\nS4: Yaya 8m")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 8.0
    for _ in range(n_yaya):
        fresh_now(bb); tree.tick()
    assert_karar("S4", "slow")

    # -----------------------------------------------------------------
    # S5: DUR levhası approach → slow
    # -----------------------------------------------------------------
    print("\nS5: DUR 7m → slow")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "DUR"
    bb.obs.levha_distance = 7.0
    tick_n(tree, 1)
    assert_karar("S5", "slow")

    # -----------------------------------------------------------------
    # S6: DUR levhası hold → dur
    # -----------------------------------------------------------------
    print("\nS6: DUR 2.5m → dur (hold)")
    fresh_now(bb)
    bb.obs.levha_distance = 2.5
    tick_n(tree, 1)
    assert_karar("S6", "dur")

    # -----------------------------------------------------------------
    # S7: DUR bekleme sonrası → cruise/normal (released)
    # -----------------------------------------------------------------
    print("\nS7: DUR bekleme sonrası")
    bb.state.stop_sign_hold_start_s = time.time() - (cfg["timers"]["dur_levhasi_bekleme_s"] + 0.5)
    fresh_now(bb)
    bb.obs.levha_distance = 2.5
    tick_n(tree, 1)
    assert_karar("S7", "normal")

    # -----------------------------------------------------------------
    # S8: Engel + sol boş → sol
    # -----------------------------------------------------------------
    print("\nS8: Engel merkezde, sol boş → sol")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 5.0     # sol boş
    bb.obs.engel_d_right = 1.0    # sağ dolu
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S8", "sol")

    # -----------------------------------------------------------------
    # S9: Engel + iki taraf da dolu → dur
    # -----------------------------------------------------------------
    print("\nS9: Engel merkez, sol+sağ dolu → dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 1.0
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S9", "dur")

    # -----------------------------------------------------------------
    # S10: Hız sınırı 30 → slow
    # -----------------------------------------------------------------
    print("\nS10: 30 levhası 6m")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "30"
    bb.obs.levha_distance = 6.0
    tick_n(tree, 1)
    assert_karar("S10", "slow")

    # -----------------------------------------------------------------
    # S11: SAG levhası 3m → sag
    # -----------------------------------------------------------------
    print("\nS11: SAG levhası 3m")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "SAG"
    bb.obs.levha_distance = 3.0
    tick_n(tree, 1)
    assert_karar("S11", "sag")

    # -----------------------------------------------------------------
    # S12: Stale yaya verisi (timeout) → cruise
    # -----------------------------------------------------------------
    print("\nS12: Stale yaya (eskimiş)")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 1.0
    bb.obs.yaya_last_seen = time.time() - 5.0  # 5 saniye eski
    tick_n(tree, 1)
    assert_karar("S12", "normal")

    # -----------------------------------------------------------------
    # S13: Engel acil (merkez < engel_acil_m) → acildurus
    # -----------------------------------------------------------------
    print("\nS13: Engel 0.8m merkez → acildurus")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 0.8  # engel_acil_m (1.2) altında
    bb.obs.engel_d_left = 1.0
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S13", "acildurus")

    # -----------------------------------------------------------------
    # S14: Emergency latch RELEASE — tehlike geçince mühür çözülür → normal
    # -----------------------------------------------------------------
    print("\nS14: Acil mühür sonra temiz → release")
    # S13'ün mührü hâlâ kapalı; ortamı temizle ve release_clear_ticks kadar tick'le
    n_release = cfg["emergency"]["release_clear_ticks"]
    bb.obs.engel_present = False
    bb.obs.engel_d_center = float("inf")
    bb.obs.yaya_present = False
    bb.obs.yaya_distance = -1.0
    for _ in range(n_release + 2):
        fresh_now(bb); tree.tick()
    assert_karar("S14", "normal")

    # -----------------------------------------------------------------
    # S15: Trafik ışığı KIRMIZI → dur
    # -----------------------------------------------------------------
    print("\nS15: KIRMIZI ışık 6m → dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "KIRMIZI"
    bb.obs.levha_distance = 6.0
    tick_n(tree, 1)
    assert_karar("S15", "dur")

    # -----------------------------------------------------------------
    # S16: Trafik ışığı YAVAS (sarı) → slow
    # -----------------------------------------------------------------
    print("\nS16: YAVAS ışık 6m → slow")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "YAVAS"
    bb.obs.levha_distance = 6.0
    tick_n(tree, 1)
    assert_karar("S16", "slow")

    # -----------------------------------------------------------------
    # S17: Şerit değişimi manevra kilidi — başlatılan kaçış, manevra penceresi
    #      (maneuver_hold_s) boyunca aynı yönde TUTULUR. control.py manevrayı
    #      kenar-tetiklemeli başlatıp kendi sürdüğü için BT "dur"a düşmemeli;
    #      yoksa fren manevrayı keser.
    # -----------------------------------------------------------------
    print("\nS17: Engel kaçışı başlayınca manevra penceresinde 'sol' tutulur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 5.0   # sol boş
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S17a (ilk kaçış)", "sol")
    # Manevra penceresi içinde: engel hâlâ merkezde olsa da aynı yön tutulur
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S17b (manevra kilidi)", "sol")

    # -----------------------------------------------------------------
    # S18: Yan sektör verisi bayat → kaçış yapma, dur
    # -----------------------------------------------------------------
    print("\nS18: Sol sektör bayat → kaçış yok → dur")
    bb.obs.__init__(); bb.state.__init__()
    t = time.time()
    bb.obs.engel_last_seen = t
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 5.0   # değer boş gösteriyor AMA timestamp eski
    bb.obs.engel_d_right = 5.0
    bb.obs.engel_left_last_seen = t - 5.0   # bayat
    bb.obs.engel_right_last_seen = t - 5.0  # bayat
    for _ in range(n_engel):
        bb.obs.engel_last_seen = time.time(); tree.tick()
    assert_karar("S18", "dur")

    # -----------------------------------------------------------------
    # S19: Yüksek hızda yaya 5m → hız eşiği genişler → erken DUR
    #      (taban dur eşiği 4.0m; 5m normalde 'slow' olurdu)
    # -----------------------------------------------------------------
    print("\nS19: 30km/h'de yaya 5m → erken dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.speed_kmh = 30.0          # ~8.3 m/s
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 5.0
    for _ in range(n_yaya):
        fresh_now(bb); bb.obs.speed_kmh = 30.0; tree.tick()
    assert_karar("S19", "dur")

    # -----------------------------------------------------------------
    # S20: Aynı 5m ama ODOM BAYAT → hız 0 sayılır → taban eşik → slow
    #      (güvenli fallback: hızı bilmiyorsak eşik büyütme)
    # -----------------------------------------------------------------
    print("\nS20: yaya 5m ama odom bayat → taban eşik → slow")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.yaya_present = True
    bb.obs.yaya_distance = 5.0
    bb.obs.speed_kmh = 30.0
    for _ in range(n_yaya):
        bb.obs.yaya_last_seen = time.time()        # yaya taze
        bb.obs.odom_last_seen = time.time() - 5.0  # odom bayat
        tree.tick()
    assert_karar("S20", "slow")

    # -----------------------------------------------------------------
    # S21: YENI detektör (PoseArray) — tam önde engel, sol şerit boş → sol
    # -----------------------------------------------------------------
    print("\nS21: Yeni detektör, merkez engel 1.5m, sağda engel → sol kaçış")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    # merkez engel (1.5m önde) + sağ şeritte engel (sol boş kalsın)
    for _ in range(n_engel):
        apply_fused(bb, [(1.5, 0.1), (4.0, -2.0)])
        tree.tick()
    assert_karar("S21", "sol")

    # -----------------------------------------------------------------
    # S22: YENI detektör — merkez engel, her iki şerit de dolu → dur
    # -----------------------------------------------------------------
    print("\nS22: Yeni detektör, merkez engel + iki şerit dolu → dur")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    for _ in range(n_engel):
        # yan engeller yan_clear (3m) içinde → her iki şerit kapalı
        apply_fused(bb, [(1.5, 0.1), (1.8, 1.5), (1.8, -1.5)])
        tree.tick()
    assert_karar("S22", "dur")

    # -----------------------------------------------------------------
    # S23: YENI detektör — merkez engel 0.8m (acil eşik altı) → acildurus
    # -----------------------------------------------------------------
    print("\nS23: Yeni detektör, merkez engel 0.8m → acildurus")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    for _ in range(n_engel):
        apply_fused(bb, [(0.8, 0.0)])
        tree.tick()
    assert_karar("S23", "acildurus")

    # -----------------------------------------------------------------
    # S24: Manevra penceresi DOLDUKTAN sonra engel hâlâ merkezde ve iki yan da
    #      kapalı/cooldown → artık tutma yok → dur.
    # -----------------------------------------------------------------
    print("\nS24: Manevra penceresi bitince engel sürüyor + yan kapalı → dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 1.0   # sol dolu
    bb.obs.engel_d_right = 1.0  # sağ dolu
    # Eski bir şerit değişimi başlatılmış gibi yap ama penceresi çoktan dolmuş
    hold_s = cfg["lane_change"].get("maneuver_hold_s", 2.0)
    bb.state.lane_change_dir = "sol"
    bb.state.last_lane_change_s = time.time() - (hold_s + 1.0)
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S24", "dur")

    # -----------------------------------------------------------------
    # S25: Yön levhası SAG — ilk tick "sag", sonraki tick'te de "sag" tutulur
    #      (eski hata: 2. tick'te cooldown nedeniyle "normal"e düşüp control.py'de
    #       manevrayı iptal ediyordu).
    # -----------------------------------------------------------------
    print("\nS25: SAG levhası — manevra penceresinde 'sag' tutulur ('normal' değil)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "SAG"
    bb.obs.levha_distance = 3.0
    tick_n(tree, 1)
    assert_karar("S25a (ilk)", "sag")
    # Sonraki tick: levha hâlâ görünür; cooldown başladı ama manevra kilidi tutar
    fresh_now(bb)
    bb.obs.levha_isim = "SAG"
    bb.obs.levha_distance = 3.0
    tick_n(tree, 1)
    assert_karar("S25b (manevra kilidi)", "sag")

    # -----------------------------------------------------------------
    # S26: DUR levhası release_grace — bekleme bitip release olduktan sonra,
    #      levha kısa süre sonra tekrar yakın görünse bile İKİNCİ duruş tetiklenmez
    #      (grace içinde). Grace dolunca yeniden tetiklenir.
    # -----------------------------------------------------------------
    print("\nS26: DUR release_grace — çift duruş engellenir, grace sonrası tekrar dur")
    grace_s = cfg["timers"].get("dur_levhasi_release_grace_s", 1.5)
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.levha_isim = "DUR"
    bb.obs.levha_distance = 2.5  # < stop_esik → holding
    # Beklemeyi çoktan tamamlanmış say → bu tick release olur
    bb.state.stop_sign_phase = "holding"
    bb.state.stop_sign_hold_start_s = time.time() - (cfg["timers"]["dur_levhasi_bekleme_s"] + 0.5)
    tick_n(tree, 1)
    assert_karar("S26a (release)", "normal")
    # Levha görüşten çıkıp idle'a sıfırlansın
    fresh_now(bb); bb.obs.levha_isim = "NONE"; bb.obs.levha_distance = -1.0
    tick_n(tree, 1)
    # Levha grace içinde tekrar yakın görünür → çift duruş YOK → normal
    fresh_now(bb); bb.obs.levha_isim = "DUR"; bb.obs.levha_distance = 2.5
    tick_n(tree, 1)
    assert_karar("S26b (grace içinde, çift duruş yok)", "normal")
    # Grace dolduktan sonra aynı levha → yeniden duruş tetiklenir
    bb.state.stop_sign_released_s = time.time() - (grace_s + 0.5)
    fresh_now(bb); bb.obs.levha_isim = "DUR"; bb.obs.levha_distance = 2.5
    tick_n(tree, 1)
    assert_karar("S26c (grace sonrası tekrar dur)", "dur")

    # -----------------------------------------------------------------
    # S27: Sarı ışık aksiyonu paramı — yellow_action="dur" iken YAVAS → dur
    #      (varsayılan "slow" davranışı S16'da doğrulanıyor).
    # -----------------------------------------------------------------
    print("\nS27: traffic_light.yellow_action='dur' → YAVAS ışık → dur")
    cfg_dur = load_cfg()
    cfg_dur.setdefault("traffic_light", {})["yellow_action"] = "dur"
    bb2 = Blackboard()
    tree2 = py_trees.trees.BehaviourTree(build_root(bb2, cfg_dur))
    fresh_now(bb2)
    bb2.obs.levha_isim = "YAVAS"
    bb2.obs.levha_distance = 6.0
    tree2.tick()
    got = bb2.last_decision.get("karar")
    if got != "dur":
        failures.append(f"[S27] beklenen=dur ama={got}")
        print(f"  ✗ S27: beklenen=dur ama={got}")
    else:
        print(f"  ✓ S27: {got}  (reason: {bb2.last_decision.get('reason')})")

    # -----------------------------------------------------------------
    # S28: YOL-BİLİNÇLİ — engel rotanın SAĞINDA → SOLA kaç (rota kaynaklı)
    #      Robot (0,0) yaw=0, rota +x (hedef 5,0 → next 10,0). Engel gövde
    #      çerçevesinde sağda (açı +18.4°, ~3.16m) → dünya (3,-1) = rota sağı.
    # -----------------------------------------------------------------
    print("\nS28: Engel rotanın sağında → SOL (yol-bilinçli)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.16
    bb.obs.engel_d_overall = 3.16
    bb.obs.engel_angle_deg = 18.4         # sağ-pozitif → engel sağda
    bb.obs.engel_d_left = float("inf")    # sol şerit boş
    bb.obs.engel_d_right = float("inf")
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S28", "sol")
    if "rota" not in bb.last_decision.get("reason", ""):
        failures.append(f"[S28] kaynak 'rota' bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S28 kaynak: {bb.last_decision.get('reason')}")

    # -----------------------------------------------------------------
    # S29: YOL-BİLİNÇLİ — engel rotanın SOLUNDA → SAĞA kaç
    # -----------------------------------------------------------------
    print("\nS29: Engel rotanın solunda → SAG (yol-bilinçli)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.16
    bb.obs.engel_d_overall = 3.16
    bb.obs.engel_angle_deg = -18.4        # sol → engel solda
    bb.obs.engel_d_left = float("inf")
    bb.obs.engel_d_right = float("inf")
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S29", "sag")

    # -----------------------------------------------------------------
    # S30: Yol sol diyor ama SOL ŞERİT KAPALI → kaçış yok → dur (2.0m içinde)
    # -----------------------------------------------------------------
    print("\nS30: Yol-bilinçli sol ama sol kapalı → dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.8            # dur bandında (< 2.0)
    bb.obs.engel_d_overall = 1.8
    bb.obs.engel_angle_deg = 18.4         # sağda → yol "sol" der
    bb.obs.engel_d_left = 0.5             # ama sol kapalı
    bb.obs.engel_d_right = 0.5
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S30", "dur")

    # -----------------------------------------------------------------
    # S31: KATMANLI — engel yavasla bandında (5m, block 3.5'in dışında) → yavasla
    # -----------------------------------------------------------------
    print("\nS31: Engel 5m (yavasla bandı, kaçışa daha var) → slow")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 5.0
    bb.obs.engel_d_overall = 5.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S31", "slow")

    # -----------------------------------------------------------------
    # S32: Engel yavasla bandının DIŞINDA (7m > 6m) → normal (over-trigger yok)
    # -----------------------------------------------------------------
    print("\nS32: Engel 7m (banttan uzak) → normal")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 7.0
    bb.obs.engel_d_overall = 7.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S32", "normal")

    # -----------------------------------------------------------------
    # S33: MERKEZİ koni (rota üzerinde, deadband içinde) + rota taze → geçiş
    #      şeridine (varsayilan=sol). CANLI BUG regresyonu (2026-06-24): engel
    #      lateral ~0.2m iken eski kod yan_sektor gürültüsünden "sag" seçip
    #      aracı koniye sokuyordu. Artık varsayilan_kacis_yon=sol gelmeli.
    # -----------------------------------------------------------------
    print("\nS33: Merkezi koni + rota taze → SOL (varsayilan, yan_sektor değil)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.0
    bb.obs.engel_d_overall = 3.0
    bb.obs.engel_angle_deg = 4.0          # ~merkez (lateral ~0.2m, deadband içinde)
    bb.obs.engel_d_left = 4.0             # iki yan da clear (>3m) → seçilen taraf commit eder
    bb.obs.engel_d_right = 5.0            # TUZAK: sağ daha açık → eski yan_sektor kodu "sag" derdi
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S33", "sol")
    if "rota_merkez" not in bb.last_decision.get("reason", ""):
        failures.append(f"[S33] kaynak 'rota_merkez' bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S33 kaynak: {bb.last_decision.get('reason')}")

    print("\n" + "=" * 50)
    if failures:
        print(f"FAIL: {len(failures)} senaryo başarısız")
        for f in failures:
            print(" -", f)
        return 1
    print(f"OK: tüm senaryolar geçti")
    return 0


if __name__ == "__main__":
    sys.exit(run_scenarios())
