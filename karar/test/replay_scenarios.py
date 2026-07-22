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


def mirror_bridge_derived(bb: Blackboard):
    """ros_bridge'in TÜRETTİĞİ alanları harness'ta doldur (her tick'ten önce).

    Senaryolar ros_bridge'i bypass edip blackboard'a doğrudan yazıyor; köprünün
    kendi hesaplayıp yazdığı alanlar burada taklit edilmezse senaryo varsayılan
    değerle (inf) çalışır ve SESSİZCE yanlış dalı doğrular — fail-safe testi
    susturur, kırmızı yanmaz. (2026-07-15 yay-kapısı: engel_d_arc doldurulmadığı
    için S13/S23 'acildurus' yerine 'dur' aldı; acil dalı hiç açılmıyordu.)

    engel_d_arc: offline harness'ta direksiyon kaynağı (/cart) YOK → ros_bridge'in
    fail-safe yolu geçerlidir: d_arc = d_center (düz-koridor davranışı). Yay-kapısı
    geometrisinin kendi testi test_yay_kapisi.py'dedir; burada amaç ağacın acil
    dalının d_center senaryolarıyla test edilebilir kalması.

    YENİ TÜRETİLMİŞ ALAN EKLERKEN: ros_bridge'e alan eklendiğinde buraya da ekle.
    """
    bb.obs.engel_d_arc = bb.obs.engel_d_center


def run_scenarios():
    cfg = load_cfg()
    bb = Blackboard()
    root = build_root(bb, cfg)
    tree = py_trees.trees.BehaviourTree(root)
    # Köprü aynası TEK noktadan: her tick'ten önce çalışır → senaryoların
    # engel alanını nasıl yazdığından (elle ya da apply_fused) bağımsız.
    tree.add_pre_tick_handler(lambda _t: mirror_bridge_derived(bb))

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

    def assert_reroute(name, cone_expected=True):
        """E-A: bloklu cone reroute_request + dünya konumu set edilmeli (kenar_blok hedefi)."""
        if bb.state.reroute_request != cone_expected:
            failures.append(f"[{name}] reroute_request={bb.state.reroute_request} (beklenen {cone_expected})")
            print(f"  ✗ {name}: reroute_request={bb.state.reroute_request}")
            return
        if cone_expected:
            cx, cy = bb.state.reroute_cone_world
            if abs(cx) < 1e-6 and abs(cy) < 1e-6:
                failures.append(f"[{name}] cone dünya konumu (0,0) — kenar_blok hedefi yok")
                print(f"  ✗ {name}: cone konumu (0,0)")
            else:
                print(f"  ✓ {name}: reroute_request + cone dünya=({cx:.2f},{cy:.2f})")

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
    # S8: Cone merkezde, dur bandında (1.5m < 2.0) → DUR + reroute
    #     §16/E-B: "sol boş → sol" kaçışı KALDIRILDI. cone rotayla (hedef reroute)
    #     geçilir; yan sektör boşluğu artık kararı etkilemez; ≤2m'de güvenlik-ağı dur.
    # -----------------------------------------------------------------
    print("\nS8: Cone merkez 1.5m (dur bandı) → DUR + reroute")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 5.0     # (artık karar etkilemiyor)
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S8", "dur")
    assert_reroute("S8")

    # -----------------------------------------------------------------
    # S9: Cone merkez dur bandında (1.5m) → dur + reroute (yan sektör artık etkisiz)
    # -----------------------------------------------------------------
    print("\nS9: Cone merkez 1.5m → dur + reroute")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 1.0
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S9", "dur")
    assert_reroute("S9")

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
    # S13'ün mührü hâlâ kapalı; ortamı temizle. Temizlik YOKLUK yoluyla
    # (engel_present=0) geldiğinden P1 №7 gereği uzun eşik geçerli:
    # release_yokluk_ticks (20) — dropout'un mührü erken çözmesi kapatıldı.
    n_release = cfg["emergency"]["release_clear_ticks"]
    n_yokluk = int(cfg["emergency"].get("release_yokluk_ticks", n_release))
    bb.obs.engel_present = False
    bb.obs.engel_d_center = float("inf")
    bb.obs.yaya_present = False
    bb.obs.yaya_distance = -1.0
    # Eski (ölçülü) eşik kadar tick'te HÂLÂ mühürlü olmalı (yokluk ≠ ölçülü kanıt)
    for _ in range(n_release + 2):
        fresh_now(bb); tree.tick()
    assert_karar("S14-erken", "acildurus")
    # Yokluk eşiği dolunca çözülür
    for _ in range(n_yokluk):
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
    # S17: DUR→REROUTE→DEVAM — cone commit bandına girince önce GERÇEK 'dur'
    #      (pause_s, planlayıcı replan yapsın), bekleme dolunca 'slow' ile reroute
    #      takibi. reroute_request her iki fazda da yenilenir (kenar_blok refresh).
    # -----------------------------------------------------------------
    print("\nS17: Cone commit bandı → DUR(bekleme) sonra SLOW(reroute takibi)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.5   # commit bandı (>dur 2.0, <block 6.0)
    bb.obs.engel_d_overall = 3.5
    bb.obs.engel_d_left = 5.0
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S17a (ilk giriş → DUR)", "dur")
    assert_reroute("S17a")
    # Bekleme süresi dolmuş gibi yap → FOLLOW fazı: 'slow' ile reroute takibi
    bb.state.reroute_stop_start_s = time.time() - (cfg["timers"]["engel_dur_reroute_pause_s"] + 0.5)
    fresh_now(bb); tree.tick()
    assert_karar("S17b (bekleme doldu → SLOW takip)", "slow")
    assert_reroute("S17b")

    # -----------------------------------------------------------------
    # S18: Cone dur bandında (1.5m) → dur + reroute. (Eskiden yan-sektör tazelik
    #      kapısını test ederdi; §16/E-B ile yan sektör kararı etkilemiyor.)
    # -----------------------------------------------------------------
    print("\nS18: Cone 1.5m → dur + reroute (yan-sektör tazeliği artık etkisiz)")
    bb.obs.__init__(); bb.state.__init__()
    t = time.time()
    bb.obs.engel_last_seen = t
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_overall = 1.5
    bb.obs.engel_d_left = 5.0
    bb.obs.engel_d_right = 5.0
    bb.obs.engel_left_last_seen = t - 5.0   # bayat (artık önemsiz)
    bb.obs.engel_right_last_seen = t - 5.0
    for _ in range(n_engel):
        bb.obs.engel_last_seen = time.time(); tree.tick()
    assert_karar("S18", "dur")
    assert_reroute("S18")

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
    # S21: YENI detektör (PoseArray) — cone commit bandına ilk giriş → DUR + reroute
    #      (bekleme fazı; sonrası SLOW-takip S17'de doğrulandı). §16/E-B: yeni
    #      detektör de aynı DUR→reroute yoluna girer (yön seçimi yok).
    # -----------------------------------------------------------------
    print("\nS21: Yeni detektör, merkez cone 3.5m → DUR(giriş) + reroute")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    for _ in range(n_engel):
        apply_fused(bb, [(3.5, 0.1), (5.0, -2.0)])
        tree.tick()
    assert_karar("S21", "dur")
    assert_reroute("S21")

    # -----------------------------------------------------------------
    # S22: YENI detektör — cone dur bandında (1.5m) → dur + reroute
    #      (yan engeller artık kararı etkilemez; ≤2m güvenlik-ağı dur.)
    # -----------------------------------------------------------------
    print("\nS22: Yeni detektör, merkez cone 1.5m → dur + reroute")
    bb.obs.__init__(); bb.state.__init__()
    bb.obs.odom_last_seen = time.time()
    for _ in range(n_engel):
        apply_fused(bb, [(1.5, 0.1), (1.8, 1.5), (1.8, -1.5)])
        tree.tick()
    assert_karar("S22", "dur")
    assert_reroute("S22")

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
    # S24: Levha şerit-değişimi penceresi DOLDUKTAN sonra (lane_change_hold artık
    #      tutmaz) cone dur bandında → dur + reroute. lane_change_hold yalnız
    #      LEVHA SAG/SOL içindir (S25); süresi geçince engel kararına düşülür.
    # -----------------------------------------------------------------
    print("\nS24: Levha manevra penceresi bitti + cone 1.5m → dur + reroute")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 1.0
    bb.obs.engel_d_right = 1.0
    # Eski bir levha şerit değişimi başlatılmış gibi yap ama penceresi çoktan dolmuş
    hold_s = cfg["lane_change"].get("maneuver_hold_s", 2.0)
    bb.state.lane_change_dir = "sol"
    bb.state.last_lane_change_s = time.time() - (hold_s + 1.0)
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S24", "dur")
    assert_reroute("S24")

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
    # S28: CONE REROUTE — engel commit bandına ilk giriş (3.16m) → DUR + kenar_blok
    #      §16/E-B: sol/sag KALDIRILDI; cone rotayla (hedef reroute) geçilir. Karar
    #      önce DUR verir (replan) + cone'u DÜNYA frame'de kenar_blok ile bildirir.
    # -----------------------------------------------------------------
    print("\nS28: Engel commit bandına giriş → DUR + reroute (kenar_blok)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.16
    bb.obs.engel_d_overall = 3.16
    bb.obs.engel_angle_deg = 18.4         # konum hesabı için (yön seçimi YOK artık)
    bb.obs.engel_d_left = float("inf")
    bb.obs.engel_d_right = float("inf")
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S28", "dur")
    assert_reroute("S28")
    if "reroute" not in bb.last_decision.get("reason", ""):
        failures.append(f"[S28] reason *reroute* bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S28 reason: {bb.last_decision.get('reason')}")

    # -----------------------------------------------------------------
    # S29: CONE REROUTE — engel solda da olsa davranış AYNI (yön seçimi yok) → SLOW
    # -----------------------------------------------------------------
    print("\nS29: Engel solda → yine DUR(giriş) + reroute (yön seçimi yok)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.16
    bb.obs.engel_d_overall = 3.16
    bb.obs.engel_angle_deg = -18.4
    bb.obs.engel_d_left = float("inf")
    bb.obs.engel_d_right = float("inf")
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S29", "dur")
    assert_reroute("S29")

    # -----------------------------------------------------------------
    # S30: GÜVENLİK AĞI — cone dur bandında (1.8m < 2.0) → reroute saptıramadı → DUR
    #      (blok talebi KORUNUR: reroute_request hâlâ True, cone hâlâ orada).
    # -----------------------------------------------------------------
    print("\nS30: Cone dur bandında (1.8m) → DUR (reroute güvenlik ağı), blok korunur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.8            # dur bandında (< 2.0)
    bb.obs.engel_d_overall = 1.8
    bb.obs.engel_angle_deg = 18.4
    bb.obs.engel_d_left = 0.5
    bb.obs.engel_d_right = 0.5
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S30", "dur")
    assert_reroute("S30")   # dur'da bile blok talebi sürmeli (cone hâlâ önde)
    if "reroute" not in bb.last_decision.get("reason", ""):
        failures.append(f"[S30] reason engel_blokaj_reroute bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S30 reason: {bb.last_decision.get('reason')}")

    # -----------------------------------------------------------------
    # S31: KATMANLI — engel yavasla bandında (7.5m, block 6.0'ın dışında) → yavasla
    #      §12.12: block(commit) 3.5→6.0, yavasla 6.0→9.0. 7.5m commit'in dışı,
    #      yavasla'nın içi → slow (kaçışa commit etmeden yaklaş).
    # -----------------------------------------------------------------
    print("\nS31: Engel 7.5m (yavasla bandı, kaçışa daha var) → slow")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 7.5
    bb.obs.engel_d_overall = 7.5
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S31", "slow")

    # -----------------------------------------------------------------
    # S32: Engel yavasla bandının DIŞINDA (11m > 9m) → normal (over-trigger yok)
    #      §12.12: yavasla 6.0→9.0; 11m hâlâ bandın dışında olmalı.
    # -----------------------------------------------------------------
    print("\nS32: Engel 11m (banttan uzak) → normal")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 11.0
    bb.obs.engel_d_overall = 11.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S32", "normal")

    # -----------------------------------------------------------------
    # S33: MERKEZİ koni (rota üzerinde, commit bandında) → DUR(giriş) + reroute.
    #      §16/E-B: artık yön seçimi (sol/sag) YOK; merkezi koni de rotayla
    #      (hedef reroute) geçilir. d_left/d_right artık karar etkilemiyor.
    # -----------------------------------------------------------------
    print("\nS33: Merkezi koni + rota taze → DUR(giriş) + reroute (yön seçimi yok)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.next_hedef_x = 10.0; bb.obs.next_hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.0
    bb.obs.engel_d_overall = 3.0
    bb.obs.engel_angle_deg = 4.0          # ~merkez (konum hesabı için)
    bb.obs.engel_d_left = 4.0
    bb.obs.engel_d_right = 5.0
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S33", "dur")
    assert_reroute("S33")
    if "reroute" not in bb.last_decision.get("reason", ""):
        failures.append(f"[S33] reason *reroute* bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S33 reason: {bb.last_decision.get('reason')}")

    # -----------------------------------------------------------------
    # S34: MÜHÜR STATİK-İNİŞ (P0 №3, inceleme 2026-07-16 E8-R1) — statik
    #      yakın engelde bırakma eşiği (1.8m) sağlanamaz; mühür ≥15s +
    #      hareketsiz + d_arc sabit + taban (1.0m) üstü → karar 'dur'a iner
    #      (reason muhur_statik_dur), mühür AÇIK KALIR (yeniden-mühür yok).
    #      Taban ALTINDA (0.8m) iniş YOK → acildurus sürer.
    # -----------------------------------------------------------------
    print("\nS34: Mühür statik-iniş → dur (mühür açık kalır)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    sc34 = cfg["emergency"]["statik_cozme"]
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 0.8      # acil eşiği (1.2) altı → mühür kurulur
    bb.obs.speed_kmh = 0.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S34-mühür", "acildurus")
    # Statik faz: engel 1.15m'de SABİT (release 1.8'in altı, taban 1.0'ın üstü);
    # mühür yaşı geriye damgalanır (testte min_muhur_s beklememek için).
    bb.obs.engel_d_center = 1.15
    bb.state.emergency_latch_start_s = time.time() - (float(sc34["min_muhur_s"]) + 5.0)
    for _ in range(int(sc34["d_arc_sabit_ticks"]) + 2):
        fresh_now(bb); tree.tick()
    assert_karar("S34-iniş", "dur")
    if bb.last_decision.get("reason") != "muhur_statik_dur":
        failures.append(f"[S34] reason muhur_statik_dur bekleniyordu: {bb.last_decision.get('reason')}")
        print(f"  ✗ S34 reason: {bb.last_decision.get('reason')}")
    if not bb.state.emergency_latched:
        failures.append("[S34] mühür ÇÖZÜLMEMELİYDİ (iniş ≠ release; anında yeniden-mühür riski)")
        print("  ✗ S34: mühür çözülmüş")
    else:
        print("  ✓ S34: mühür açık kaldı (iniş release değil)")
    # Taban altı: 0.8m < d_arc_min_m → iniş yok, acildurus sürmeli
    bb.obs.engel_d_center = 0.8
    for _ in range(int(sc34["d_arc_sabit_ticks"]) + 2):
        fresh_now(bb); tree.tick()
    assert_karar("S34-taban", "acildurus")

    # -----------------------------------------------------------------
    # S35: STATİK-İNİŞ + ALGI FLICKER'I (canlı doğrulama 2026-07-17):
    #      detektör her 5. tick'te kareyi düşürüyor (present=0, d=inf) —
    #      E3'ün 1-2 Hz tek-tick dropout deseni. Dropout sabitlik sayacını
    #      SIFIRLAMAMALI; iniş yine gerçekleşmeli. (Mühür de çözülmemeli:
    #      release 8 ARDIŞIK temiz tick ister, flicker 4'te bir kesiyor.)
    # -----------------------------------------------------------------
    print("\nS35: Statik-iniş algı flicker'ı altında → yine dur")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 0.8
    bb.obs.speed_kmh = 0.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S35-mühür", "acildurus")
    bb.state.emergency_latch_start_s = time.time() - (float(sc34["min_muhur_s"]) + 5.0)
    for i in range(int(sc34["d_arc_sabit_ticks"]) * 2 + 5):
        if i % 5 == 4:   # her 5. tick dropout
            bb.obs.engel_present = False
            bb.obs.engel_d_center = float("inf")
        else:
            bb.obs.engel_present = True
            bb.obs.engel_d_center = 1.15
        fresh_now(bb); tree.tick()
    # Son tick'i finite d ile bitir (iniş o tick'te değerlendirilir)
    bb.obs.engel_present = True; bb.obs.engel_d_center = 1.15
    fresh_now(bb); tree.tick()
    assert_karar("S35-iniş", "dur")
    if not bb.state.emergency_latched:
        failures.append("[S35] flicker mührü çözmemeliydi (release 8 ardışık temiz tick ister)")
        print("  ✗ S35: mühür çözülmüş")
    else:
        print("  ✓ S35: mühür açık kaldı, dropout'lar inişi engellemedi")

    # -----------------------------------------------------------------
    # S36: TAM AKIŞ — engel gelir → DUR(bekleme) → SLOW(reroute takibi) →
    #      engel banttan çıkar → NORMAL. Kullanıcı gereksinimi: dur → yeniden
    #      planla → devam. (reset_gap: engel gidince faz sıfırlanır.)
    # -----------------------------------------------------------------
    print("\nS36: Engel → DUR → reroute takibi(SLOW) → engel temiz → NORMAL")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.x = 0.0; bb.obs.y = 0.0; bb.obs.yaw = 0.0
    bb.obs.hedef_x = 5.0; bb.obs.hedef_y = 0.0
    bb.obs.hedef_last_seen = time.time()
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.5
    bb.obs.engel_d_overall = 3.5
    bb.obs.engel_angle_deg = 5.0
    # 1) İlk giriş → DUR
    for _ in range(n_engel):
        fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S36a (engel geldi → DUR)", "dur")
    assert_reroute("S36a")
    # 2) Bekleme dolar → SLOW (reroute takibi)
    bb.state.reroute_stop_start_s = time.time() - (cfg["timers"]["engel_dur_reroute_pause_s"] + 0.5)
    fresh_now(bb); bb.obs.hedef_last_seen = time.time(); tree.tick()
    assert_karar("S36b (bekleme doldu → SLOW takip)", "slow")
    # 3) Engel banttan çıkar (rerouteu takip edip geçtik) → NORMAL.
    #    YAPIŞKAN KAPI: temizlikten sonra karar hold_ticks (~1.5s) 'slow' TUTAR,
    #    sonra cikis-debounce (slow→normal) tamamlanınca 'normal'. Bu gecikme
    #    kasıtlı (flicker→normal flip-flop'unu keser). Hold + cikis + pay kadar
    #    tikleyip nihai 'normal'i doğrula.
    bb.obs.engel_present = False
    bb.obs.engel_d_center = float("inf")
    bb.obs.engel_d_overall = float("inf")
    hold_ticks = int(cfg["debounce"].get("engel_blokaj_hold_ticks", 15))
    cikis = int(cfg["debounce"].get("cikis_debounce_ticks", 3))
    for _ in range(hold_ticks + cikis + 3):
        fresh_now(bb); tree.tick()
    assert_karar("S36c (engel temiz → NORMAL)", "normal")

    # -----------------------------------------------------------------
    # S37: FLICKER — engel angaje (SLOW takip) iken 1 tick detektör boşluğu
    #      (engel_present=0). Yapışkan kapı sayesinde karar 'normal'e DÜŞMEZ;
    #      'slow' korunur. (2026-07-22 karar-kararsızlığı fix: normal↔slow flip yok.)
    # -----------------------------------------------------------------
    print("\nS37: SLOW takip iken 1-tick engel boşluğu → 'normal'e düşme (yapışkan kapı)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.5
    bb.obs.engel_d_overall = 3.5
    bb.obs.engel_angle_deg = 5.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    # bekleme dolur → SLOW takip
    bb.state.reroute_stop_start_s = time.time() - (cfg["timers"]["engel_dur_reroute_pause_s"] + 0.5)
    fresh_now(bb); tree.tick()
    assert_karar("S37a (angaje → SLOW)", "slow")
    # 1-tick boşluk: engel yok ama pose/tazelik taze (detektör titremesi)
    bb.obs.engel_present = False
    bb.obs.engel_d_center = float("inf")
    bb.obs.engel_d_overall = float("inf")
    fresh_now(bb); tree.tick()
    assert_karar("S37b (1-tick boşluk → hâlâ SLOW, normal DEĞİL)", "slow")

    # -----------------------------------------------------------------
    # S38: TEK-SEFERLİK DUR — engel angaje + DUR yapıldı; kısa (<reset_gap) boşluk
    #      sonrası engel geri gelince YENİDEN DUR YAPMAZ, doğrudan 'slow' takip.
    #      (canlı 160358Z: 382 tekrarlı engel_dur_reroute kilidi bu senaryo.)
    # -----------------------------------------------------------------
    print("\nS38: DUR sonrası kısa boşluk → engel dönünce YENİDEN DUR YOK (tek-seferlik)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.5
    bb.obs.engel_d_overall = 3.5
    bb.obs.engel_angle_deg = 5.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S38a (ilk giriş → DUR)", "dur")
    # bekleme dolar → SLOW (follow fazına geç)
    bb.state.reroute_stop_start_s = time.time() - (cfg["timers"]["engel_dur_reroute_pause_s"] + 0.5)
    fresh_now(bb); tree.tick()
    assert_karar("S38b (bekleme doldu → SLOW)", "slow")
    # kısa boşluk (reset_gap içinde): engel kaybol, birkaç tick (yapışkan pencere içinde)
    bb.obs.engel_present = False
    bb.obs.engel_d_center = float("inf")
    bb.obs.engel_d_overall = float("inf")
    for _ in range(2):
        fresh_now(bb); tree.tick()
    # engel geri gelir → faz KORUNDU (reset_gap büyük) → FOLLOW ('slow'), DUR DEĞİL
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 3.5
    bb.obs.engel_d_overall = 3.5
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S38c (engel döndü → SLOW takip, YENİDEN DUR YOK)", "slow")

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
