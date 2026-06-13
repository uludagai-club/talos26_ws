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
    # S17: Lane-change cooldown — ikinci kaçış cooldown içinde bloklanır → dur
    # -----------------------------------------------------------------
    print("\nS17: Cooldown içinde 2. engel → dur (kaçış yok)")
    bb.obs.__init__(); bb.state.__init__()
    fresh_now(bb)
    bb.obs.engel_present = True
    bb.obs.engel_d_center = 1.5
    bb.obs.engel_d_left = 5.0   # sol boş
    bb.obs.engel_d_right = 1.0
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S17a (ilk kaçış)", "sol")
    # cooldown henüz dolmadı → sol boş olsa bile kaçamaz, dur'a düşer
    for _ in range(n_engel):
        fresh_now(bb); tree.tick()
    assert_karar("S17b (cooldown blok)", "dur")

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
