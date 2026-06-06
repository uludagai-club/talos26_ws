#!/usr/bin/env python3
"""
waypoint_dual_lane_generator — TALOS sim için waypoint graf düzeltici.

SADELEŞTİRİLMİŞ scope (kullanıcı geri bildirimi sonrası):
  * Mevcut 61 node KONUMU AYNEN kalır (lane_detector runtime'da fine-tune ediyor)
  * 6 göbek-içi yanlış diyagonal (25-38, 25-41, 37-25, 38-37, 38-41, 41-37) düşer
  * Park topolojisi: 53-60 (eski spot) → FRONT (doğuya kayar), eski pozisyonlara
    yeni SPOT'lar konur, her front[i] sadece spot[i] ile bağlı
  * Görev koordinatları en yakın grafnoduna snap edilir (gorev_snapped.geojson)
  * Çıkış: yaml (waypoint_pub.py uyumlu) + json (meta) + preview PNG

Çift hat / spline / densify M3'e ertelendi.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple

REPO_ROOT  = os.path.expanduser("~/talos-sim")
GRAPH_JSON = f"{REPO_ROOT}/scripts/talos26_ws/missions/current_graph.json"
GOREV_JSON = f"{REPO_ROOT}/scripts/talos26_ws/missions/gorev.geojson"
TRACK_IMG  = f"{REPO_ROOT}/scripts/talos26_ws/waypoint-editor/data/track_layout.jpg"
TRACK_CALIB = f"{REPO_ROOT}/scripts/talos26_ws/waypoint-editor/track_calibration.json"

OUT_DIR    = f"{REPO_ROOT}/scripts/talos26_ws/missions"
OUT_YAML   = f"{OUT_DIR}/dual_lane_graph.yaml"
OUT_JSON   = f"{OUT_DIR}/dual_lane_graph.json"
OUT_PNG    = f"{REPO_ROOT}/ktr_gorseller/dual_lane_preview.png"
OUT_GOREV  = f"{OUT_DIR}/gorev_snapped.geojson"

# Göbek içinden geçen kötü diyagonaller (sıra önemsiz, undirected)
BAD_GOBEK_DIAGONALS: Set[frozenset] = {
    frozenset({25, 38}), frozenset({25, 41}), frozenset({37, 25}),
    frozenset({38, 37}), frozenset({38, 41}), frozenset({41, 37}),
}

# Park parametreleri
PARK_SPOT_IDS_OLD = list(range(53, 61))     # mevcut 53-60 = spotlar olacak
PARK_FRONT_DX = 2.5                          # m — spotun doğusuna yeni front nodu
PARK_FRONT_CHAIN_TO_MAIN_MAX_DIST = 8.0      # m — main road bağlantısı için max mesafe

GOREV_SNAP_MAX = 6.0   # m — bundan uzak snap "uyarı"


# ============================================================
@dataclass
class Graph:
    nodes: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    edges: List[Tuple[int, int]] = field(default_factory=list)


def load_current_graph(path: str = GRAPH_JSON) -> Graph:
    raw = json.load(open(path))
    return Graph(
        nodes={n["id"]: (n["x"], n["y"]) for n in raw["nodes"]},
        edges=[tuple(e) for e in raw["edges"]],
    )


# ============================================================
def build_fixed_graph() -> dict:
    g = load_current_graph()
    print(f"[1] mevcut graf: {len(g.nodes)} node, {len(g.edges)} edge")

    # ---- 1) Göbek diyagonallerini at ----
    kept_edges = []
    dropped = []
    for (u, v) in g.edges:
        if frozenset({u, v}) in BAD_GOBEK_DIAGONALS:
            dropped.append((u, v))
            continue
        kept_edges.append((u, v))
    print(f"[2] göbek diyagonali drop: {dropped} → {len(kept_edges)} edge kaldı")

    # ---- 2) Park topolojisini düzelt ----
    # Eski 53-60 spot pozisyonlarını HATIRLA
    spot_old_positions = {i: g.nodes[i] for i in PARK_SPOT_IDS_OLD if i in g.nodes}
    print(f"[3] eski park node'ları (yeni spot olacak): {list(spot_old_positions.keys())}")

    # ESKI 53-60'ı YENI front pozisyonuna kaydır (doğuya PARK_FRONT_DX)
    # Bu YENİ konumda ID'leri korunur, sadece x'leri kayar.
    for i, (x, y) in list(spot_old_positions.items()):
        g.nodes[i] = (x + PARK_FRONT_DX, y)
    print(f"[3a] eski 53-60 ID'leri korunur, pozisyon doğuya +{PARK_FRONT_DX}m kaydırıldı (FRONT rolünde)")

    # Spotlar için YENİ ID'ler (mevcut max ID'den sonra)
    max_id = max(g.nodes)
    spot_new_ids: Dict[int, int] = {}    # old_id (53-60) → new_spot_id
    next_id = max_id + 1
    for old_id, (orig_x, orig_y) in spot_old_positions.items():
        spot_new_ids[old_id] = next_id
        g.nodes[next_id] = (orig_x, orig_y)
        next_id += 1
    print(f"[3b] yeni spot ID'leri: {spot_new_ids}")

    # Eski 53-60 arasındaki zincir kenarları (53-54, 54-55, ...) FRONT-FRONT zinciri olarak kalır
    # Spotlara dik bağlantılar EKLE: front[i] (= old 53-60) ↔ spot[i] (= new id)
    park_dik_edges = []
    for old_front_id, new_spot_id in spot_new_ids.items():
        kept_edges.append((old_front_id, new_spot_id))
        park_dik_edges.append((old_front_id, new_spot_id))
    print(f"[3c] dik bağlantılar (front↔spot): {len(park_dik_edges)}")

    # 23 ↔ 53 eski kenarı (16m) tutmaya değer mi? Aslında front zincirinin BAŞINA
    # main road'dan giriş olmalı. 23→eski 53 (= yeni front 53) artık 12.4m olur (eskisinden kısa).
    # Bu zaten kept_edges içinde (23, 53) olarak kalmış olabilir. Kontrol:
    have_23_53 = any(frozenset({23, 53}) == frozenset({u, v}) for u, v in kept_edges)
    if have_23_53:
        # 23 (-4.83, -14.42) → yeni 53 (-18.4, -13.49). Dist = 13.6m, hâlâ uzun ama mantıklı
        d = math.hypot(g.nodes[23][0] - g.nodes[53][0], g.nodes[23][1] - g.nodes[53][1])
        print(f"[3d] 23→53 (main→front) eski kenarı: {d:.2f}m korundu")
    else:
        # Eklenmeli
        kept_edges.append((23, 53))
        print(f"[3d] 23→53 main giriş kenarı eklendi")

    print(f"[4] son graf: {len(g.nodes)} node, {len(kept_edges)} edge")

    # ---- 3) Görev snap ----
    snapped = {}
    geo = json.load(open(GOREV_JSON))
    for feat in geo['features']:
        p = feat['properties']
        nm = p['name']
        if nm in ('datum', 'start'):
            continue
        xy = (p.get('local_x', 0), p.get('local_y', 0))
        # park_giris → en yakın FRONT (eski 53-60 ID'leri)
        # diğerleri → en yakın NON-PARK node
        if nm == 'park_giris':
            candidates = list(spot_new_ids.keys())   # front ID'leri
        else:
            spot_ids = set(spot_new_ids.values())
            front_ids = set(spot_new_ids.keys())
            candidates = [i for i in g.nodes
                          if i not in spot_ids and i not in front_ids]
        sn = min(candidates, key=lambda i: math.hypot(g.nodes[i][0] - xy[0],
                                                     g.nodes[i][1] - xy[1]))
        dist = math.hypot(g.nodes[sn][0] - xy[0], g.nodes[sn][1] - xy[1])
        ok = dist <= GOREV_SNAP_MAX
        snapped[nm] = {"original": xy, "snap_node": sn,
                       "snap_xy": g.nodes[sn], "distance": dist, "ok": ok}
        flag = "✓" if ok else "✗ UZAK"
        print(f"[5] snap {nm:12s}: ({xy[0]:6.2f},{xy[1]:6.2f}) → node {sn:3d} "
              f"({g.nodes[sn][0]:6.2f},{g.nodes[sn][1]:6.2f}) d={dist:5.2f}m {flag}")

    return {
        "nodes": g.nodes,
        "edges": kept_edges,
        "meta": {
            "park_front_ids": list(spot_new_ids.keys()),    # eski 53-60
            "park_spot_ids":  list(spot_new_ids.values()),  # yeni 61-68 (veya max+1..)
            "park_spot_old_to_new": spot_new_ids,
            "dropped_diagonals": [(min(d), max(d)) for d in
                                   [tuple(s) for s in BAD_GOBEK_DIAGONALS]],
            "snapped_gorevs": snapped,
        },
    }


# ============================================================
def write_yaml(result: dict, path: str = OUT_YAML):
    lines = ["nodes:"]
    for nid in sorted(result['nodes']):
        x, y = result['nodes'][nid]
        lines.append(f"  - id: {nid}\n    x: {x:.4f}\n    y: {y:.4f}")
    lines.append("edges:")
    for u, v in result['edges']:
        lines.append(f"  - [{u}, {v}]")
    with open(path, 'w') as f:
        f.write("\n".join(lines) + "\n")
    print(f"[w] YAML: {path}")


def write_json(result: dict, path: str = OUT_JSON):
    serial = {
        "nodes": [{"id": i, "x": x, "y": y}
                  for i, (x, y) in sorted(result['nodes'].items())],
        "edges": [list(e) for e in result['edges']],
        "meta": {
            "park_front_ids": result['meta']['park_front_ids'],
            "park_spot_ids":  result['meta']['park_spot_ids'],
            "park_spot_old_to_new": {str(k): v for k, v in
                                     result['meta']['park_spot_old_to_new'].items()},
            "dropped_diagonals": result['meta']['dropped_diagonals'],
            "snapped_gorevs": {
                nm: {**{k: v for k, v in d.items() if k != 'snap_xy'},
                     "snap_xy": list(d['snap_xy'])}
                for nm, d in result['meta']['snapped_gorevs'].items()
            },
        },
    }
    with open(path, 'w') as f:
        json.dump(serial, f, indent=2)
    print(f"[w] JSON: {path}")


def write_gorev_snapped(result: dict, path: str = OUT_GOREV):
    geo = json.load(open(GOREV_JSON))
    snaps = result['meta']['snapped_gorevs']
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in snaps and snaps[nm]['ok']:
            sx, sy = snaps[nm]['snap_xy']
            feat['properties']['local_x'] = float(sx)
            feat['properties']['local_y'] = float(sy)
            feat['properties']['_snapped_from'] = list(snaps[nm]['original'])
            feat['properties']['_snap_node'] = snaps[nm]['snap_node']
    geo['_aciklama'] = (geo.get('_aciklama', '') +
        " | gorev_snapped.geojson: yeni graf node'larına snap edildi.")
    with open(path, 'w') as f:
        json.dump(geo, f, indent=2, ensure_ascii=False)
    print(f"[w] gorev_snapped: {path}")


def render_preview(result: dict, path: str = OUT_PNG):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from PIL import Image
    import numpy as np

    fig, ax = plt.subplots(figsize=(16, 13), dpi=120)

    # Track image underlay
    calib = json.load(open(TRACK_CALIB))
    img = np.array(Image.open(TRACK_IMG))
    ax.imshow(img, extent=[calib['x_min'], calib['x_max'],
                           calib['y_min'], calib['y_max']],
              origin='upper', aspect='equal', alpha=0.95)

    nodes = result['nodes']
    edges = result['edges']
    meta = result['meta']

    front_set = set(meta['park_front_ids'])
    spot_set  = set(meta['park_spot_ids'])

    # Edge'leri kategoriye göre renklendir
    seg_main = []
    seg_park_chain = []
    seg_park_dik = []
    for u, v in edges:
        p1, p2 = nodes[u], nodes[v]
        if u in front_set and v in spot_set:
            seg_park_dik.append([p1, p2])
        elif u in spot_set and v in front_set:
            seg_park_dik.append([p1, p2])
        elif u in front_set and v in front_set:
            seg_park_chain.append([p1, p2])
        else:
            seg_main.append([p1, p2])

    ax.add_collection(LineCollection(seg_main, colors='#30d070', linewidths=2.0,
                                     alpha=0.9, zorder=10))
    ax.add_collection(LineCollection(seg_park_chain, colors='#ff60ff', linewidths=2.4,
                                     alpha=0.95, zorder=11))
    ax.add_collection(LineCollection(seg_park_dik, colors='#ffe000', linewidths=2.0,
                                     alpha=1.0, zorder=12))

    # Düştürülen diyagonaller (referans için açık kırmızı kesik)
    for u, v in meta['dropped_diagonals']:
        # eski pozisyonları gösteremiyoruz çünkü 53-60 kaydırıldı; ama 25/37/38/41 stabil
        if u in nodes and v in nodes:
            x1, y1 = nodes[u]; x2, y2 = nodes[v]
            ax.plot([x1, x2], [y1, y2], '--', color='#ff3030', alpha=0.4, lw=1.0, zorder=8)

    # Node'lar — kategoriye göre
    for i, (x, y) in nodes.items():
        if i in front_set:
            ax.plot(x, y, 's', ms=10, mfc='#ff60ff', mec='white', mew=1.0, zorder=15)
            ax.annotate(f"F{i}", (x, y), xytext=(4, 4), textcoords='offset points',
                        color='#ffaaff', fontsize=7, fontweight='bold', zorder=16)
        elif i in spot_set:
            ax.plot(x, y, '^', ms=12, mfc='#ffe000', mec='black', mew=1.0, zorder=15)
            ax.annotate(f"S{i}", (x, y), xytext=(4, 4), textcoords='offset points',
                        color='#ffff80', fontsize=7, fontweight='bold', zorder=16)
        else:
            ax.plot(x, y, 'o', ms=7, mfc='#ff5050', mec='white', mew=0.6, zorder=14)
            ax.annotate(str(i), (x, y), xytext=(3, 2), textcoords='offset points',
                        color='yellow', fontsize=6, zorder=15)

    # Görev snap'leri
    for nm, d in meta['snapped_gorevs'].items():
        ox, oy = d['original']
        sx, sy = d['snap_xy']
        col = '#00ffff' if nm == 'park_giris' else '#ff4040'
        ax.plot(ox, oy, '*', ms=22, mfc=col, mec='black', mew=1.4, alpha=0.5, zorder=20)
        ax.plot(sx, sy, '*', ms=22, mfc=col, mec='white', mew=1.4, zorder=21)
        ax.plot([ox, sx], [oy, sy], '-', color=col, lw=1.2, alpha=0.5, zorder=19)
        ax.annotate(f"{nm}\nd={d['distance']:.1f}m", (sx, sy), xytext=(10, 8),
                    textcoords='offset points', color='black', fontsize=9,
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc=col, alpha=0.9),
                    zorder=22)

    ax.set_xlim(calib['x_min'] - 1, calib['x_max'] + 1)
    ax.set_ylim(calib['y_min'] - 1, calib['y_max'] + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')
    ax.set_facecolor('#1a1c22')

    info = (f"{len(nodes)} node ({len(front_set)} front + {len(spot_set)} spot + "
            f"{len(nodes)-len(front_set)-len(spot_set)} ana yol), {len(edges)} edge\n"
            f"yeşil=ana yol  |  pembe(kare)=park-önü-yol  |  sarı(üçgen)=park spot  "
            f"|  kırmızı kesik=düştürülen göbek diyagonali\n"
            f"kırmızı yıldız=görev nokta (snap) | cyan=park_giris")
    ax.text(0.01, 0.99, info, transform=ax.transAxes, va='top', color='white',
            fontsize=9, family='monospace',
            bbox=dict(boxstyle='round,pad=0.4', fc='#0a0c14', ec='white', alpha=0.85))

    ax.set_title('TALOS sim — düzeltilmiş waypoint graf (lane detector ile fine-tune)',
                 color='white', fontsize=11)
    plt.tight_layout()
    plt.savefig(path, facecolor='#101218', dpi=120)
    print(f"[w] preview: {path}")


# ============================================================
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--no-write', action='store_true',
                        help='YAML/JSON yazma, sadece preview')
    args = parser.parse_args()

    result = build_fixed_graph()

    if not args.no_write:
        write_yaml(result)
        write_json(result)
        write_gorev_snapped(result)

    if args.preview:
        render_preview(result)


if __name__ == '__main__':
    main()
