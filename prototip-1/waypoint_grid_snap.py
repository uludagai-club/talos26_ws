#!/usr/bin/env python3
"""
waypoint_grid_snap — Mevcut grafdaki node'ları DÜZ + 90° grid'e snap eder.

Strateji:
  1. Tüm 74 orijinal edge KORUNUR (göbek diyagonalleri DAHIL).
  2. Göbek node'ları (25, 37, 38, 41) ve D durak node'ları (24, 26, 30, 33, 35)
     POZİSYONLARI KORUNUR — eğri yapılar.
  3. Diğer node'lar: benzer X/Y kümelerine göre snap.
     - Y kümeleri (tolerans 2.5m) → horizontal road, hepsi aynı Y'ye
     - X kümeleri (tolerans 2.5m) → vertical road, hepsi aynı X'e
     - Hem H hem V kümede → kavşak, (cluster_X, cluster_Y) noktasına
  4. Park topolojisi: 53-60 doğuya kaydırılıp FRONT, eski pos'ta yeni spot'lar.
  5. (Opsiyonel) Çift hat dual-lane offset.

Kullanım:
  python3 waypoint_grid_snap.py [--lane-width 2.5] [--preview]
"""
from __future__ import annotations
import argparse, json, math, os
from collections import defaultdict
from typing import Dict, List, Tuple, Set

REPO = os.path.expanduser("~/talos-sim")
GRAPH_JSON = f"{REPO}/scripts/talos26_ws/missions/current_graph.json"
TRACK_IMG  = f"{REPO}/scripts/talos26_ws/waypoint-editor/data/track_layout.jpg"
TRACK_CALIB = f"{REPO}/scripts/talos26_ws/waypoint-editor/track_calibration.json"
GOREV_JSON = f"{REPO}/scripts/talos26_ws/missions/gorev.geojson"
OUT_YAML   = f"{REPO}/scripts/talos26_ws/missions/dual_lane_graph.yaml"
OUT_JSON   = f"{REPO}/scripts/talos26_ws/missions/dual_lane_graph.json"
OUT_PNG    = f"{REPO}/ktr_gorseller/dual_lane_preview.png"

# Korunan node'lar (snap edilmez)
GOBEK_NODES   = {25, 37, 38, 41}    # roundabout — sağ ve sol şerit AYNI pozisyona girer
D_STOP_NODES  = {26, 33}            # iki D durağı — sağ ve sol şerit AYNI pozisyona girer
SHARED_NODES  = GOBEK_NODES | D_STOP_NODES   # her şerit bunları paylaşır
PROTECTED     = SHARED_NODES        # snap edilmez
PARK_OLD_IDS  = set(range(53, 61))

# Cluster toleransları
Y_TOL = 2.5    # m
X_TOL = 2.5    # m

# Park parametreleri
PARK_FRONT_DX = 2.5

# Dual lane
LANE_WIDTH = 2.5

# Göbek merkez (CCW radial reference)
GOBEK_CENTER = (12.0, 0.0)


# ============================================================
def load_graph():
    raw = json.load(open(GRAPH_JSON))
    return ({n["id"]: (n["x"], n["y"]) for n in raw["nodes"]},
            [tuple(e) for e in raw["edges"]])


def classify_node_direction(nid: int, nodes, adj) -> str:
    """
    Node'un en güçlü edge yönelimini bul.
    Returns: 'H' (horizontal), 'V' (vertical), 'B' (both → junction), 'N' (none)
    """
    h_count = v_count = 0
    for nb in adj[nid]:
        if nb not in nodes: continue
        dx = nodes[nb][0] - nodes[nid][0]
        dy = nodes[nb][1] - nodes[nid][1]
        if abs(dx) >= abs(dy):
            h_count += 1
        else:
            v_count += 1
    if h_count == 0 and v_count == 0: return 'N'
    if h_count > 0 and v_count > 0:   return 'B'
    return 'H' if h_count > 0 else 'V'


def cluster_1d(values: List[Tuple[int, float]], tol: float) -> Dict[int, float]:
    """
    Tek-boyutlu greedy cluster: aynı tolerans içindeki value'lar tek küme.
    Returns: {node_id: cluster_value (median)}
    """
    sorted_vals = sorted(values, key=lambda r: r[1])
    clusters = []   # list of [(nid, val), ...]
    for nid, v in sorted_vals:
        if clusters and abs(v - clusters[-1][-1][1]) <= tol:
            clusters[-1].append((nid, v))
        else:
            clusters.append([(nid, v)])
    result = {}
    for cl in clusters:
        med = sorted(x[1] for x in cl)[len(cl) // 2]
        for nid, _ in cl:
            result[nid] = med
    return result


def snap_to_grid(nodes, edges):
    """
    Protected hariç tüm node'ları H/V kümelerine snap.
    """
    adj = defaultdict(set)
    for u, v in edges:
        adj[u].add(v); adj[v].add(u)

    snappable = [n for n in nodes if n not in PROTECTED and n not in PARK_OLD_IDS]
    print(f"  snap'lenebilir: {len(snappable)}, korunan: {len(PROTECTED)} (göbek+D), "
          f"park: {len(PARK_OLD_IDS)}")

    # Sınıflandırma
    classes = {n: classify_node_direction(n, nodes, adj) for n in snappable}
    h_nodes = [(n, nodes[n][1]) for n in snappable if classes[n] in ('H', 'B')]
    v_nodes = [(n, nodes[n][0]) for n in snappable if classes[n] in ('V', 'B')]

    y_snap = cluster_1d(h_nodes, Y_TOL)
    x_snap = cluster_1d(v_nodes, X_TOL)
    print(f"  Y kümeleri: {sorted(set(y_snap.values()))}")
    print(f"  X kümeleri: {sorted(set(x_snap.values()))}")

    new_nodes = dict(nodes)
    for n in snappable:
        x, y = nodes[n]
        nx = x_snap[n] if n in x_snap else x
        ny = y_snap[n] if n in y_snap else y
        new_nodes[n] = (nx, ny)
    return new_nodes


def build_park_topology(nodes, edges):
    new_nodes = dict(nodes)
    spot_old_pos = {i: nodes[i] for i in PARK_OLD_IDS if i in nodes}
    # 53-60 → FRONT (doğuya kaydır)
    for i, (x, y) in spot_old_pos.items():
        new_nodes[i] = (x + PARK_FRONT_DX, y)
    # Yeni spot ID'leri
    next_id = max(new_nodes) + 1
    spot_new_ids = {}
    new_edges = list(edges)
    for old_id, (ox, oy) in spot_old_pos.items():
        spot_new_ids[old_id] = next_id
        new_nodes[next_id] = (ox, oy)
        new_edges.append((old_id, next_id))   # dik bağlantı
        next_id += 1
    return new_nodes, new_edges, list(spot_new_ids.keys()), list(spot_new_ids.values())


def edge_is_horizontal(p1, p2):
    return abs(p2[0] - p1[0]) > abs(p2[1] - p1[1])


def lane_endpoint(node_id, other_id, nodes, adj, gobek, lane_width, side='right'):
    """
    Return lane endpoint at node_id for edge (node_id → other_id).
    side: 'right' (göbeğin dışı, CCW) veya 'left' (içi).
    Köşe/kavşakta endpoint = iki lane'in kesişim noktası (corner intersection).
    """
    nx, ny = nodes[node_id]
    ox, oy = nodes[other_id]
    half = lane_width / 2.0
    s_outer = 1 if side == 'right' else -1   # right=dışa, left=içe

    sign_x_pos = 1 if nx > gobek[0] else -1   # node'un göbeğe göre x tarafı
    sign_y_pos = 1 if ny > gobek[1] else -1
    edge_is_h = edge_is_horizontal((nx, ny), (ox, oy))

    # Bu edge için ana offset (perpendicular direction)
    if edge_is_h:
        # midpoint y'ye göre offset (yatay edge tüm noktaları aynı y'ye offsetler)
        mid_y = (ny + oy) / 2
        s_my = 1 if mid_y > gobek[1] else -1
        y_off = s_outer * s_my * half
        x_off = 0
        # Köşe adjustment: node aynı zamanda V edge'e mi sahip?
        has_other_v = any(
            not edge_is_horizontal((nx, ny), nodes[nb])
            for nb in adj[node_id] if nb != other_id and nb in nodes
        )
        if has_other_v:
            x_off = s_outer * sign_x_pos * half
    else:
        mid_x = (nx + ox) / 2
        s_mx = 1 if mid_x > gobek[0] else -1
        x_off = s_outer * s_mx * half
        y_off = 0
        has_other_h = any(
            edge_is_horizontal((nx, ny), nodes[nb])
            for nb in adj[node_id] if nb != other_id and nb in nodes
        )
        if has_other_h:
            y_off = s_outer * sign_y_pos * half
    return (nx + x_off, ny + y_off)


def build_dual_lane_nodes(centerline_nodes, edges, lane_width=LANE_WIDTH,
                          gobek=(12.0, 0.0),
                          shared_ids: Set[int] = None,
                          start_id: int = None):
    """
    Her non-shared centerline node için YENİ right ve left lane node ID üret.
    Shared node'lar (göbek + D durak): right_id = left_id = original_id.

    Returns:
      new_nodes: dict {id: (x, y)} — shared'lar orijinal pozisyonda, lane'ler offset'li
      right_id_map: {original_id: right_lane_node_id}
      left_id_map:  {original_id: left_lane_node_id}
    """
    shared_ids = shared_ids or set()
    adj = defaultdict(set)
    for u, v in edges:
        adj[u].add(v); adj[v].add(u)

    new_nodes = {}
    right_id_map = {}
    left_id_map = {}

    # Shared node'ları olduğu gibi kopyala
    for nid in shared_ids:
        if nid in centerline_nodes:
            new_nodes[nid] = centerline_nodes[nid]
            right_id_map[nid] = nid
            left_id_map[nid] = nid

    next_id = start_id if start_id is not None else max(centerline_nodes) + 1
    for nid, (nx, ny) in centerline_nodes.items():
        if nid in shared_ids:
            continue
        # Bu node'un herhangi bir edge'i ile lane endpoint hesapla (corner adjusted)
        nbrs = list(adj[nid])
        if not nbrs:
            # İzole node — atla
            continue
        # İlk komşuyla lane endpoint compute et (corner adjustment ile aynı sonucu verir)
        r_pos = lane_endpoint(nid, nbrs[0], centerline_nodes, adj, gobek,
                              lane_width, 'right')
        l_pos = lane_endpoint(nid, nbrs[0], centerline_nodes, adj, gobek,
                              lane_width, 'left')
        right_id_map[nid] = next_id
        new_nodes[next_id] = r_pos
        next_id += 1
        left_id_map[nid] = next_id
        new_nodes[next_id] = l_pos
        next_id += 1
    return new_nodes, right_id_map, left_id_map


def is_ccw_direction(pu, pv, gobek):
    """
    pu→pv edge'i CCW yön mü? Edge midpointinin radial outward'ı ile CCW tangent
    hesapla, edge vektörünün projeksiyonu pozitifse CCW.
    """
    mx, my = (pu[0] + pv[0]) / 2, (pu[1] + pv[1]) / 2
    rx, ry = mx - gobek[0], my - gobek[1]   # radial outward
    # CCW tangent at radial = rotate +90° = (-ry, rx)
    tcx, tcy = -ry, rx
    ex, ey = pv[0] - pu[0], pv[1] - pu[1]
    return (ex * tcx + ey * tcy) > 0


def build_dual_lane_edges(centerline_edges, centerline_nodes, right_map, left_map,
                          shared_ids: Set[int], gobek=(12.0, 0.0)):
    """
    DIRECTED dual-lane edge üretimi.
      - right_edges: CCW yönünde directed (right lane CCW dolaşır)
      - left_edges:  CW yönünde directed (left lane CW dolaşır, right'ın tersi)
      - turn_edges:  intersection diyagonalleri, undirected (her iki yön valid)
      - Shared↔shared edge'ler (göbek diyagonalleri): undirected (her iki yön)
    """
    right_edges = []
    left_edges  = []
    turn_edges  = []

    for u, v in centerline_edges:
        u_shared = u in shared_ids
        v_shared = v in shared_ids
        ru = right_map.get(u); rv = right_map.get(v)
        lu = left_map.get(u);  lv = left_map.get(v)
        if ru is None or rv is None:
            continue
        if u_shared and v_shared:
            # Göbek diyagonalleri: her iki yön valid (roundabout)
            right_edges.append((u, v))
            right_edges.append((v, u))
            continue
        # Centerline u→v CCW yön mü?
        pu = centerline_nodes[u]; pv = centerline_nodes[v]
        ccw = is_ccw_direction(pu, pv, gobek)
        if ccw:
            right_edges.append((ru, rv))   # CCW yönünde right lane
            left_edges.append((lv, lu))    # CW yönünde left lane (ters)
        else:
            right_edges.append((rv, ru))
            left_edges.append((lu, lv))

    # Junction diyagonalleri (undirected — turn semantics her iki yön valid)
    adj = defaultdict(set)
    for u, v in centerline_edges:
        adj[u].add(v); adj[v].add(u)
    for nid in centerline_nodes:
        if nid in shared_ids: continue
        if nid not in right_map or nid not in left_map: continue
        if right_map[nid] == left_map[nid]: continue
        has_h = False; has_v = False
        nx, ny = centerline_nodes[nid]
        for nb in adj[nid]:
            if nb not in centerline_nodes: continue
            bx, by = centerline_nodes[nb]
            if abs(bx - nx) > abs(by - ny):
                has_h = True
            else:
                has_v = True
        if has_h and has_v:
            turn_edges.append((right_map[nid], left_map[nid]))
            turn_edges.append((left_map[nid], right_map[nid]))
    return right_edges, left_edges, turn_edges


def snap_gorev_to(xy, candidate_ids, nodes):
    sn = min(candidate_ids, key=lambda i: math.hypot(nodes[i][0]-xy[0], nodes[i][1]-xy[1]))
    d = math.hypot(nodes[sn][0]-xy[0], nodes[sn][1]-xy[1])
    return sn, d


# ============================================================
def write_yaml(nodes, edges, path):
    lines = ["nodes:"]
    for nid in sorted(nodes):
        x, y = nodes[nid]
        lines.append(f"  - id: {nid}\n    x: {x:.4f}\n    y: {y:.4f}")
    lines.append("edges:")
    for u, v in edges:
        lines.append(f"  - [{u}, {v}]")
    with open(path, 'w') as f:
        f.write("\n".join(lines) + "\n")


def render_preview_v2(nodes, right_edges, left_edges, park_edges,
                      turn_edges,
                      front_ids, spot_ids, snapped, gobek,
                      centerline_nodes=None, right_map=None, left_map=None,
                      path=OUT_PNG):
    """Yeni mimari: gerçek lane node'larıyla çizer."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from PIL import Image
    import numpy as np

    calib = json.load(open(TRACK_CALIB))
    img = np.array(Image.open(TRACK_IMG))

    fig, ax = plt.subplots(figsize=(22, 17), dpi=130)
    ax.imshow(img, extent=[calib['x_min'], calib['x_max'],
                           calib['y_min'], calib['y_max']],
              origin='upper', aspect='equal', alpha=1.0)

    front_set = set(front_ids); spot_set = set(spot_ids)

    # Edges
    seg_r = [[nodes[u], nodes[v]] for u, v in right_edges if u in nodes and v in nodes]
    seg_l = [[nodes[u], nodes[v]] for u, v in left_edges if u in nodes and v in nodes]
    seg_p = [[nodes[u], nodes[v]] for u, v in park_edges if u in nodes and v in nodes]
    seg_t = [[nodes[u], nodes[v]] for u, v in turn_edges if u in nodes and v in nodes]
    ax.add_collection(LineCollection(seg_r, colors='#30d070', linewidths=2.6, alpha=0.95, zorder=10))
    ax.add_collection(LineCollection(seg_l, colors='#3090ff', linewidths=2.6, alpha=0.85, zorder=10))
    ax.add_collection(LineCollection(seg_t, colors='#ff60c0', linewidths=2.0, alpha=0.85, zorder=11, linestyle=(0,(4,2))))
    ax.add_collection(LineCollection(seg_p, colors='#ffe000', linewidths=2.2, alpha=1.0, zorder=11))

    # Right lane nodes (yeşil dolgu)
    right_lane_only = set(right_map.values()) - SHARED_NODES
    left_lane_only  = set(left_map.values())  - SHARED_NODES
    for nid in right_lane_only:
        if nid in nodes:
            x, y = nodes[nid]
            ax.plot(x, y, 'o', ms=6, mfc='#30d070', mec='white', mew=0.4, zorder=14)
            ax.annotate(str(nid), (x, y), xytext=(3, 2), textcoords='offset points',
                        color='#a0ffa0', fontsize=6, zorder=15)
    for nid in left_lane_only:
        if nid in nodes:
            x, y = nodes[nid]
            ax.plot(x, y, 'o', ms=6, mfc='#3090ff', mec='white', mew=0.4, zorder=14)
            ax.annotate(str(nid), (x, y), xytext=(3, 2), textcoords='offset points',
                        color='#a0c0ff', fontsize=6, zorder=15)
    # Shared node'lar (göbek + D durak) — büyük diamond/+
    for nid in SHARED_NODES & set(nodes):
        x, y = nodes[nid]
        if nid in GOBEK_NODES:
            ax.plot(x, y, 'D', ms=14, mfc='#ff8800', mec='white', mew=1.5, zorder=18)
            ax.annotate(f"{nid}", (x, y), xytext=(6, 5), textcoords='offset points',
                        color='#ffd060', fontsize=10, fontweight='bold', zorder=19)
        else:  # D-stop
            ax.plot(x, y, 'P', ms=16, mfc='#00ff80', mec='black', mew=1.5, zorder=18)
            ax.annotate(f"{nid}", (x, y), xytext=(6, 5), textcoords='offset points',
                        color='#a0ffa0', fontsize=10, fontweight='bold', zorder=19)
    # Park front (pembe) + spot (sarı)
    for nid in front_ids:
        if nid in nodes:
            x, y = nodes[nid]
            ax.plot(x, y, 's', ms=12, mfc='#ff60ff', mec='white', mew=1.0, zorder=16)
            ax.annotate(f"F{nid}", (x, y), xytext=(5, 4), textcoords='offset points',
                        color='#ffaaff', fontsize=7, fontweight='bold', zorder=17)
    for nid in spot_ids:
        if nid in nodes:
            x, y = nodes[nid]
            ax.plot(x, y, '^', ms=13, mfc='#ffe000', mec='black', mew=1.0, zorder=16)
            ax.annotate(f"S{nid}", (x, y), xytext=(5, 4), textcoords='offset points',
                        color='#ffff80', fontsize=7, fontweight='bold', zorder=17)

    # Göbek merkez
    ax.plot(*gobek, 'X', ms=18, mfc='#ffff00', mec='black', mew=2, zorder=20)

    # Görev snap
    geo = json.load(open(GOREV_JSON))
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in ('datum', 'start'): continue
        ox = feat['properties'].get('local_x'); oy = feat['properties'].get('local_y')
        if nm not in snapped: continue
        sn, dist = snapped[nm]
        sx, sy = nodes[sn]
        col = '#00ddff' if nm == 'park_giris' else '#ff4488'
        ax.plot(ox, oy, '*', ms=22, mfc=col, mec='black', mew=1.4, alpha=0.55, zorder=20)
        ax.plot(sx, sy, '*', ms=26, mfc=col, mec='white', mew=1.4, zorder=21)
        ax.plot([ox, sx], [oy, sy], '--', color='white', lw=1.5, alpha=0.85, zorder=22)
        ax.annotate(f"{nm} → {sn} (d={dist:.2f}m)", (sx, sy),
                    xytext=(10, 10), textcoords='offset points',
                    color='black', fontsize=10, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc=col, alpha=0.95), zorder=23)

    # Spawn
    ax.plot(-14.935761, -34.031181, 's', ms=18, mfc='#00ffff', mec='black', mew=2, zorder=20)

    ax.set_xlim(calib['x_min']-1, calib['x_max']+1)
    ax.set_ylim(calib['y_min']-1, calib['y_max']+1)
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1c22')
    ax.tick_params(colors='white')
    ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white')
    ax.set_title(
        f'TALOS — dual-lane (her node ID\'li) — lane_width={LANE_WIDTH}m  göbek={gobek}\n'
        f'yeşil(o) right lane node, mavi(o) left lane, turuncu(♦) göbek SHARED, '
        f'yeşil(+) D-stop SHARED\n'
        f'pembe(F) park-önü-yol, sarı(S) park spot, '
        f'şeritler shared node\'lara DOĞRUDAN bağlanır',
        color='white', fontsize=11)
    import matplotlib.pyplot as plt
    plt.tight_layout()
    plt.savefig(path, facecolor='#101218', dpi=130)


def render_preview(nodes, right_segs, left_segs, exc_segs, edges,
                   front_ids, spot_ids, snapped, gobek, path=OUT_PNG):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from PIL import Image
    import numpy as np

    calib = json.load(open(TRACK_CALIB))
    img = np.array(Image.open(TRACK_IMG))

    fig, ax = plt.subplots(figsize=(20, 16), dpi=130)
    ax.imshow(img, extent=[calib['x_min'], calib['x_max'],
                           calib['y_min'], calib['y_max']],
              origin='upper', aspect='equal', alpha=1.0)

    front_set = set(front_ids); spot_set = set(spot_ids)

    # Park bağlantıları (front↔spot ve front zinciri) ayrı kategori
    seg_park_dik = []
    seg_park_chain = []
    for u, v in edges:
        if u not in nodes or v not in nodes: continue
        if (u in spot_set and v in front_set) or (v in spot_set and u in front_set):
            seg_park_dik.append([nodes[u], nodes[v]])
        elif u in front_set and v in front_set:
            seg_park_dik.append([nodes[u], nodes[v]])  # park front chain de dik gibi
            seg_park_chain.append([nodes[u], nodes[v]])

    ax.add_collection(LineCollection(right_segs, colors='#30d070', linewidths=2.4,
                                     alpha=0.95, zorder=10))
    ax.add_collection(LineCollection(left_segs,  colors='#3090ff', linewidths=2.4,
                                     alpha=0.85, zorder=10))
    ax.add_collection(LineCollection(exc_segs,   colors='#ff8800', linewidths=2.4,
                                     alpha=0.9, zorder=11))
    ax.add_collection(LineCollection(seg_park_chain, colors='#ff60ff', linewidths=2.0,
                                     alpha=0.9, zorder=12))
    ax.add_collection(LineCollection([s for s in seg_park_dik if s not in seg_park_chain],
                                      colors='#ffe000', linewidths=2.0,
                                      alpha=1.0, zorder=13))

    # Centerline nodes (küçük, ID'li referans)
    for nid, (x, y) in nodes.items():
        if nid in spot_set:
            ax.plot(x, y, '^', ms=12, mfc='#ffe000', mec='black', mew=1.0, zorder=15)
            ax.annotate(f"S{nid}", (x, y), xytext=(4, 3), textcoords='offset points',
                        color='black', fontsize=7, fontweight='bold', zorder=16)
        elif nid in front_set:
            ax.plot(x, y, 's', ms=11, mfc='#ff60ff', mec='white', mew=0.8, zorder=15)
            ax.annotate(f"F{nid}", (x, y), xytext=(4, 3), textcoords='offset points',
                        color='white', fontsize=7, fontweight='bold', zorder=16)
        elif nid in GOBEK_NODES:
            ax.plot(x, y, 'D', ms=11, mfc='#ff8800', mec='white', mew=1.0, zorder=15)
            ax.annotate(str(nid), (x, y), xytext=(4, 3), textcoords='offset points',
                        color='#ffaa00', fontsize=8, fontweight='bold', zorder=16)
        elif nid in D_STOP_NODES:
            ax.plot(x, y, 'P', ms=12, mfc='#00ff80', mec='black', mew=1.0, zorder=15)
            ax.annotate(str(nid), (x, y), xytext=(4, 3), textcoords='offset points',
                        color='#80ffaa', fontsize=8, fontweight='bold', zorder=16)
        else:
            ax.plot(x, y, 'o', ms=7, mfc='#ff4444', mec='white', mew=0.5, zorder=14)
            ax.annotate(str(nid), (x, y), xytext=(3, 2), textcoords='offset points',
                        color='yellow', fontsize=6, zorder=15)

    # Göbek merkezi göstergesi
    ax.plot(*gobek, 'X', ms=18, mfc='#ffff00', mec='black', mew=2, zorder=18)
    ax.annotate('göbek\nmerkez', gobek, xytext=(10, 5), textcoords='offset points',
                color='black', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', fc='#ffff00', alpha=0.9), zorder=19)

    # Görev snap
    geo = json.load(open(GOREV_JSON))
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in ('datum', 'start'): continue
        ox = feat['properties'].get('local_x'); oy = feat['properties'].get('local_y')
        if nm not in snapped: continue
        sn, dist = snapped[nm]
        sx, sy = nodes[sn]
        col = '#00ddff' if nm == 'park_giris' else '#ff4488'
        ax.plot(ox, oy, '*', ms=22, mfc=col, mec='black', mew=1.4, alpha=0.55, zorder=20)
        ax.plot(sx, sy, '*', ms=26, mfc=col, mec='white', mew=1.4, zorder=21)
        ax.plot([ox, sx], [oy, sy], '--', color='white', lw=1.5, alpha=0.85, zorder=22)
        ax.annotate(f"{nm}\n→ {sn}\nd={dist:.2f}m",
                    (sx, sy), xytext=(10, 10), textcoords='offset points',
                    color='black', fontsize=9, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc=col, alpha=0.95), zorder=23)

    # Spawn
    ax.plot(-14.935761, -34.031181, 's', ms=18, mfc='#00ffff', mec='black', mew=2, zorder=20)

    ax.set_xlim(calib['x_min']-1, calib['x_max']+1)
    ax.set_ylim(calib['y_min']-1, calib['y_max']+1)
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1c22')
    ax.tick_params(colors='white')
    ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white')
    ax.set_title(
        f'TALOS — axis-aligned grid + per-edge dual lane (lane_width={LANE_WIDTH}m)\n'
        f'yeşil=sağ şerit (göbeğin DIŞI, CCW yön), mavi=sol şerit (göbeğin İÇİ)\n'
        f'turuncu=göbek/D (korunan), pembe=park-önü zinciri, sarı=park dik bağ',
        color='white', fontsize=11)
    import matplotlib.pyplot as plt
    plt.tight_layout()
    plt.savefig(path, facecolor='#101218', dpi=130)


# ============================================================
def main():
    global LANE_WIDTH
    p = argparse.ArgumentParser()
    p.add_argument('--lane-width', type=float, default=LANE_WIDTH)
    p.add_argument('--preview', action='store_true')
    args = p.parse_args()
    LANE_WIDTH = args.lane_width

    print(f"[1] graf yükleniyor")
    nodes, edges = load_graph()
    print(f"  {len(nodes)} node, {len(edges)} edge (TÜM orijinal edge'ler korundu)")

    print(f"[2] grid snap (Y_TOL={Y_TOL}, X_TOL={X_TOL})")
    snapped_nodes = snap_to_grid(nodes, edges)

    print(f"[3] park topolojisi (centerline üzerinde — 53-60 doğuya, yeni spotlar eski yere)")
    with_park_nodes, park_extra_edges, front_ids, spot_ids = build_park_topology(
        snapped_nodes, [])   # sadece yeni park edge'leri döndürür
    print(f"  {len(front_ids)} front + {len(spot_ids)} spot, {len(park_extra_edges)} dik bağ")

    # Park nodelarını shared'a ekle (dual-lane offset uygulanmasın)
    PARK_ALL = set(front_ids) | set(spot_ids)
    shared_for_lane = SHARED_NODES | PARK_ALL

    print(f"[4] dual-lane node üretimi (her non-shared/non-park centerline için right+left)")
    lane_nodes, right_map, left_map = build_dual_lane_nodes(
        with_park_nodes, edges, LANE_WIDTH, GOBEK_CENTER, shared_for_lane,
        start_id=max(with_park_nodes) + 1)
    print(f"  shared (göbek+D+park): {len(shared_for_lane)}")
    print(f"  yeni toplam node: {len(lane_nodes)} (centerline {len(with_park_nodes)} → +{len(lane_nodes)-len(with_park_nodes)} lane node)")

    print(f"[5] DIRECTED dual-lane edge üretimi + junction turn edge'leri")
    right_edges, left_edges, turn_edges = build_dual_lane_edges(
        edges, with_park_nodes, right_map, left_map, shared_for_lane, GOBEK_CENTER)
    print(f"  right (CCW directed): {len(right_edges)}")
    print(f"  left  (CW directed):  {len(left_edges)}")
    print(f"  turn (undirected diagonal): {len(turn_edges)}")

    full_nodes = lane_nodes
    full_edges = right_edges + left_edges + turn_edges + park_extra_edges

    print(f"[6] görev snap (dual-lane node'lara)")
    geo = json.load(open(GOREV_JSON))
    snap_result = {}
    # main road right-lane ID'leri (snap target)
    right_lane_ids = [right_map[i] for i in snapped_nodes
                      if i not in SHARED_NODES and i in right_map and i not in PARK_OLD_IDS]
    front_set = set(front_ids)
    spot_set = set(spot_ids)
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in ('datum', 'start'): continue
        xy = (feat['properties'].get('local_x'), feat['properties'].get('local_y'))
        if nm == 'park_giris':
            sn, d = snap_gorev_to(xy, front_ids, full_nodes)
        else:
            # Görevler için: shared D node'lara ya da right_lane'e snap
            candidates = right_lane_ids + list(SHARED_NODES & set(full_nodes))
            sn, d = snap_gorev_to(xy, candidates, full_nodes)
        snap_result[nm] = (sn, d)
        flag = '✓' if d <= 6 else '✗'
        print(f"  {nm:12s}: → node {sn:4d} d={d:.2f}m {flag}")

    print(f"[7] çıktılar yazılıyor")
    write_yaml(full_nodes, full_edges, OUT_YAML)
    print(f"  YAML: {OUT_YAML} ({len(full_nodes)} node, {len(full_edges)} edge)")
    serial = {
        "nodes": [{"id": i, "x": x, "y": y} for i, (x, y) in sorted(full_nodes.items())],
        "right_edges": [list(e) for e in right_edges],
        "left_edges":  [list(e) for e in left_edges],
        "turn_edges":  [list(e) for e in turn_edges],
        "park_edges":  [list(e) for e in park_extra_edges],
        "meta": {
            "lane_width": LANE_WIDTH,
            "gobek_center": list(GOBEK_CENTER),
            "shared_nodes": sorted(SHARED_NODES),
            "right_id_map": {str(k): v for k, v in right_map.items()},
            "left_id_map":  {str(k): v for k, v in left_map.items()},
            "front_ids": front_ids,
            "spot_ids": spot_ids,
            "snapped": {nm: {"node": int(sn), "distance": float(d)}
                        for nm, (sn, d) in snap_result.items()},
        },
    }
    with open(OUT_JSON, 'w') as f:
        json.dump(serial, f, indent=2)
    print(f"  JSON: {OUT_JSON}")

    if args.preview:
        render_preview_v2(full_nodes, right_edges, left_edges, park_extra_edges,
                          turn_edges,
                          front_ids, spot_ids, snap_result, GOBEK_CENTER,
                          centerline_nodes=with_park_nodes,
                          right_map=right_map, left_map=left_map)
        print(f"  preview: {OUT_PNG}")


if __name__ == '__main__':
    main()
