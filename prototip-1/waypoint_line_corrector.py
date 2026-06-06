#!/usr/bin/env python3
"""
waypoint_line_corrector — Bag'den /line ortalamalarını her node yakınında
çıkartıp pozisyonları lateral kaydırır. Sonra eksik ara nokta ekler ve
±lane_width/2 perpendicular ofsetle çift hat üretir.

İş akışı:
  1. Mevcut grafdan başla (6 göbek diyagonali atılmış hâl)
  2. Bag oku: /base_pose_ground_truth + /line
  3. Her node N için: car ∈ R metre içindeyken /line ortalamasını al
  4. N'in tangent yönünü bul (komşu node'lara göre)
  5. shift = K * mean_line  (K = m/° kalibrasyon sabiti)
  6. N += shift * perp_right
  7. Edge uzunluğu > GAP_MAX olan yerlere midpoint(ler) ekle
  8. Park topolojisi: 53-60 doğuya kaydırılıp FRONT olur, yeni spot'lar eski yere
  9. Dual lane: her node için travel direction'a dik ofset → right + left
 10. Çıktı: yaml + json + preview

Kullanım:
  python3 waypoint_line_corrector.py BAG.bag [--lane-width 2.5] [--k 0.05] [--preview]
"""
from __future__ import annotations
import argparse, json, math, os, sys
from typing import Dict, List, Tuple, Set
from collections import defaultdict

REPO = os.path.expanduser("~/talos-sim")
GRAPH_JSON = f"{REPO}/scripts/talos26_ws/missions/current_graph.json"
TRACK_IMG  = f"{REPO}/scripts/talos26_ws/waypoint-editor/data/track_layout.jpg"
TRACK_CALIB = f"{REPO}/scripts/talos26_ws/waypoint-editor/track_calibration.json"
GOREV_JSON = f"{REPO}/scripts/talos26_ws/missions/gorev.geojson"
OUT_DIR    = f"{REPO}/scripts/talos26_ws/missions"
OUT_YAML   = f"{OUT_DIR}/dual_lane_graph.yaml"
OUT_JSON   = f"{OUT_DIR}/dual_lane_graph.json"
OUT_PNG    = f"{REPO}/ktr_gorseller/dual_lane_preview.png"

# --- Parametreler ---
LANE_WIDTH      = 2.5     # çift hat genişliği
K_DEG_TO_M      = 0.05    # /line derece → metre lateral shift sabiti (max ±1.5m)
NODE_RADIUS     = 3.0     # bir node'a "yakın" sayılma yarıçapı (bag içinde)
GAP_MAX         = 5.0     # bu mesafeden büyük edge'lere ara nokta ekle
PARK_FRONT_DX   = 2.5     # park front'un spot'tan doğusu mesafesi
BAD_GOBEK_DIAG = {
    frozenset({25,38}), frozenset({25,41}), frozenset({37,25}),
    frozenset({38,37}), frozenset({38,41}), frozenset({41,37}),
}


# ============================================================
def load_graph():
    raw = json.load(open(GRAPH_JSON))
    nodes = {n["id"]: (n["x"], n["y"]) for n in raw["nodes"]}
    edges = [tuple(e) for e in raw["edges"]
             if frozenset(e) not in BAD_GOBEK_DIAG]
    return nodes, edges


def load_bag(path: str):
    """Bag'den pose ve /line zaman serileri çıkar."""
    import rosbag
    import tf.transformations as tft
    poses = []   # list of (t, x, y, yaw)
    lines = []   # list of (t, deg)
    with rosbag.Bag(path) as b:
        for _, m, t in b.read_messages(topics=['/base_pose_ground_truth']):
            p = m.pose.pose
            q = p.orientation
            yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            poses.append((t.to_sec(), p.position.x, p.position.y, yaw))
        for _, m, t in b.read_messages(topics=['/line']):
            lines.append((t.to_sec(), float(m.data)))
    print(f"  bag yüklendi: {len(poses)} pose, {len(lines)} /line msg, "
          f"süre {poses[-1][0]-poses[0][0]:.1f}s")
    return poses, lines


def correct_nodes_with_line(nodes, edges, poses, lines, K=K_DEG_TO_M,
                            radius=NODE_RADIUS):
    """
    Her node için araç yakındayken /line ortalamasını ölç, perpendicular shift uygula.
    Returns: yeni nodes dict, per-node stats.
    """
    import numpy as np
    P = np.array(poses)   # (N, 4): t, x, y, yaw
    L = np.array(lines)   # (M, 2): t, deg
    if len(P) == 0 or len(L) == 0:
        print("  uyarı: bag boş, düzeltme atlandı")
        return dict(nodes), {}

    # Build adjacency for tangent direction
    adj = defaultdict(set)
    for u, v in edges:
        adj[u].add(v); adj[v].add(u)

    new_nodes = {}
    stats = {}
    for nid, (nx, ny) in nodes.items():
        # 1) Find pose timestamps where car was near node
        d2 = (P[:, 1] - nx)**2 + (P[:, 2] - ny)**2
        near_mask = d2 < radius**2
        if near_mask.sum() < 3:
            new_nodes[nid] = (nx, ny)
            stats[nid] = {"shift": 0.0, "n_samples": 0, "mean_line": 0.0,
                          "reason": "no pose in radius"}
            continue

        # 2) Get /line values at those timestamps (nearest match)
        near_t = P[near_mask, 0]
        # For each near_t find nearest /line timestamp
        line_vals = []
        for t in near_t:
            idx = int(np.argmin(np.abs(L[:, 0] - t)))
            if abs(L[idx, 0] - t) < 0.2:   # < 200ms tolerance
                line_vals.append(L[idx, 1])
        if len(line_vals) < 3:
            new_nodes[nid] = (nx, ny)
            stats[nid] = {"shift": 0.0, "n_samples": 0, "mean_line": 0.0,
                          "reason": "no /line synced"}
            continue
        mean_line = float(np.mean(line_vals))   # degrees

        # 3) Perpendicular direction at node
        # Use average of edge directions to/from this node
        if not adj[nid]:
            tangent = (1.0, 0.0)   # default east
        else:
            # average heading vectors to all neighbors
            tx, ty = 0.0, 0.0
            for nb in adj[nid]:
                bx, by = nodes[nb]
                dx, dy = bx - nx, by - ny
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    tx += dx / norm; ty += dy / norm
            tn = math.hypot(tx, ty)
            if tn < 1e-6:
                tangent = (1.0, 0.0)
            else:
                tangent = (tx / tn, ty / tn)
        # right perpendicular = rotate tangent -90°: (ty, -tx)
        perp_right = (tangent[1], -tangent[0])

        # 4) Shift (positive /line = lane is right of camera → shift node right)
        shift_m = K * mean_line   # +/- 1.5m at clamp
        new_x = nx + shift_m * perp_right[0]
        new_y = ny + shift_m * perp_right[1]
        new_nodes[nid] = (new_x, new_y)
        stats[nid] = {"shift": shift_m, "n_samples": len(line_vals),
                      "mean_line": mean_line, "tangent": tangent,
                      "perp_right": perp_right}

    n_shifted = sum(1 for s in stats.values() if s.get("n_samples", 0) >= 3)
    print(f"  /line düzeltme: {n_shifted}/{len(nodes)} node kaydırıldı "
          f"(diğerleri bag içinde ziyaret edilmemiş)")
    return new_nodes, stats


def densify_edges(nodes, edges, gap_max=GAP_MAX):
    """Edge uzunluğu > gap_max olan kenarlara midpoint(ler) ekle."""
    new_nodes = dict(nodes)
    next_id = max(nodes) + 1
    new_edges = []
    added = 0
    for u, v in edges:
        if u not in new_nodes or v not in new_nodes:
            continue
        ux, uy = new_nodes[u]
        vx, vy = new_nodes[v]
        d = math.hypot(vx - ux, vy - uy)
        if d <= gap_max:
            new_edges.append((u, v))
            continue
        # Bölme sayısı
        k = int(math.ceil(d / gap_max))
        prev = u
        for i in range(1, k):
            t = i / k
            mx = ux + t * (vx - ux)
            my = uy + t * (vy - uy)
            new_nodes[next_id] = (mx, my)
            new_edges.append((prev, next_id))
            prev = next_id
            next_id += 1
            added += 1
        new_edges.append((prev, v))
    print(f"  densify: +{added} ara node, {len(new_edges)} edge ({gap_max}m eşik)")
    return new_nodes, new_edges


def build_park_topology(nodes, edges, old_spot_ids=range(53, 61),
                        dx=PARK_FRONT_DX):
    """53-60 doğuya kaydırılır (front), eski pozisyonlara yeni spot'lar."""
    new_nodes = dict(nodes)
    spot_old_pos = {i: nodes[i] for i in old_spot_ids if i in nodes}
    # Eski 53-60 doğuya kaydır → FRONT pozisyonu
    for i, (x, y) in spot_old_pos.items():
        new_nodes[i] = (x + dx, y)
    # Eski pozisyonlara yeni spot ID'leri
    next_id = max(new_nodes) + 1
    spot_new_ids = {}
    for old_id, (ox, oy) in spot_old_pos.items():
        spot_new_ids[old_id] = next_id
        new_nodes[next_id] = (ox, oy)
        next_id += 1
    # Dik bağlantı: front[i] ↔ spot[i]
    new_edges = list(edges)
    for old_front, new_spot in spot_new_ids.items():
        new_edges.append((old_front, new_spot))
    print(f"  park topoloji: {len(spot_new_ids)} front + {len(spot_new_ids)} spot")
    return new_nodes, new_edges, list(spot_new_ids.keys()), list(spot_new_ids.values())


def build_dual_lane(nodes, edges, lane_width=LANE_WIDTH,
                    excluded_node_ids: Set[int] = None):
    """
    Her node için dominant travel direction bul, perpendicular ofsetle
    right + left lane positions üret. Excluded node'lar (park spotları gibi)
    ofsetlenmez.
    """
    excluded_node_ids = excluded_node_ids or set()
    adj = defaultdict(set)
    for u, v in edges:
        adj[u].add(v); adj[v].add(u)

    right_pos = {}
    left_pos = {}
    for nid, (nx, ny) in nodes.items():
        if nid in excluded_node_ids:
            right_pos[nid] = (nx, ny)
            left_pos[nid] = (nx, ny)
            continue
        if not adj[nid]:
            right_pos[nid] = (nx, ny); left_pos[nid] = (nx, ny); continue
        tx, ty = 0.0, 0.0
        for nb in adj[nid]:
            bx, by = nodes[nb]
            dx, dy = bx - nx, by - ny
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                tx += dx / norm; ty += dy / norm
        tn = math.hypot(tx, ty)
        if tn < 1e-6:
            right_pos[nid] = (nx, ny); left_pos[nid] = (nx, ny); continue
        tx /= tn; ty /= tn
        perp_r = (ty, -tx)
        off = lane_width / 2.0
        right_pos[nid] = (nx + off * perp_r[0], ny + off * perp_r[1])
        left_pos[nid]  = (nx - off * perp_r[0], ny - off * perp_r[1])
    return right_pos, left_pos


def snap_gorevs(gorev_xy, right_nodes, exclude=None, max_dist=6.0):
    exclude = exclude or set()
    cand = [i for i in right_nodes if i not in exclude]
    sn = min(cand, key=lambda i: math.hypot(right_nodes[i][0] - gorev_xy[0],
                                            right_nodes[i][1] - gorev_xy[1]))
    d = math.hypot(right_nodes[sn][0] - gorev_xy[0],
                   right_nodes[sn][1] - gorev_xy[1])
    return sn, d, d <= max_dist


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


def render_preview(nodes_centerline, right_pos, left_pos, edges, front_ids,
                   spot_ids, snapped, shift_stats, path=OUT_PNG):
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

    # Right + Left lane edges
    seg_right, seg_left, seg_park_dik, seg_park_chain = [], [], [], []
    for u, v in edges:
        if u not in nodes_centerline or v not in nodes_centerline: continue
        if u in spot_set or v in spot_set:
            # park dik
            p1 = right_pos[u] if u not in spot_set else nodes_centerline[u]
            p2 = right_pos[v] if v not in spot_set else nodes_centerline[v]
            seg_park_dik.append([p1, p2])
        elif u in front_set and v in front_set:
            seg_park_chain.append([right_pos[u], right_pos[v]])
        else:
            seg_right.append([right_pos[u], right_pos[v]])
            seg_left.append([left_pos[u], left_pos[v]])

    ax.add_collection(LineCollection(seg_right, colors='#30d070', linewidths=2.0, alpha=0.9, zorder=10))
    ax.add_collection(LineCollection(seg_left,  colors='#3080ff', linewidths=2.0, alpha=0.8, zorder=10))
    ax.add_collection(LineCollection(seg_park_chain, colors='#ff60ff', linewidths=2.2, alpha=0.95, zorder=11))
    ax.add_collection(LineCollection(seg_park_dik, colors='#ffe000', linewidths=2.0, alpha=1.0, zorder=12))

    # Nodes
    for nid, (x, y) in nodes_centerline.items():
        if nid in spot_set:
            ax.plot(x, y, '^', ms=11, mfc='#ffe000', mec='black', mew=1.0, zorder=15)
            ax.annotate(f"S{nid}", (x, y), xytext=(4, 3), textcoords='offset points',
                        color='black', fontsize=7, fontweight='bold', zorder=16)
        elif nid in front_set:
            ax.plot(x, y, 's', ms=10, mfc='#ff60ff', mec='white', mew=0.8, zorder=15)
        else:
            # Renk shift_stats'a göre: kaydırıldıysa parlak, kaydırılmadıysa karanlık
            stat = shift_stats.get(nid, {})
            n_samples = stat.get('n_samples', 0)
            color = '#ff4444' if n_samples >= 3 else '#664444'
            ax.plot(x, y, 'o', ms=6, mfc=color, mec='white', mew=0.4, zorder=14)
            ax.annotate(str(nid), (x, y), xytext=(3, 2), textcoords='offset points',
                        color='yellow', fontsize=6, zorder=15)

    # Right + Left lane node points
    for nid in nodes_centerline:
        if nid in spot_set or nid in front_set: continue
        rx, ry = right_pos[nid]; lx, ly = left_pos[nid]
        ax.plot(rx, ry, '.', ms=4, color='#30d070', zorder=13)
        ax.plot(lx, ly, '.', ms=4, color='#3080ff', zorder=13)

    # Görev snap (yıldız)
    geo = json.load(open(GOREV_JSON))
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in ('datum', 'start'): continue
        ox = feat['properties'].get('local_x'); oy = feat['properties'].get('local_y')
        if nm not in snapped: continue
        sn, dist, ok = snapped[nm]
        sx, sy = right_pos[sn] if sn in right_pos else nodes_centerline[sn]
        col = '#00ddff' if nm == 'park_giris' else '#ff4488'
        ax.plot(ox, oy, '*', ms=22, mfc=col, mec='black', mew=1.4, alpha=0.6, zorder=20)
        ax.plot(sx, sy, '*', ms=24, mfc=col, mec='white', mew=1.4, zorder=21)
        ax.plot([ox, sx], [oy, sy], '--', color='white', lw=1.5, alpha=0.85, zorder=22)
        flag = '✓' if ok else '✗'
        ax.annotate(f"{nm}\n→ node {sn}\nd={dist:.2f}m {flag}",
                    (sx, sy), xytext=(10, 10), textcoords='offset points',
                    color='black', fontsize=9, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc=col, alpha=0.9), zorder=23)

    # Spawn
    ax.plot(-14.935761, -34.031181, 's', ms=18, mfc='#00ffff', mec='black', mew=2, zorder=20)

    ax.set_xlim(calib['x_min']-1, calib['x_max']+1)
    ax.set_ylim(calib['y_min']-1, calib['y_max']+1)
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1c22')
    ax.tick_params(colors='white')
    ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white')

    n_shifted = sum(1 for s in shift_stats.values() if s.get('n_samples', 0) >= 3)
    ax.set_title(
        f'TALOS — /line düzeltilmiş + dual-lane (K={K_DEG_TO_M} m/°, lane_width={LANE_WIDTH}m)\n'
        f'yeşil=sağ şerit, mavi=sol şerit, pembe(F)=park-önü, sarı(S)=park spot\n'
        f'{n_shifted}/{len(shift_stats)} node /line ile kaydırıldı (parlak kırmızı=kaydırıldı, koyu=yetersiz veri)',
        color='white', fontsize=11)
    import matplotlib.pyplot as plt
    plt.tight_layout()
    plt.savefig(path, facecolor='#101218', dpi=130)
    print(f"  preview: {path}")


# ============================================================
def main():
    global LANE_WIDTH, K_DEG_TO_M
    p = argparse.ArgumentParser()
    p.add_argument('bag', help='bag dosyası yolu')
    p.add_argument('--lane-width', type=float, default=LANE_WIDTH)
    p.add_argument('--k', type=float, default=K_DEG_TO_M,
                   help='/line derece → metre çevirim sabiti (varsayılan 0.05)')
    p.add_argument('--preview', action='store_true')
    args = p.parse_args()

    LANE_WIDTH = args.lane_width
    K_DEG_TO_M = args.k

    print(f"[1] graf yükleniyor")
    nodes, edges = load_graph()
    print(f"  {len(nodes)} node, {len(edges)} edge (6 göbek diyagonali atılmış)")

    print(f"[2] bag okunuyor: {args.bag}")
    poses, lines = load_bag(args.bag)

    print(f"[3] /line tabanlı node düzeltme (K={K_DEG_TO_M} m/°, r={NODE_RADIUS}m)")
    corrected, stats = correct_nodes_with_line(nodes, edges, poses, lines)

    print(f"[4] edge densify (gap_max={GAP_MAX}m)")
    densified, densified_edges = densify_edges(corrected, edges)

    print(f"[5] park topolojisi")
    full_nodes, full_edges, front_ids, spot_ids = build_park_topology(
        densified, densified_edges)

    print(f"[6] dual lane offset (lane_width={LANE_WIDTH}m)")
    excluded = set(spot_ids)   # spot'ları ofsetleme
    right_pos, left_pos = build_dual_lane(full_nodes, full_edges,
                                          lane_width=LANE_WIDTH,
                                          excluded_node_ids=excluded)

    print(f"[7] görev snap")
    geo = json.load(open(GOREV_JSON))
    snapped = {}
    for feat in geo['features']:
        nm = feat['properties']['name']
        if nm in ('datum', 'start'): continue
        xy = (feat['properties'].get('local_x'), feat['properties'].get('local_y'))
        if nm == 'park_giris':
            sn, d, ok = snap_gorevs(xy, full_nodes, exclude=set(spot_ids)|set(right_pos.keys())-set(front_ids))
        else:
            sn, d, ok = snap_gorevs(xy, right_pos, exclude=set(spot_ids)|set(front_ids))
        snapped[nm] = (sn, d, ok)
        flag = '✓' if ok else '✗'
        print(f"  {nm:12s}: → node {sn:4d} d={d:.2f}m {flag}")

    print(f"[8] çıktılar yazılıyor")
    write_yaml(full_nodes, full_edges, OUT_YAML)
    print(f"  YAML: {OUT_YAML}")

    serial = {
        "centerline": [{"id": i, "x": x, "y": y} for i, (x, y) in sorted(full_nodes.items())],
        "right_lane": [{"id": i, "x": x, "y": y} for i, (x, y) in sorted(right_pos.items())],
        "left_lane":  [{"id": i, "x": x, "y": y} for i, (x, y) in sorted(left_pos.items())],
        "edges": [list(e) for e in full_edges],
        "meta": {
            "lane_width": LANE_WIDTH,
            "k_deg_to_m": K_DEG_TO_M,
            "front_ids": front_ids,
            "spot_ids": spot_ids,
            "snapped": {nm: {"node": int(sn), "distance": float(d), "ok": bool(ok)}
                        for nm, (sn, d, ok) in snapped.items()},
            "n_shifted": sum(1 for s in stats.values() if s.get('n_samples', 0) >= 3),
            "n_total": len(stats),
        },
    }
    with open(OUT_JSON, 'w') as f:
        json.dump(serial, f, indent=2)
    print(f"  JSON: {OUT_JSON}")

    if args.preview:
        render_preview(full_nodes, right_pos, left_pos, full_edges,
                       front_ids, spot_ids, snapped, stats)


if __name__ == '__main__':
    main()
