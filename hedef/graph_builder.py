# -*- coding: utf-8 -*-
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import math
def build_track_graph():
    import networkx as nx
    import numpy as np

    SPACING = 2.0
    MIN_DENSIFY = 2.5
    DENSE_SPACING = 2.0

    def densify_segment(p1, p2, spacing):
        d = np.linalg.norm(np.array(p2) - np.array(p1))
        n = int(round(d / spacing))
        if n <= 1:
            return []
        t_vals = np.linspace(0, 1, n + 1)[1:-1]
        return [(round(p1[0] + t*(p2[0]-p1[0]), 2),
                 round(p1[1] + t*(p2[1]-p1[1]), 2)) for t in t_vals]

    def build_lane(prefix, key_points, closed=True, densify=True, dense_spacing_non_curve=None):
        vertices = []
        vertex_types = []
        n_keys = len(key_points)
        edge_count = n_keys if closed else n_keys - 1
        for i in range(n_keys):
            x1, y1, v1 = key_points[i]
            vertices.append((x1, y1))
            if v1 == True:
                vertex_types.append('viraj')
            elif v1 == 'giris':
                vertex_types.append('giris')
            else:
                vertex_types.append('key')
            if densify and i < edge_count:
                j = (i + 1) % n_keys
                x2, y2, v2 = key_points[j]
                d = np.linalg.norm(np.array([x2, y2]) - np.array([x1, y1]))
                both_special = (v1 in (True, 'giris')) and (v2 in (True, 'giris'))
                if not both_special:
                    is_curve_segment = (v1 == True) or (v2 == True)
                    if dense_spacing_non_curve is not None and not is_curve_segment:
                        current_spacing = dense_spacing_non_curve
                        current_min_densify = dense_spacing_non_curve * 1.2
                    else:
                        current_spacing = SPACING
                        current_min_densify = MIN_DENSIFY

                    if d >= current_min_densify:
                        for pt in densify_segment((x1, y1), (x2, y2), current_spacing):
                            vertices.append(pt)
                            vertex_types.append('intermediate')
        names = [f"{prefix}{i+1}" for i in range(len(vertices))]
        return names, vertices, vertex_types

    A_key = [
        (-1.94, -34.27, False),   (10.10, -34.27, False),
        (23.97, -34.27, False),   (33.05, -34.27, False),
        (34.41, -34.41, True),    (34.97, -33.78, True),
        (35.20, -32.19, False),   (35.20, -21.32, False),
        (35.20, -2.37,  False),   (35.20,  9.14,  False),
        (35.18,  11.07, True),    (33.40,  11.65, False),
        (25.70,  11.65, False),   ( 9.88,  11.65, False),
        (-2.36,  11.65, False),   (-3.69,  11.76, True),
        (-4.55,  11.53, True),    (-4.73,  10.78, True),
        (-5.21,   9.56, False),   (-5.21,  -1.09, False),
        (-5.21, -19.69, False),   (-5.21, -31.06, False),
        (-5.48, -32.93, True),    (-4.48, -33.95, True),
    ]

    B_key = [
        (-1.34,  9.38, False),    ( 8.81,  9.38, False),    (22.64,  9.38, False),    (31.98,  9.38, False),
        (33.25,  8.89, True),     (33.07,  7.63, False),    (33.07, -0.10, False),    (33.07,-18.57, False),
        (33.07,-30.23, False),    (32.99,-31.77, True),     (32.23,-32.15, True),     (31.15,-32.05, False),
        (26.32,-32.05, False),    (11.90,-32.05, False),    (-1.96,-32.05, False),    (-2.85,-31.95, True),
        (-2.96,-31.30, True),     (-2.96,-29.71, False),    (-2.96,-22.62, False),    (-2.96, -4.36, False),
        (-2.96,  7.62, False),    (-2.71,  9.20, True),     (-2.27,  9.58, True),
    ]

    C_key = [
        (-1.48, -22.04, False),   ( 8.63, -22.04, False),   (22.43, -22.04, False),   (31.91, -22.04, False),
    ]

    D_key = [
        (31.91, -19.74, False),   (22.43, -19.74, False),   ( 8.63, -19.74, False),   (-1.48, -19.74, False),
    ]

    E_key = [
        (23.47,   7.54, False),   (23.47, -30.43, False),
    ]

    F_key = [
        (25.74, -30.43, False),   (25.74,   7.54, False),
    ]

    G_key = [
        (11.78,  3.45, 'giris'),  ( 9.97,  3.45, 'giris'),  ( 8.38,  2.66, False),    ( 7.11,  1.53, False),
        ( 6.11,  0.36, False),    ( 5.62, -1.24, 'giris'),  ( 5.62, -3.11, 'giris'),  ( 6.16, -4.59, False),
        ( 6.96, -5.99, False),    ( 8.27, -6.95, False),    ( 9.65, -7.52, 'giris'),  (11.64, -7.52, 'giris'),
        (12.92, -6.97, False),    (14.25, -6.15, False),    (15.30, -4.82, False),    (16.04, -3.08, 'giris'),
        (16.04, -0.84, 'giris'),  (15.02,  0.97, False),    (13.41,  2.48, False),
    ]

    H_key = [
        (11.55, -30.18, False),   (11.55, -20.97, False),   (11.55, -10.04, False),
    ]

    I_key = [
        ( 9.46, -10.04, False),   ( 9.46, -20.97, False),   ( 9.46, -30.18, False),
    ]

    J_key = [
        (31.64, -1.14, False),    (24.61, -1.14, False),    (18.72, -1.14, False),
    ]

    K_key = [
        (18.72, -3.27, False),    (24.61, -3.27, False),    (31.64, -3.27, False),
    ]

    L_key = [
        ( 9.85,  7.39, False),    ( 9.85,  5.70, False),
    ]

    M_key = [
        (12.08,  5.70, False),    (12.08,  7.39, False),
    ]

    N_key = [
        (-1.18, -3.10, False),    ( 1.17, -3.10, False),    ( 3.56, -3.10, False),
    ]

    O_key = [
        ( 3.56, -0.62, False),    ( 1.17, -0.62, False),    (-1.18, -0.62, False),
    ]

    P_key = [
        (36.33, -7.88, False),    (37.38, -6.91, False),    (37.44, -5.57, False),
        (37.43, -4.18, False),    (37.45, -2.90, False),    (37.13, -1.83, False),
        (36.22, -1.06, False),
    ]

    # Build original reference lanes to map original node names to coordinates
    ref_nodes = {}
    for prefix, key_points, closed in [
        ("A", A_key, True), ("B", B_key, True),
        ("C", C_key, False), ("D", D_key, False),
        ("E", E_key, False), ("F", F_key, False),
        ("G", G_key, True), ("H", H_key, False),
        ("I", I_key, False), ("J", J_key, False),
        ("K", K_key, False), ("L", L_key, False),
        ("M", M_key, False), ("N", N_key, False),
        ("O", O_key, False), ("P", P_key, False)
    ]:
        names, vertices, _ = build_lane(prefix, key_points, closed=closed, densify=True)
        for name, pt in zip(names, vertices):
            ref_nodes[name] = pt

    def resolve_node_name(node_name, G_new):
        if node_name in ref_nodes:
            target_pos = ref_nodes[node_name]
            lane_prefix = node_name[0]
            lane_nodes = [n for n in G_new.nodes() if G_new.nodes[n].get('lane') == lane_prefix]
            if not lane_nodes:
                return node_name
            closest_node = min(lane_nodes, key=lambda n: np.linalg.norm(np.array(G_new.nodes[n]['pos']) - np.array(target_pos)))
            return closest_node
        return node_name

    G = nx.DiGraph()

    lanes = {
        "A": build_lane("A", A_key, closed=True, dense_spacing_non_curve=DENSE_SPACING),
        "B": build_lane("B", B_key, closed=True, dense_spacing_non_curve=DENSE_SPACING),
        "C": build_lane("C", C_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "D": build_lane("D", D_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "E": build_lane("E", E_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "F": build_lane("F", F_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "G": build_lane("G", G_key, closed=True, densify=False),
        "H": build_lane("H", H_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "I": build_lane("I", I_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "J": build_lane("J", J_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "K": build_lane("K", K_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "L": build_lane("L", L_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "M": build_lane("M", M_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "N": build_lane("N", N_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "O": build_lane("O", O_key, closed=False, dense_spacing_non_curve=DENSE_SPACING),
        "P": build_lane("P", P_key, closed=False),
    }

    closed_lanes = {"A": True, "B": True, "C": False, "D": False, "E": False, "F": False, "G": True, "H": False, "I": False, "J": False, "K": False, "L": False, "M": False, "N": False, "O": False, "P": False}
    bidirectional_lanes = set()

    for prefix, (names, vertices, vtypes) in lanes.items():
        for i, (name, (x, y), vt) in enumerate(zip(names, vertices, vtypes)):
            G.add_node(name, pos=(x, y), type=vt, lane=prefix)
        is_closed = closed_lanes[prefix]
        edge_count = len(names) if is_closed else len(names) - 1
        for i in range(edge_count):
            j = (i + 1) % len(names)
            d = float(np.linalg.norm(np.array(vertices[j]) - np.array(vertices[i])))
            G.add_edge(names[i], names[j], weight=d, type="lane")
            if prefix in bidirectional_lanes:
                G.add_edge(names[j], names[i], weight=d, type="lane")

    def add_curved_conn(src, dst, approach_dir, exit_dir, n_mid=4, conn_type='connection'):
        resolved_src = resolve_node_name(src, G)
        resolved_dst = resolve_node_name(dst, G)
        p0 = np.array(G.nodes[resolved_src]['pos'])
        p2 = np.array(G.nodes[resolved_dst]['pos'])
        if approach_dir == "left" and exit_dir == "right":
            # Left U-turn: loops westward (further left) for right-hand traffic
            p1 = np.array([min(p0[0], p2[0]) - 2.0, (p0[1] + p2[1]) / 2.0])
        elif approach_dir == "right" and exit_dir == "left":
            # Right U-turn: loops eastward
            p1 = np.array([max(p0[0], p2[0]) + 2.0, (p0[1] + p2[1]) / 2.0])
        elif approach_dir in ('right', 'left') and exit_dir in ('up', 'down'):
            p1 = np.array([p2[0], p0[1]])
        elif approach_dir in ('up', 'down') and exit_dir in ('right', 'left'):
            p1 = np.array([p0[0], p2[1]])
        else:
            p1 = (p0 + p2) / 2.0
        t_vals = np.linspace(0, 1, n_mid + 2)[1:-1]
        mid_nodes = []
        for idx, t in enumerate(t_vals):
            pt = (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2
            pt = (round(pt[0], 2), round(pt[1], 2))
            node_name = f"X_{resolved_src}_{resolved_dst}_{idx+1}"
            G.add_node(node_name, pos=pt, type=conn_type, lane='X')
            mid_nodes.append(node_name)
        prev = resolved_src
        for node in mid_nodes:
            d = float(np.linalg.norm(np.array(G.nodes[node]['pos']) - np.array(G.nodes[prev]['pos'])))
            G.add_edge(prev, node, weight=d, type=conn_type)
            prev = node
        d = float(np.linalg.norm(np.array(G.nodes[resolved_dst]['pos']) - np.array(G.nodes[prev]['pos'])))
        G.add_edge(prev, resolved_dst, weight=d, type=conn_type)

    connections_to_build = [
        ("L2", "G2", "down", "left"),
        ("G1", "M1", "left", "up"),
        ("H11", "G12", "up", "right"),
        ("G11", "I1", "right", "down"),
        ("J8", "G17", "left", "up"),
        ("G16", "K1", "up", "right"),
        ("N3", "G7", "right", "down"),
        ("G6", "O1", "down", "left"),
        ("B6", "L1", "right", "down"),
        ("M2", "B8", "up", "right"),
        ("B51", "H1", "left", "up"),
        ("I11", "B52", "down", "left"),
        ("B24", "J1", "down", "left"),
        ("K8", "B26", "right", "down"),
        ("B74", "N1", "up", "right"),
        ("O3", "B76", "left", "up"),
        ("B65", "C1", "up", "right"),
        ("C18", "B35", "right", "down"),
        ("B33", "D1", "down", "left"),
        ("D18", "B67", "left", "up"),
        ("B13", "E1", "right", "down"),
        ("E20", "B46", "down", "left"),
        ("B44", "F1", "left", "up"),
        ("F20", "B15", "up", "right"),
        ("C6", "I7", "right", "down"),
        ("H5", "C8", "up", "right"),
        ("D11", "H7", "left", "up"),
        ("I5", "D13", "down", "left"),
        ("C7", "H7", "right", "up"),
        ("H6", "D13", "up", "left"),
        ("D12", "I7", "left", "down"),
        ("I6", "C8", "down", "right"),
        ("C13", "E16", "right", "down"),
        ("F5", "C15", "up", "right"),
        ("D4", "F7", "left", "up"),
        ("E14", "D6", "down", "left"),
        ("C14", "F7", "right", "up"),
        ("F6", "D6", "up", "left"),
        ("D5", "E16", "left", "down"),
        ("E15", "C15", "down", "right"),
        ("J4", "F16", "left", "up"),
        ("K3", "E7", "right", "down"),
        ("E5", "J6", "down", "left"),
        ("F14", "K5", "up", "right"),
        ("J5", "E7", "left", "down"),
        ("K4", "F16", "right", "up"),
        ("E6", "K5", "down", "right"),
        ("F15", "J6", "up", "left"),
        ("A48", "E1", "left", "down"),
        ("F20", "A49", "up", "left"),
        ("A55", "L1", "left", "down"),
        ("M2", "A56", "up", "left"),
        ("A14", "F1", "right", "up"),
        ("E20", "A15", "down", "right"),
        ("A7", "H1", "right", "up"),
        ("I11", "A8", "down", "right"),
        ("A26", "D1", "up", "left"),
        ("C18", "A27", "right", "up"),
        ("A36", "J1", "up", "left"),
        ("K8", "A37", "right", "up"),
        ("A80", "C1", "down", "right"),
        ("D18", "A81", "left", "down"),
        ("A71", "N1", "down", "right"),
        ("O3", "A72", "left", "down"),
    ]

    def get_nodes_in_lane(prefix):
        names, _, _ = lanes[prefix]
        return names

    a_nodes = get_nodes_in_lane("A")
    b_nodes = get_nodes_in_lane("B")

    a_top = sorted([n for n in a_nodes if G.nodes[n]['pos'][1] > 11.0 and -3.0 < G.nodes[n]['pos'][0] < 34.0], key=lambda n: G.nodes[n]['pos'][0])
    b_top = sorted([n for n in b_nodes if G.nodes[n]['pos'][1] > 9.0 and -2.0 < G.nodes[n]['pos'][0] < 32.5], key=lambda n: G.nodes[n]['pos'][0])
    a_bottom = sorted([n for n in a_nodes if G.nodes[n]['pos'][1] < -33.0 and -2.5 < G.nodes[n]['pos'][0] < 34.0], key=lambda n: G.nodes[n]['pos'][0])
    b_bottom = sorted([n for n in b_nodes if G.nodes[n]['pos'][1] < -31.5 and -2.5 < G.nodes[n]['pos'][0] < 32.5], key=lambda n: G.nodes[n]['pos'][0])
    a_left = sorted([n for n in a_nodes if G.nodes[n]['pos'][0] < -4.5 and -31.5 < G.nodes[n]['pos'][1] < 10.0], key=lambda n: G.nodes[n]['pos'][1])
    b_left = sorted([n for n in b_nodes if G.nodes[n]['pos'][0] < -2.5 and -30.5 < G.nodes[n]['pos'][1] < 8.0], key=lambda n: G.nodes[n]['pos'][1])
    a_right = sorted([n for n in a_nodes if G.nodes[n]['pos'][0] > 34.5 and -32.5 < G.nodes[n]['pos'][1] < 9.5], key=lambda n: G.nodes[n]['pos'][1])
    b_right = sorted([n for n in b_nodes if G.nodes[n]['pos'][0] > 32.5 and -30.5 < G.nodes[n]['pos'][1] < 8.0], key=lambda n: G.nodes[n]['pos'][1])

    c_straight = sorted(get_nodes_in_lane("C"), key=lambda n: G.nodes[n]['pos'][0])
    d_straight = sorted(get_nodes_in_lane("D"), key=lambda n: G.nodes[n]['pos'][0])
    e_straight = sorted(get_nodes_in_lane("E"), key=lambda n: G.nodes[n]['pos'][1])
    f_straight = sorted(get_nodes_in_lane("F"), key=lambda n: G.nodes[n]['pos'][1])
    j_straight = sorted(get_nodes_in_lane("J"), key=lambda n: G.nodes[n]['pos'][0])
    k_straight = sorted(get_nodes_in_lane("K"), key=lambda n: G.nodes[n]['pos'][0])
    n_straight = sorted(get_nodes_in_lane("N"), key=lambda n: G.nodes[n]['pos'][0])
    o_straight = sorted(get_nodes_in_lane("O"), key=lambda n: G.nodes[n]['pos'][0])
    l_straight = sorted(get_nodes_in_lane("L"), key=lambda n: G.nodes[n]['pos'][1])
    m_straight = sorted(get_nodes_in_lane("M"), key=lambda n: G.nodes[n]['pos'][1])
    h_straight = sorted(get_nodes_in_lane("H"), key=lambda n: G.nodes[n]['pos'][1])
    i_straight = sorted(get_nodes_in_lane("I"), key=lambda n: G.nodes[n]['pos'][1])

    slalom_segments = [
        (a_top, b_top, "horizontal"),
        (a_bottom, b_bottom, "horizontal"),
        (a_left, b_left, "vertical"),
        (a_right, b_right, "vertical"),
        (c_straight, d_straight, "horizontal"),
        (e_straight, f_straight, "vertical"),
        (h_straight, i_straight, "vertical"),
        (k_straight, j_straight, "horizontal"),
        (n_straight, o_straight, "horizontal"),
        (m_straight, l_straight, "vertical"),
    ]

    for lane1_nodes, lane2_nodes, orientation in slalom_segments:
        paired = []
        for u in lane1_nodes:
            u_coord = G.nodes[u]['pos'][0] if orientation == "horizontal" else G.nodes[u]['pos'][1]
            nearest_v = min(lane2_nodes, key=lambda v: abs((G.nodes[v]['pos'][0] if orientation == "horizontal" else G.nodes[v]['pos'][1]) - u_coord))
            v_coord = G.nodes[nearest_v]['pos'][0] if orientation == "horizontal" else G.nodes[nearest_v]['pos'][1]
            if abs(u_coord - v_coord) < 3.0:
                paired.append((u, nearest_v))
        seen_v = set()
        unique_paired = []
        for u, v in paired:
            if v not in seen_v:
                unique_paired.append((u, v))
                seen_v.add(v)
        for u, v in unique_paired:
            pos_u = G.nodes[u]['pos']
            pos_v = G.nodes[v]['pos']
            if orientation == "horizontal":
                x_avg = round((pos_u[0] + pos_v[0]) / 2.0, 2)
                G.nodes[u]['pos'] = (x_avg, pos_u[1])
                G.nodes[v]['pos'] = (x_avg, pos_v[1])
            else:
                y_avg = round((pos_u[1] + pos_v[1]) / 2.0, 2)
                G.nodes[u]['pos'] = (pos_u[0], y_avg)
                G.nodes[v]['pos'] = (pos_v[0], y_avg)

    slalom_connections_to_build = []
    # Karşı-şerit boylamasına (lane-parallel) segment kenarları: blok geldiğinde
    # bunların ters yönü "ters şeritte KALMA" (ucuz) cezasıyla enjekte edilir →
    # araç engeli geçene kadar karşı şeritte İLERİ seyredebilir (yalnız crossing
    # zig-zag'ı değil). YALNIZ düz paralel kesim node'ları (köşe/connection YOK)
    # → dikey şeride kaçma artefaktı oluşmaz.
    slalom_lane_segments = []
    for lane1_nodes, lane2_nodes, orientation in slalom_segments:
        paired = []
        for u in lane1_nodes:
            u_coord = G.nodes[u]['pos'][0] if orientation == "horizontal" else G.nodes[u]['pos'][1]
            nearest_v = min(lane2_nodes, key=lambda v: abs((G.nodes[v]['pos'][0] if orientation == "horizontal" else G.nodes[v]['pos'][1]) - u_coord))
            v_coord = G.nodes[nearest_v]['pos'][0] if orientation == "horizontal" else G.nodes[nearest_v]['pos'][1]
            if abs(u_coord - v_coord) < 3.0:
                paired.append((u, nearest_v))
        seen_v = set()
        unique_paired = []
        for u, v in paired:
            if v not in seen_v:
                unique_paired.append((u, v))
                seen_v.add(v)
        unique_paired.sort(key=lambda p: G.nodes[p[0]]['pos'][0] if orientation == "horizontal" else G.nodes[p[0]]['pos'][1])
        for i in range(len(unique_paired) - 1):
            u1, v1 = unique_paired[i]
            u2, v2 = unique_paired[i+1]
            # İki şeridin boylamasına segmentleri (ardışık eşleşmiş node'lar arası)
            slalom_lane_segments.append((u1, u2))
            slalom_lane_segments.append((v1, v2))
            if orientation == "horizontal":
                slalom_connections_to_build.append((u1, v2, "left", "right"))
                slalom_connections_to_build.append((v2, u1, "right", "left"))
                slalom_connections_to_build.append((u2, v1, "right", "left"))
                slalom_connections_to_build.append((v1, u2, "left", "right"))
            else:
                slalom_connections_to_build.append((u1, v2, "down", "up"))
                slalom_connections_to_build.append((v2, u1, "up", "down"))
                slalom_connections_to_build.append((u2, v1, "up", "down"))
                slalom_connections_to_build.append((v1, u2, "down", "up"))

    for src, dst, app, ex in connections_to_build:
        add_curved_conn(src, dst, app, ex, conn_type='connection')

    # NOT: slalom_connections_to_build (karşı şeride geçme) TABAN grafa BİLİNÇLİ
    # olarak eklenmiyor — tek-yönlü yapı korunur (yoksa "ters şeritten kolay yol"
    # bug'ı geri gelir). Bunun yerine karşı-şerit crossing'leri RUNTIME'da, yalnız
    # karar `kenar_blok`/`sollama` yolladığında ve YALNIZ engelin çevresinde geçici
    # enjekte edilir (cezalı; turn-aware cusp kapısıyla sürülebilir zig-zag seçilir;
    # blok kalkınca geri alınır). Bkz. plan §16 / _slalom_enjekte / _blok_uygula.
    # Crossing listesini G üzerinde sakla (runtime _load_graph_from_import okur).
    G.graph['slalom_connections'] = slalom_connections_to_build
    G.graph['slalom_lane_segments'] = slalom_lane_segments
    # KARŞI ŞERİTLER (sollama/overtake lane'leri) = her slalom çiftinin İKİNCİ şeridi
    # (forward=ilk: A/C/E/H/K/N/M; karşı=ikinci: B/D/F/I/J/O/L). "Sağ şeride döndü mü"
    # tespiti için (HESAPLAMA KİLİDİ sağ-şerit-açma): araç bu şeritlerdeyse sollama
    # ortasında (recalc cusp üretir), değilse forward/sağ şeritte (re-plan güvenli).
    G.graph['karsi_seritler'] = {G.nodes[l2[0]]['lane']
                                 for (_l1, l2, _o) in slalom_segments if l2}

    # A şeridi ile P1 ve P7 arasındaki bağlantıyı dinamik olarak ekliyoruz
    a_nodes = [n for n in G.nodes() if G.nodes[n].get('lane') == 'A']
    if 'P1' in G.nodes() and 'P7' in G.nodes():
        p1_pos = G.nodes['P1']['pos']
        nearest_to_p1 = min(a_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(p1_pos)))
        d1 = float(np.linalg.norm(np.array(p1_pos) - np.array(G.nodes[nearest_to_p1]['pos'])))
        G.add_edge(nearest_to_p1, 'P1', weight=d1, type='connection')

        p7_pos = G.nodes['P7']['pos']
        nearest_to_p7 = min(a_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(p7_pos)))
        d7 = float(np.linalg.norm(np.array(G.nodes[nearest_to_p7]['pos']) - np.array(p7_pos)))
        G.add_edge('P7', nearest_to_p7, weight=d7, type='connection')

    # ── A ŞERİDİNE BAĞLI ESKİ DURAK CEPİNİ YENİDEN ADLANDIR (ÇAKIŞMAYI ÖNLEMEK İÇİN) ──
    old_p_nodes = [n for n in G.nodes() if G.nodes[n].get('lane') == 'P']
    relabel_map = {n: n.replace('P', 'PA') for n in old_p_nodes}
    nx.relabel_nodes(G, relabel_map, copy=False)
    for n in relabel_map.values():
        G.nodes[n]['lane'] = 'PA'

    # ── Yeni Şerit Q (Kullanıcının Verdiği Koordinatlar) ──────────────
    Q_key = [
        (8.52, -10.54),
        (7.76, -11.22),
        (7.38, -12.12),
        (7.37, -13.15),
        (7.31, -14.50),
        (7.37, -15.44),
        (7.47, -16.54),
        (8.11, -17.28)
    ]
    
    q_names = [f"Q{i+1}" for i in range(len(Q_key))]
    for i, (x, y) in enumerate(Q_key):
        G.add_node(q_names[i], pos=(x, y), type='key' if i in (0, len(Q_key)-1) else 'intermediate', lane='Q')
        
    for i in range(len(q_names) - 1):
        d = float(np.linalg.norm(np.array(Q_key[i+1]) - np.array(Q_key[i])))
        G.add_edge(q_names[i], q_names[i+1], weight=d, type='lane')
        
    # I şeridi ile dinamik bağlantıları kur
    i_nodes = [n for n in G.nodes() if G.nodes[n].get('lane') == 'I']
    if i_nodes:
        # Q1'i I şeridine bağla (Giriş) - Sadece geriden gelen (Y'si Q1'den büyük olan) düğümlerden bağla
        q1_pos = Q_key[0]
        upstream_i_nodes = [n for n in i_nodes if G.nodes[n]['pos'][1] > q1_pos[1]]
        if upstream_i_nodes:
            nearest_to_q1 = min(upstream_i_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(q1_pos)))
        else:
            nearest_to_q1 = min(i_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(q1_pos)))
        d1 = float(np.linalg.norm(np.array(q1_pos) - np.array(G.nodes[nearest_to_q1]['pos'])))
        G.add_edge(nearest_to_q1, 'Q1', weight=d1, type='connection')
        
        # Q8'i I şeridine bağla (Çıkış) - Sadece ileriye doğru giden (Y'si Q8'den küçük olan) düğümlere bağla
        q8_pos = Q_key[-1]
        downstream_i_nodes = [n for n in i_nodes if G.nodes[n]['pos'][1] < q8_pos[1]]
        if downstream_i_nodes:
            nearest_to_q8 = min(downstream_i_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(q8_pos)))
        else:
            nearest_to_q8 = min(i_nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(q8_pos)))
        d8 = float(np.linalg.norm(np.array(q8_pos) - np.array(G.nodes[nearest_to_q8]['pos'])))
        G.add_edge('Q8', nearest_to_q8, weight=d8, type='connection')

    # ── PİST GRAF GÜNCELLEMELERİ (pist_graph_A_O'dan Gelenler) ────────
    # 1. Lane A and B Extensions
    names_a_ext, vertices_a_ext, vtypes_a_ext = build_lane("A_ext", [
        (-16.09, -29.49, False),   # Will be renamed to P1
        (-16.01, -32.15, True),    # Curve point (viraj)
        (-15.77, -33.74, True),    # Curve point (viraj)
        (-15.37, -34.27, False),   # Straight point
        (-10.03, -34.27, False),   # Straight point
        (-1.95, -34.27, False)     # Will be renamed to resolved_a1
    ], closed=False, dense_spacing_non_curve=DENSE_SPACING)
    resolved_a1 = resolve_node_name("A1", G)
    names_a_ext[0] = "P1"
    names_a_ext[-1] = resolved_a1

    for name, (x, y), vt in zip(names_a_ext, vertices_a_ext, vtypes_a_ext):
        if name not in (resolved_a1, "P1"):
            G.add_node(name, pos=(x, y), type=vt, lane="A")
            
    for i in range(len(names_a_ext) - 1):
        d = float(np.linalg.norm(np.array(vertices_a_ext[i+1]) - np.array(vertices_a_ext[i])))
        G.add_edge(names_a_ext[i], names_a_ext[i+1], weight=d, type="lane")

    names_b_ext, vertices_b_ext, vtypes_b_ext = build_lane("B_ext", [
        (-1.95, -32.05, False),    # Will be renamed to resolved_b58
        (-10.03, -32.05, False),   # Straight point
        (-15.37, -32.05, False),   # Straight point
        (-15.77, -31.77, True),    # Curve point (viraj)
        (-16.01, -30.91, True),    # Curve point (viraj)
        (-16.09, -29.49, False)    # Will be renamed to P1
    ], closed=False, dense_spacing_non_curve=DENSE_SPACING)
    resolved_b58 = resolve_node_name("B58", G)
    names_b_ext[0] = resolved_b58
    names_b_ext[-1] = "P1"

    for name, (x, y), vt in zip(names_b_ext, vertices_b_ext, vtypes_b_ext):
        if name not in (resolved_b58, "P1"):
            G.add_node(name, pos=(x, y), type=vt, lane="B")
            
    for i in range(len(names_b_ext) - 1):
        d = float(np.linalg.norm(np.array(vertices_b_ext[i+1]) - np.array(vertices_b_ext[i])))
        G.add_edge(names_b_ext[i], names_b_ext[i+1], weight=d, type="lane")

    # 2. Park Road (Bidirectional, 8 nodes)
    y_coords = np.linspace(-29.49, -13.85, 8)
    park_nodes = []
    for idx, y in enumerate(y_coords):
        node_name = f"P{idx+1}"
        pos = (-16.09, round(y, 2))
        park_nodes.append((node_name, pos))
        vt = 'key' if (idx == 0 or idx == 7) else 'intermediate'
        G.add_node(node_name, pos=pos, type=vt, lane="P")
        
    for i in range(7):
        u, pos_u = park_nodes[i]
        v, pos_v = park_nodes[i+1]
        d = float(np.linalg.norm(np.array(pos_u) - np.array(pos_v)))
        G.add_edge(u, v, weight=d, type="lane")
        G.add_edge(v, u, weight=d, type="lane")

    # 3. Parking Slots (8 slots, 16 nodes, bidirectional)
    y_spots = np.linspace(-29.44, -13.92, 8)
    for idx, y in enumerate(y_spots):
        spot_num = idx + 1
        pos_1 = (-18.76, round(y, 2))
        pos_2 = (-21.78, round(y, 2))
        
        name_1 = f"Spot_{spot_num}_1"
        name_2 = f"Spot_{spot_num}_2"
        
        G.add_node(name_1, pos=pos_1, type='key', lane='Spot')
        G.add_node(name_2, pos=pos_2, type='key', lane='Spot')
        
        d_spot = float(np.linalg.norm(np.array(pos_1) - np.array(pos_2)))
        G.add_edge(name_1, name_2, weight=d_spot, type='lane')
        G.add_edge(name_2, name_1, weight=d_spot, type='lane')
        
        name_p = f"P{spot_num}"
        pos_p = G.nodes[name_p]['pos']
        d_conn = float(np.linalg.norm(np.array(pos_1) - np.array(pos_p)))
        G.add_edge(name_p, name_1, weight=d_conn, type='connection')
        G.add_edge(name_1, name_p, weight=d_conn, type='connection')

    # 4. Connections between extensions and park road (using curves)
    # Direct U-turn at the end of extensions (Turkish traffic: loops left/westward)
    # Removed manually added curve here because build_lane already creates a natural, non-overlapping loop at P1.

    # 5. Horizontal Parking Lanes & Vertical Corridor (from the 4 empty nodes)
    empty_points = [
        ("empty_1", (-7.17, -13.99)),
        ("empty_2", (-9.97, -13.99)),
        ("empty_3", (-7.17, -16.36)),
        ("empty_4", (-9.97, -16.36))
    ]
    for name, pos in empty_points:
        G.add_node(name, pos=pos, type="key", lane="P")  # Added as part of Lane P (Park Yolu)

    # Yatay segmentler (Tek yönlü, kendi aralarında dikey bağlantı/kare oluşturulmadı)
    # Üst Sol: empty_2 -> P8 (Tek yönlü, sola doğru)
    d_e2_p8 = float(np.linalg.norm(np.array(G.nodes['P8']['pos']) - np.array(G.nodes['empty_2']['pos'])))
    G.add_edge('empty_2', 'P8', weight=d_e2_p8, type='lane')

    # Üst Sağ: empty_1 -> empty_2 (Tek yönlü, sola doğru)
    d_e1_e2 = float(np.linalg.norm(np.array(G.nodes['empty_1']['pos']) - np.array(G.nodes['empty_2']['pos'])))
    G.add_edge('empty_1', 'empty_2', weight=d_e1_e2, type='lane')

    # Alt Sol: P7 -> empty_4 (Tek yönlü, sağa doğru)
    d_p7_e4 = float(np.linalg.norm(np.array(G.nodes['P7']['pos']) - np.array(G.nodes['empty_4']['pos'])))
    G.add_edge('P7', 'empty_4', weight=d_p7_e4, type='lane')

    # Alt Sağ: empty_4 -> empty_3 (Tek yönlü, sağa doğru)
    d_e4_e3 = float(np.linalg.norm(np.array(G.nodes['empty_4']['pos']) - np.array(G.nodes['empty_3']['pos'])))
    G.add_edge('empty_4', 'empty_3', weight=d_e4_e3, type='lane')

    # 6. Connecting Horizontal Parking Lanes to Lanes A & B (using curves)
    # Top road entrance from B: B68 (flowing up) -> empty_1 (Virajı düzeltmek için B68'den bağlandı, sola yönü destekler)
    add_curved_conn("B68", "empty_1", "up", "left", n_mid=3)
    # Top road entrance from A: A76 (flowing down) -> empty_1 (Şerit A'dan sola dönerek üst yola giriş)
    add_curved_conn("A76", "empty_1", "down", "left", n_mid=3)

    # Bottom road exit to A: empty_3 -> A79 (flowing down, sağa giden yolun çıkışıdır)
    add_curved_conn("empty_3", "A79", "right", "down", n_mid=3)
    # Bottom road exit to B: empty_3 -> B69 (flowing up, sağa giden yoldan yukarı Şerit B'ye çıkış)
    add_curved_conn("empty_3", "B69", "right", "up", n_mid=3)

    # 7. Sol alt otopark giriş/çıkış virajları (Simetrik bağlantılar)
    # Giriş Virajı: Şerit A dikey -> B_ext yatay
    a_nodes_ent = [n for n in G.nodes() if G.nodes[n].get('lane') == 'A' and not n.startswith("A_ext")]
    src_entrance = min(a_nodes_ent, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array([-5.21, -29.14])))
    b_ext_nodes_ent = [n for n in G.nodes() if n.startswith("B_ext")]
    dst_entrance = min(b_ext_nodes_ent, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array([-7.00, -32.05])))
    add_curved_conn(src_entrance, dst_entrance, "down", "left", n_mid=15)

    # Çıkış Virajı: A_ext yatay -> Şerit B dikey
    a_ext_nodes_ex = [n for n in G.nodes() if n.startswith("A_ext")]
    src_exit = min(a_ext_nodes_ex, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array([-6.00, -34.27])))
    b_nodes_ex = [n for n in G.nodes() if G.nodes[n].get('lane') == 'B' and not n.startswith("B_ext")]
    dst_exit = min(b_nodes_ex, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array([-2.96, -29.14])))
    add_curved_conn(src_exit, dst_exit, "right", "up", n_mid=23)

    return G


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import networkx as nx

    print("=== TALOS Graph Builder Standalone Mode ===")
    print("Building track graph...")
    G = build_track_graph()
    print(f"Success! Graph built with {len(G.nodes)} nodes and {len(G.edges)} edges.")

    print("\nVisualizing graph... Close the plot window to exit.")
    
    # Setup dark premium visual style matching the application's visualizer.py
    BG = '#2e2d2a'
    PANEL_BG = '#272622'
    EDGE_COL = '#3d3c38'
    NODE_COL = '#4a4945'
    ROTA_MAIN = '#ff4d5e'
    DURAK_COL = '#00d9ff'
    
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(PANEL_BG)
    ax.set_aspect('equal')
    
    pos = nx.get_node_attributes(G, 'pos')
    
    # Draw edges
    for u, v, edge_data in G.edges(data=True):
        p1 = pos[u]
        p2 = pos[v]
        etype = edge_data.get('type', 'lane')
        if etype == 'connection':
            ecol = '#4e79a7'  # Blue for connections
            ew = 0.8
            alpha = 0.6
        else:
            ecol = EDGE_COL
            ew = 0.5
            alpha = 0.8
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=ecol, alpha=alpha, linewidth=ew, zorder=1)

    # Draw slalom connections (dashed green lines)
    slalom_conns = G.graph.get('slalom_connections', [])
    for src, dst, _, _ in slalom_conns:
        if src in pos and dst in pos:
            p1 = pos[src]
            p2 = pos[dst]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#2ca02c', alpha=0.3, linestyle='--', linewidth=0.5, zorder=1)

    # Draw nodes grouped by type
    x_nodes = [p[0] for p in pos.values()]
    y_nodes = [p[1] for p in pos.values()]
    ax.scatter(x_nodes, y_nodes, c=NODE_COL, s=8, alpha=0.5, zorder=2, edgecolors='none')
    
    # Highlight connection and curve (viraj) nodes
    conn_nodes_pos = [pos[n] for n, d in G.nodes(data=True) if d.get('type') == 'connection']
    viraj_nodes_pos = [pos[n] for n, d in G.nodes(data=True) if d.get('type') == 'viraj']
    giris_nodes_pos = [pos[n] for n, d in G.nodes(data=True) if d.get('type') == 'giris']
    
    if conn_nodes_pos:
        cx, cy = zip(*conn_nodes_pos)
        ax.scatter(cx, cy, c='#4e79a7', s=18, alpha=0.9, zorder=3, label='Bağlantı Noktası')
    if viraj_nodes_pos: 

        vx, vy = zip(*viraj_nodes_pos)
        ax.scatter(vx, vy, c='#f28e2b', s=18, alpha=0.9, zorder=3, label='Viraj')
    if giris_nodes_pos:
        gx, gy = zip(*giris_nodes_pos)
        ax.scatter(gx, gy, c='#ff4d5e', s=24, alpha=1.0, zorder=4, label='Giriş')

    # Add dummy line for Slalom in legend
    ax.plot([], [], color='#2ca02c', linestyle='--', alpha=0.6, label='Slalom Geçişi (Sollama)')

    ax.axis('off')
    plt.title("TALOS Yol Grafları Yapısı", color='#7a7a6e', fontfamily='monospace', fontsize=12)
    plt.legend(facecolor='#1e1d1b', edgecolor='#3d3c38', labelcolor='#7a7a6e')
    plt.tight_layout()
    
    # Save the visualization to a file so it can be viewed when running in Docker/headless
    output_filename = "graph_output.png"
    plt.savefig(output_filename, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Graph visualization saved to {output_filename}")
    
    try:
        plt.show()
    except Exception as e:
        print(f"Could not open GUI window (headless/docker environment): {e}")