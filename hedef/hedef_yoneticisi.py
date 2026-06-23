#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import math
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
import heapq
import networkx as nx

# Kalıcı tanı logu (opsiyonel) — import edilemezse node yine çalışır.
try:
    from hedef_logger import HedefLogger
except Exception as _e:  # noqa: BLE001
    HedefLogger = None
    sys.stderr.write(f"[hedef_yoneticisi] hedef_logger yok, loglama kapalı: {_e}\n")

YESIL  = "\033[92m"
KIRMIZI = "\033[91m"
SARI   = "\033[93m"
SIFIRLA = "\033[0m"

GOREV_GEOJSON = {
  "type": "FeatureCollection",
  "features": [
    {"type": "Feature", "properties": {"name": "durak_1", "description": "1. Durak (Eski Park Cebi - Sağ)"},
     "geometry": {"type": "Point", "coordinates": [37.0, -4.5]}},
    {"type": "Feature", "properties": {"name": "durak_2", "description": "2. Durak (Yeni Park Yolu - Sol)"},
     "geometry": {"type": "Point", "coordinates": [7.35, -13.9]}},
    {"type": "Feature", "properties": {"name": "durak_1_donus", "description": "Ara durak_1 (park dönüşü için tekrar)"},
     "geometry": {"type": "Point", "coordinates": [37.0, -4.5]}},
    {"type": "Feature", "properties": {"name": "park", "description": "Park (Spot lane, demo node 7)"},
     "geometry": {"type": "Point", "coordinates": [-21.78, -13.92]}}
  ]
}

# Sapma kontrolü için eşik değerleri
ILERI_MESAFE_M     = 2.0   # start seçimi: aracın yaw yönünde bu kadar ileriye
                           # sanal nokta atılır → start o noktaya en yakın düğüm
                           # (Samed'in eski sürümündeki yaw forward-projection).
                           # 5.0 → 2.0: 5m'lik ileri-projeksiyon hem yanlış paralel şeride
                           # snap'e hem de aşağıdaki sapma-eşiği şişmesine yol açıyordu.
SAPMA_ESIK_METRE   = 2.5   # FAZ5: sapma aracın BURUN noktasından (yaw yönünde ILERI_MESAFE_M
                           # ileride) en yakın WP'ye ölçülür. Burun zaten rotada (start o noktaya
                           # en yakın seçildiğinden) → on-route'ta mesafe ~0; bu yüzden eşik sıkı
                           # (2.5m) olabilir ve döngü KENDİLİĞİNDEN kapanır (4.5m'lik artifact+
                           # tolerans şişmesine gerek yok). Yön-bilinçli: araç rotaya dönükse
                           # burun rotaya yakın kalır → hemen kopmaz.
SAPMA_DEBOUNCE_SURE = 1.5  # FAZ2: sapma reroute tetiklemeden önce eşiği bu kadar saniye
                           # sürmeli. Anlık konum sıçraması/tek-kare gürültü reroute
                           # etmesin → kararlılık (literatürdeki minimum-dwell/debounce).
SAPMA_TEMIZ_METRE  = 1.5   # FAZ5 histerezis clear (band 1.5..2.5m). on-route burun-mesafesi
                           # ~1m < 1.5 → sayaç sıfırlanır. Eşik etrafında salınan (flapping)
                           # araçta sayacın sıfırlanıp reroute'un hiç tetiklenmemesini önler.
GOREV_YAKINLIK_M   = 2.0   # bu mesafede görevi tamamlandı sayar (5.0'dan düşürüldü)
MATCH_KORIDOR_M    = 4.5   # FAZ3 map-match snap koridoru — sapma eşiğinden AYRI (FAZ5'te ayrıldı).
                           # Grafın en uzun kenarı 6.13m (park yolu) → kenar ortasında en yakın
                           # düğüm 3.06m uzakta; koridor bundan büyük olmalı, yoksa o kenarların
                           # ortasında map-match snap edemez. Sapma eşiği (2.5m) bunu karşılamaz.
MATCH_PENCERE      = 6     # FAZ3 map-matching: current_wp_index'i ileri pencerede (bu kadar
                           # WP) en yakın rota noktasına snap'le. Tek-tek +1 yerine; geri
                           # zıplama yok (pencere ileri başlar). Snap koridoru MATCH_KORIDOR_M:
                           # nokta o koridorun dışındaysa off-route'tur, snap yapılmaz.
YON_FILTRE_ACIISI  = math.pi - 0.3   # geri yön filtresi açısı (162 derece)

# Görselleştirme Arayüzü (GUI) Ayarı
ENABLE_GUI         = True  # Matplotlib penceresini açmak/kapatmak için (False = Headless/Penceresiz)

# Ağırlık ve Ceza Katsayıları (Yol ve Şerit Tercihleri)
AGIRLIK_LANE_DUZ         = 1.0   # Düz şeritte normal sürüş çarpanı
AGIRLIK_LANE_TERS_MULT   = 150.0 # Ters şeritte sürüş çarpanı (geriye doğru yol planlamasını önlemek için çok yüksek)
AGIRLIK_LANE_TERS_P      = 0.0   # Ters şeritte sürüş sabit cezası (iptal)

AGIRLIK_SLALOM_MULT      = 6.0   # Şerit değiştirme (slalom) kat sayısı
AGIRLIK_SLALOM_P         = 0.0   # Şerit değiştirme (slalom) sabit cezası (iptal)

AGIRLIK_CONN_DUZ         = 1.0   # Bağlantı yollarında normal geçiş çarpanı
AGIRLIK_CONN_TERS_MULT   = 150.0 # Bağlantı yollarında ters yön çarpanı (geriye doğru yol planlamasını önlemek için çok yüksek)
AGIRLIK_CONN_TERS_P      = 0.0   # Bağlantı yollarında ters yön sabit cezası (iptal)


# ==========================================
#   D* LITE PLANNER
# ==========================================
class DLitePlanner:
    def __init__(self):
        self.adj_list: dict[tuple, list] = {}
        self.pred_list: dict[tuple, list] = {}  # Yönlendirilmiş graf için ters komşuluk listesi
        self.edge_weights: dict[tuple[tuple, tuple], float] = {}
        self.node_types: dict[tuple, str] = {}
        self.nodes: set[tuple] = set()
        self.g:   dict[tuple, float] = {}
        self.rhs: dict[tuple, float] = {}
        self.U:   list = []
        self.s_start = None
        self.s_goal  = None

    # ── Graph yönetimi ──────────────────────────────────────────────
    def add_edge(self, p1: tuple, p2: tuple, weight: float = None) -> None:
        self.adj_list.setdefault(p1, [])
        if p2 not in self.adj_list[p1]:
            self.adj_list[p1].append(p2)
        
        self.pred_list.setdefault(p2, [])
        if p1 not in self.pred_list[p2]:
            self.pred_list[p2].append(p1)

        self.nodes.add(p1)
        self.nodes.add(p2)
        if weight is not None:
            self.edge_weights[(p1, p2)] = weight
        else:
            self.edge_weights[(p1, p2)] = self.dist(p1, p2)

    def get_cost(self, u: tuple, v: tuple) -> float:
        return self.edge_weights.get((u, v), self.dist(u, v))

    def remove_edge_directed(self, src: tuple, dst: tuple) -> bool:
        """Tek yönlü kenar siler; kenar yoksa False döner."""
        try:
            self.adj_list[src].remove(dst)
            if dst in self.pred_list:
                try:
                    self.pred_list[dst].remove(src)
                except ValueError:
                    pass
            return True
        except (KeyError, ValueError):
            return False

    def restore_edge_directed(self, src: tuple, dst: tuple) -> None:
        """Tek yönlü kenarı geri ekler (varsa tekrar eklemez)."""
        self.adj_list.setdefault(src, [])
        if dst not in self.adj_list[src]:
            self.adj_list[src].append(dst)
        
        self.pred_list.setdefault(dst, [])
        if src not in self.pred_list[dst]:
            self.pred_list[dst].append(src)

    # ── Yardımcılar ─────────────────────────────────────────────────
    @staticmethod
    def dist(p1: tuple, p2: tuple) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def calculate_key(self, s: tuple) -> tuple:
        # NOT: Klasik D* Lite'taki km (key modifier) burada YOK. km yalnızca start
        # hareket ederken g/rhs'yi koruyup artımlı replan yapan sürümde gerekir.
        # Bu planlayıcı her find_path'te sıfırdan arıyor (aşağıdaki açıklamaya bak),
        # dolayısıyla km her zaman 0 olurdu → ölü terim, kaldırıldı (Faz4-lite).
        g_val  = self.g.get(s,   float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        min_val = min(g_val, rhs_val)
        return (min_val + self.dist(self.s_start, s), min_val)

    # ── Çekirdek D* Lite ────────────────────────────────────────────
    def update_vertex(self, u: tuple) -> None:
        if u != self.s_goal:
            min_rhs = float('inf')
            for nb in self.adj_list.get(u, []):
                val = self.get_cost(u, nb) + self.g.get(nb, float('inf'))
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u] = min_rhs
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self) -> None:
        """
        Orijinal D* Lite erken çıkış koşulu:
          U boş DEĞİLSE ve
          (U.top_key < calculate_key(s_start)) VEYA rhs[s_start] != g[s_start]
        koşulu sağlandığı sürece çalış.
        """
        visited: set[tuple] = set()
        while self.U:
            # Heap'in güncel tepesine bak (pop etme)
            top_key, _ = self.U[0]
            start_key   = self.calculate_key(self.s_start)

            # Erken çıkış: s_start çözüldü ve heap'te daha iyi düğüm yok
            if (top_key >= start_key and
                    self.rhs.get(self.s_start, float('inf')) ==
                    self.g.get(self.s_start, float('inf'))):
                break

            k_old, u = heapq.heappop(self.U)
            if u in visited:
                continue

            k_new = self.calculate_key(u)
            if k_old < k_new:          # anahtar eskimiş → yeniden ekle
                heapq.heappush(self.U, (k_new, u))
                continue

            visited.add(u)
            if self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.pred_list.get(u, []):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.pred_list.get(u, []):
                    self.update_vertex(s)

    def find_path(self, start: tuple, goal: tuple):
        if start not in self.adj_list or goal not in self.adj_list:
            rospy.logwarn(f"[D*Lite] start veya goal graph'ta yok! "
                          f"start:{start} goal:{goal}")
            return None

        # ── Soğuk (sıfırdan) arama — bilinçli tercih (Faz4-lite) ────────
        # Klasik D* Lite g/rhs/U/km'yi replanlar arası korur ve km += h ile
        # start'ı kaydırarak artımlı (ucuz) replan yapar. Burada her find_path
        # sıfırdan arıyor. Gerekçe: (1) graf statik ve küçük (644 düğüm) — ölçüm:
        # cold find_path medyan 0.88ms / p95 3.18ms (20Hz=50ms bütçe içinde önemsiz);
        # (2) ileri-yön filtresi her replan'da start kenarlarını geçici silip geri
        # ekliyor → artımlı durumu korumak her seferinde update_vertex yayılımı
        # gerektirir, küçük kazancı yer ve risk getirir. Bu yüzden artımlı
        # sürüme geçilmedi (Faz4 atlandı).
        self.s_start = start
        self.s_goal  = goal
        self.U  = []
        self.g  = {}
        self.rhs = {}
        self.rhs[self.s_goal] = 0.0
        heapq.heappush(self.U, (self.calculate_key(self.s_goal), self.s_goal))
        self.compute_shortest_path()

        if self.g.get(self.s_start, float('inf')) == float('inf'):
            rospy.logwarn("[D*Lite] Yol bulunamadı (g=inf)")
            return None

        # Yolu geri iz sür
        path  = [self.s_start]
        curr  = self.s_start
        seen  = {self.s_start}              # O(1) döngü tespiti
        max_steps = len(self.nodes) + 10

        for _ in range(max_steps):
            if curr == self.s_goal:
                break
            neighbors = self.adj_list.get(curr, [])
            if not neighbors:
                rospy.logwarn("[D*Lite] Çıkışsız düğüme ulaşıldı.")
                return None             # yarım rota gönderme

            best_next = min(
                neighbors,
                key=lambda n: self.get_cost(curr, n) + self.g.get(n, float('inf'))
            )

            if best_next in seen:
                rospy.logwarn("[D*Lite] Döngü tespit edildi — rota geçersiz.")
                return None             # yarım rota yerine None dön

            seen.add(best_next)
            path.append(best_next)
            curr = best_next

        return path if len(path) > 1 and curr == self.s_goal else None


# ==========================================
#   GRAF YAPISI OLUŞTURUCU (PİST GRAFI)
# ==========================================
def build_track_graph():
    import networkx as nx
    import numpy as np

    SPACING = 2.0
    MIN_DENSIFY = 2.5

    def densify_segment(p1, p2, spacing):
        d = np.linalg.norm(np.array(p2) - np.array(p1))
        n = int(round(d / spacing))
        if n <= 1:
            return []
        t_vals = np.linspace(0, 1, n + 1)[1:-1]
        return [(round(p1[0] + t*(p2[0]-p1[0]), 2),
                 round(p1[1] + t*(p2[1]-p1[1]), 2)) for t in t_vals]

    def build_lane(prefix, key_points, closed=True, densify=True):
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
                if not both_special and d >= MIN_DENSIFY:
                    for pt in densify_segment((x1, y1), (x2, y2), SPACING):
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

    G = nx.DiGraph()

    lanes = {
        "A": build_lane("A", A_key, closed=True),
        "B": build_lane("B", B_key, closed=True),
        "C": build_lane("C", C_key, closed=False),
        "D": build_lane("D", D_key, closed=False),
        "E": build_lane("E", E_key, closed=False),
        "F": build_lane("F", F_key, closed=False),
        "G": build_lane("G", G_key, closed=True, densify=False),
        "H": build_lane("H", H_key, closed=False),
        "I": build_lane("I", I_key, closed=False),
        "J": build_lane("J", J_key, closed=False),
        "K": build_lane("K", K_key, closed=False),
        "L": build_lane("L", L_key, closed=False),
        "M": build_lane("M", M_key, closed=False),
        "N": build_lane("N", N_key, closed=False),
        "O": build_lane("O", O_key, closed=False),
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
        p0 = np.array(G.nodes[src]['pos'])
        p2 = np.array(G.nodes[dst]['pos'])
        if approach_dir in ('right', 'left') and exit_dir in ('up', 'down'):
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
            node_name = f"X_{src}_{dst}_{idx+1}"
            G.add_node(node_name, pos=pt, type=conn_type, lane='X')
            mid_nodes.append(node_name)
        prev = src
        for node in mid_nodes:
            d = float(np.linalg.norm(np.array(G.nodes[node]['pos']) - np.array(G.nodes[prev]['pos'])))
            G.add_edge(prev, node, weight=d, type=conn_type)
            prev = node
        d = float(np.linalg.norm(np.array(G.nodes[dst]['pos']) - np.array(G.nodes[prev]['pos'])))
        G.add_edge(prev, dst, weight=d, type=conn_type)

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
        (-15.37, -34.27, False),
        (-10.03, -34.27, False),
        (-1.95, -34.27, False)
    ], closed=False)
    names_a_ext[-1] = "A1"

    for name, (x, y), vt in zip(names_a_ext, vertices_a_ext, vtypes_a_ext):
        if name != "A1":
            G.add_node(name, pos=(x, y), type=vt, lane="A")
            
    for i in range(len(names_a_ext) - 1):
        d = float(np.linalg.norm(np.array(vertices_a_ext[i+1]) - np.array(vertices_a_ext[i])))
        G.add_edge(names_a_ext[i], names_a_ext[i+1], weight=d, type="lane")

    names_b_ext, vertices_b_ext, vtypes_b_ext = build_lane("B_ext", [
        (-1.95, -32.05, False),
        (-10.03, -32.05, False),
        (-15.37, -32.05, False)
    ], closed=False)
    names_b_ext[0] = "B58"

    for name, (x, y), vt in zip(names_b_ext, vertices_b_ext, vtypes_b_ext):
        if name != "B58":
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
    b_ext_last = names_b_ext[-1]  # Node at (-15.37, -32.05)
    a_ext_first = names_a_ext[0]  # Node at (-15.37, -34.27)
    
    # Bottom Entrance & Exit curved connections
    add_curved_conn(b_ext_last, "P1", "left", "up", n_mid=3)
    add_curved_conn("P1", a_ext_first, "down", "right", n_mid=3)

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

    return G



# ==========================================
#   YÖNETİCİ SINIFI
# ==========================================
class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')

        # ── Görselleştirme verisi ────────────────────────────────────
        self.new_data_available = False

        # ── Robot durumu ─────────────────────────────────────────────
        self.robot_x         = None   # None: henüz konum gelmedi
        self.robot_y         = None
        self.robot_yaw       = None
        self._ilk_konum_alindi = False  # FIX: ilk konuma kadar görev kontrolü yapma

        # ── Rota durumu ──────────────────────────────────────────────
        self.full_path_world     = []
        self.current_wp_index    = 0
        self.current_task_index  = 0
        self.is_path_calculated  = False
        self.geo_targets_world   = []
        self.geo_targets_built   = False  # FIX: tek seferlik build kontrolü
        self._graph_loaded       = False

        # ── Zamanlayıcılar ───────────────────────────────────────────
        # FIX: 0.0 yerine time.time() → node başlar başlamaz cooldown aktif
        self.son_hesaplama_zamani = time.time()
        self._son_varildi_zamani  = time.time()
        self._son_gorev_zamani    = time.time()
        # FAZ2: sapmanın eşiği kesintisiz aştığı ilk an (debounce); eşik altına
        # düşünce None'a sıfırlanır. reroute ancak bu süre >= SAPMA_DEBOUNCE_SURE olunca.
        self._sapma_baslangic     = None

        # ── Thread güvenliği ─────────────────────────────────────────
        # FIX: varildi_callback & konum_callback çakışmasını önler
        self._wp_lock = threading.Lock()

        # ── Tanı logu (kalıcı; docker kapanınca host'ta kalır) ───────
        self.logger = None
        if HedefLogger is not None:
            try:
                self.logger = HedefLogger()
                # SIGTERM/shutdown'da tamponları flush et (son satırlar kaybolmasın)
                rospy.on_shutdown(lambda: self.logger.close() if self.logger else None)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"[hedef_yoneticisi] logger başlatılamadı: {e}\n")

        # ── Planner ─────────────────────────────────────────────────
        self.planner = DLitePlanner()
        self._load_graph_from_import()

        # ── Görselleştirme ───────────────────────────────────────────
        if ENABLE_GUI:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.ax.set_aspect('equal')
            plt.show(block=False)
        self._static_drawn = False
        self.line_rota_past = None
        self.line_rota_glow = None
        self.line_rota_main = None
        self.line_rota_shin = None
        self.scatter_wp1 = None
        self.scatter_wp2 = None
        self.scatter_car_glow = None
        self.scatter_car = None
        self.arrow_car = None

        # ── ROS bağlantıları ─────────────────────────────────────────
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10)

        rospy.Subscriber('/konum',         Pose2D,        self.konum_callback)
        rospy.Subscriber('/gorev_durumu',  String,        self.varildi_callback)

        self.new_data_available = True
        print(f"{YESIL}>>> SİSTEM HAZIR. Bekleniyor: /konum{SIFIRLA}")

    # ==========================================
    #   GRAF YÜKLEME
    # ==========================================
    def _load_graph_from_import(self) -> None:
        rospy.loginfo("[hedef_yoneticisi] Graf yapısı oluşturuluyor...")
        try:
            G = build_track_graph()
            self.G = G
        except Exception as e:
            rospy.logerr(f"[hedef_yoneticisi] Graf oluşturulamadı: {e}")
            return

        self.planner.adj_list.clear()
        self.planner.pred_list.clear()
        self.planner.edge_weights.clear()
        self.planner.node_types.clear()
        self.planner.nodes.clear()
        self.pos_to_node = {}

        node_to_pos = {}
        for node_name, data in G.nodes(data=True):
            pos = data.get('pos')
            if pos is not None:
                node_to_pos[node_name] = (pos[0], pos[1])
                self.planner.node_types[(pos[0], pos[1])] = data.get('type', 'intermediate')
                self.pos_to_node[(pos[0], pos[1])] = node_name

        for u, v, edge_data in G.edges(data=True):
            p1 = node_to_pos.get(u)
            p2 = node_to_pos.get(v)
            if not p1 or not p2 or p1 == p2:
                continue

            # Base distance
            d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            etype = edge_data.get('type', 'lane')

            # Calculate forward weight
            if etype == 'lane':
                w_forward = d * AGIRLIK_LANE_DUZ
            elif etype == 'connection':
                w_forward = d * AGIRLIK_CONN_DUZ
            elif etype == 'slalom':
                w_forward = d * AGIRLIK_SLALOM_MULT + AGIRLIK_SLALOM_P
            else:
                w_forward = d

            # Add forward edge
            self.planner.add_edge(p1, p2, w_forward)

            # Check if reverse edge exists in G. If not, do NOT add penalized reverse edge for overtaking
            # (Orijinal tek yönlü DiGraph yapısını korumak ve ters yön planlamasını önlemek için devre dışı bırakıldı)
            # if not G.has_edge(v, u):
            #     if etype == 'lane':
            #         w_reverse = d * AGIRLIK_LANE_TERS_MULT + AGIRLIK_LANE_TERS_P
            #     elif etype == 'connection':
            #         w_reverse = d * AGIRLIK_CONN_TERS_MULT + AGIRLIK_CONN_TERS_P
            #     elif etype == 'slalom':
            #         w_reverse = d * AGIRLIK_SLALOM_MULT + AGIRLIK_SLALOM_P
            #     else:
            #         w_reverse = d * 10.0
            # 
            #     self.planner.add_edge(p2, p1, w_reverse)

        self._graph_loaded = True
        rospy.loginfo(
            f"[graph] Graf yapısından {len(self.planner.nodes)} düğüm başarıyla yüklendi."
        )

        if not self.geo_targets_built:
            self._build_geo_targets()

    def _build_geo_targets(self) -> None:
        """Her görev koordinatını graph'taki en yakın düğüme snap'ler."""
        self.geo_targets_world.clear()
        for feature in GOREV_GEOJSON['features']:
            coords = feature['geometry']['coordinates']
            tx, ty = coords[0], coords[1]
            nearest = min(
                self.planner.nodes,
                key=lambda n: (n[0] - tx)**2 + (n[1] - ty)**2
            )
            snap = math.hypot(nearest[0] - tx, nearest[1] - ty)
            print(f"{YESIL}>>> [{feature['properties']['name']}] "
                  f"snap:{snap:.2f}m → {nearest}{SIFIRLA}")
            self.geo_targets_world.append(nearest)

        self.geo_targets_built = True

        # İlk konum zaten geldiyse hemen rota hesapla
        if self._ilk_konum_alindi and self.robot_x is not None:
            self.recalculate_path_from_robot()

    # ==========================================
    #   CALLBACKLER
    # ==========================================
    def konum_callback(self, msg: Pose2D) -> None:
        self.robot_x   = msg.x
        self.robot_y   = msg.y
        self.robot_yaw = msg.theta

        if self.robot_x is None or self.robot_y is None:
            return

        # ── FIX: İlk konum alındığında cooldown'ları sıfırla ────────
        if not self._ilk_konum_alindi:
            self._ilk_konum_alindi = True
            now = time.time()
            self._son_gorev_zamani    = now
            self._son_varildi_zamani  = now
            self.son_hesaplama_zamani = now
            rospy.loginfo(f"[konum] İlk konum alındı: "
                          f"({self.robot_x:.1f}, {self.robot_y:.1f})")

        # ── Rota yoksa hesapla ───────────────────────────────────────
        if (not self.is_path_calculated
                and self.planner.nodes
                and self.geo_targets_world):
            self.recalculate_path_from_robot(reason="ilk_rota")

        if not self.is_path_calculated or not self.full_path_world:
            self.new_data_available = True
            return

        now = time.time()

        # ── Otomatik WP geçişi: hafif map-matching ──────────────────
        # FAZ3: tek-tek +1 yerine, aracın rotadaki yerini İLERİ pencerede
        # [idx, idx+MATCH_PENCERE) en yakın noktaya snap'le (geri zıplama yok,
        # pencere ileri başlar). Sadece koridor içindeyse (best_d < MATCH_KORIDOR_M)
        # ilerlet; dışındaysa off-route → snap yapma, sapma/reroute mantığı halleder.
        wp_gecis_log = None   # lock içinde doldurulur, log lock DIŞINDA atılır
        with self._wp_lock:
            n_path = len(self.full_path_world)
            ust    = min(self.current_wp_index + MATCH_PENCERE, n_path)
            best_i = self.current_wp_index
            best_d = math.hypot(self.robot_x - self.full_path_world[best_i][0],
                                self.robot_y - self.full_path_world[best_i][1])
            for i in range(self.current_wp_index + 1, ust):
                wx_wp, wy_wp = self.full_path_world[i]
                d = math.hypot(self.robot_x - wx_wp, self.robot_y - wy_wp)
                if d < best_d:
                    best_d = d
                    best_i = i

            if (best_i > self.current_wp_index
                    and best_i <= n_path - 1
                    and best_d < MATCH_KORIDOR_M):
                atlanan = best_i - self.current_wp_index
                self.current_wp_index = best_i
                self._son_varildi_zamani = now
                rospy.loginfo(f"[OTO] WP {self.current_wp_index} "
                              f"(map-match +{atlanan}, d:{best_d:.1f}m)")
                wp_gecis_log = dict(wp_idx=self.current_wp_index,
                                    n_path=n_path,
                                    dist=round(best_d, 2),
                                    atlanan=atlanan,
                                    task_idx=self.current_task_index)

        # Disk I/O'yu _wp_lock dışında yap (lock contention'ı önler)
        if wp_gecis_log is not None and self.logger is not None:
            self.logger.log_event("wp_gecis", **wp_gecis_log)

        # ── Ana hedef (durak) kontrolü ───────────────────────────────
        if self.current_task_index < len(self.geo_targets_world):
            wx_g, wy_g = self.geo_targets_world[self.current_task_index]
            dist_to_goal = math.hypot(self.robot_x - wx_g, self.robot_y - wy_g)

            if dist_to_goal < GOREV_YAKINLIK_M and now - self._son_gorev_zamani > 5.0:
                self._son_gorev_zamani = now
                self.current_task_index += 1

                if self.current_task_index >= len(self.geo_targets_world):
                    print(f"{YESIL}>>> TÜM GÖREVLER TAMAMLANDI!{SIFIRLA}")
                    self.is_path_calculated = False
                    self.full_path_world    = []
                    self.new_data_available = True
                    return

                next_name = GOREV_GEOJSON['features'][self.current_task_index]['properties']['name']
                print(f"{YESIL}>>> DURAK TAMAMLANDI! Yeni hedef: {next_name}{SIFIRLA}")
                if self.logger is not None:
                    self.logger.log_event("gorev_tamam", task_idx=self.current_task_index,
                                          next_name=next_name,
                                          robot=[round(self.robot_x, 2), round(self.robot_y, 2)])
                self.recalculate_path_from_robot(reason="durak_tamamlandi")

        # ── Sapma kontrolü ───────────────────────────────────────────
        # FIX: Tüm rota yerine sadece yakındaki WP'lere bak (CPU tasarrufu)
        lookahead = self.full_path_world[self.current_wp_index:
                                         self.current_wp_index + 20]
        if lookahead:
            # ── FAZ5: sapmayı aracın BURUN noktasından ölç ──────────────
            # Burun = robot + ILERI_MESAFE_M * yaw yönü (start seçiminde kullanılan
            # nokta). Böylece on-route'ta mesafe ~0 (döngü kendiliğinden kapanır) ve
            # ölçüm yön-bilinçli: araç rotaya dönükse burun rotaya yakın → kopmaz.
            if self.robot_yaw is not None:
                burun_x = self.robot_x + ILERI_MESAFE_M * math.cos(self.robot_yaw)
                burun_y = self.robot_y + ILERI_MESAFE_M * math.sin(self.robot_yaw)
            else:
                burun_x, burun_y = self.robot_x, self.robot_y
            min_dist = min(
                math.hypot(burun_x - wx, burun_y - wy)
                for wx, wy in lookahead
            )
            # ── FAZ2: debounce + histerezis (Schmitt-trigger) ───────────
            # Sapma SAPMA_DEBOUNCE_SURE boyunca sürmeli. Eşik etrafında salınan
            # (flapping) araçta sayaç sıfırlanmasın diye: sayaç SAPMA_ESIK üstünde
            # KURULUR, ancak SAPMA_TEMIZ ALTINA inince SIFIRLANIR. Ara bantta
            # (TEMIZ..ESIK) sayaca dokunulmaz → kenarda süren araç da tetikler.
            if min_dist > SAPMA_ESIK_METRE:
                if self._sapma_baslangic is None:
                    self._sapma_baslangic = now
            elif min_dist < SAPMA_TEMIZ_METRE:
                self._sapma_baslangic = None
            sapma_sureli = (self._sapma_baslangic is not None
                            and (now - self._sapma_baslangic) >= SAPMA_DEBOUNCE_SURE)

            if (sapma_sureli
                    and now - self.son_hesaplama_zamani > 5.0):
                sapma_suresi = now - self._sapma_baslangic
                print(f"{SARI}>>> [DİKKAT] Burun rotadan {min_dist:.1f}m uzak "
                      f"({sapma_suresi:.1f}s süregeldi)! Güncelleniyor...{SIFIRLA}")
                if self.logger is not None:
                    self.logger.log_event("sapma", min_dist=round(min_dist, 2),
                                          esik=SAPMA_ESIK_METRE,
                                          burun=[round(burun_x, 2), round(burun_y, 2)],
                                          robot=[round(self.robot_x, 2), round(self.robot_y, 2)],
                                          yaw_deg=round(math.degrees(self.robot_yaw), 2)
                                          if self.robot_yaw is not None else None,
                                          wp_idx=self.current_wp_index,
                                          task_idx=self.current_task_index)
                self.son_hesaplama_zamani = now
                self._sapma_baslangic = None   # FAZ2: reroute sonrası debounce sıfırla
                self.recalculate_path_from_robot(reason="sapma")

        # ── Konum izi (kısılmış; pose.csv) ──────────────────────────
        # pose_due(): throttle hint — mesafe hesapları sadece yazılacaksa
        # yapılır (kısılan tick'lerde boşa hesap yok). d_wp/d_goal o anki
        # current_wp_index/task ile tutarlı kalsın diye burada hesaplanır.
        if self.logger is not None and self.logger.pose_due():
            d_wp = d_goal = None
            try:
                nx_idx = min(self.current_wp_index + 1, len(self.full_path_world) - 1)
                wxn, wyn = self.full_path_world[nx_idx]
                d_wp = math.hypot(self.robot_x - wxn, self.robot_y - wyn)
                if self.current_task_index < len(self.geo_targets_world):
                    gxn, gyn = self.geo_targets_world[self.current_task_index]
                    d_goal = math.hypot(self.robot_x - gxn, self.robot_y - gyn)
            except Exception:  # noqa: BLE001
                pass
            self.logger.log_pose(
                self.robot_x, self.robot_y, self.robot_yaw,
                self.current_task_index, self.current_wp_index,
                len(self.full_path_world), d_wp, d_goal,
            )

        self.new_data_available = True

    def varildi_callback(self, msg: String) -> None:
        """
        /gorev_durumu 'varildi' gelince WP'yi ilerlet.
        FIX: mesaj içeriği kontrol ediliyor + mutex ile konum_callback çakışması önleniyor.
        """
        if msg.data.strip().lower() != 'varildi':
            return

        now = time.time()
        if now - self._son_varildi_zamani < 0.5:
            return

        if not self.is_path_calculated or not self.full_path_world:
            return

        with self._wp_lock:
            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_world) - 1)
            if wp1_idx < len(self.full_path_world) - 1:
                self.current_wp_index = wp1_idx
                self._son_varildi_zamani = now
                rospy.loginfo(f"[varildi] WP → {self.current_wp_index}")

    # ==========================================
    #   ROTA HESAPLAMA
    # ==========================================
    def recalculate_path_from_robot(self, reason: str = "?") -> None:
        if not self.geo_targets_world or not self.planner.nodes:
            rospy.logwarn("[recalculate] geo_targets veya planner.nodes boş!")
            return

        if self.current_task_index >= len(self.geo_targets_world):
            print(f"{YESIL}>>> TÜM GÖREVLER BİTTİ!{SIFIRLA}")
            self.full_path_world = []
            return

        if self.robot_x is None or self.robot_y is None:
            rospy.logwarn("[recalculate] Robot konumu henüz yok!")
            return

        rx, ry = self.robot_x, self.robot_y

        # ── Yaw forward-projection start seçimi (Samed'in eski sürümünden) ──
        # Aracın ÖNÜNDE (yaw yönünde) ILERI_MESAFE_M ileride sanal bir nokta
        # hesapla, start düğümünü O noktaya en yakın düğüm yap. Böylece rota
        # aracın BAKTIĞI yöne göre başlar (yaw'a göre döner) — salt mesafe değil.
        if self.robot_yaw is not None:
            front_x = rx + ILERI_MESAFE_M * math.cos(self.robot_yaw)
            front_y = ry + ILERI_MESAFE_M * math.sin(self.robot_yaw)
        else:
            front_x, front_y = rx, ry

        start_node = min(
            self.planner.nodes,
            key=lambda n: (n[0] - front_x) ** 2 + (n[1] - front_y) ** 2
        )

        goal_node = self.geo_targets_world[self.current_task_index]
        rospy.loginfo(f"[recalculate] {start_node} → {goal_node}")

        # ── İleri yönlü filtre ──────────────────────────────────────
        # FIX: her iki tarafı da güvenli sil + finally ile geri yükle
        removed_fwd  = []   # start_node → n silinen kenarlar
        removed_back = []   # n → start_node silinen kenarlar

        try:
            if self.robot_yaw is not None:
                neighbors = list(self.planner.adj_list.get(start_node, []))
                candidates = []
                for n in neighbors:
                    # FIX: yön filtresi komşunun start_node'a göre yönüne bakmalı
                    # (robot konumuna göre değil); aksi halde start robottan uzakken
                    # geri-yön kenarları yanlış değerlendirilir.
                    dx, dy = n[0] - start_node[0], n[1] - start_node[1]
                    if dx == 0 and dy == 0:
                        continue
                    diff = (math.atan2(dy, dx) - self.robot_yaw + math.pi) \
                           % (2 * math.pi) - math.pi
                    if abs(diff) > YON_FILTRE_ACIISI:
                        candidates.append(n)

                # FIX: tüm komşular kaldırılacaksa filtreyi uygulama
                if len(candidates) < len(neighbors):
                    for n in candidates:
                        if self.planner.remove_edge_directed(start_node, n):
                            removed_fwd.append(n)
                        if self.planner.remove_edge_directed(n, start_node):
                            removed_back.append(n)

            path = self.planner.find_path(start_node, goal_node)

        finally:
            # FIX: exception olsa bile kenarları geri yükle
            for n in removed_fwd:
                self.planner.restore_edge_directed(start_node, n)
            for n in removed_back:
                self.planner.restore_edge_directed(n, start_node)

        # ── Tanı logu: rota uzaktan mı çiziliyor? ───────────────────
        if self.logger is not None:
            try:
                task_name = (GOREV_GEOJSON['features'][self.current_task_index]
                             ['properties']['name'])
            except Exception:  # noqa: BLE001
                task_name = None
            # log buradan önce full_path_world hâlâ ESKİ rota → kıyas geçerli.
            # path_changed=False → rota değişmedi (boşa recalc / oscillation işareti)
            path_changed = (path != self.full_path_world) if path else None
            self.logger.log_recalc(
                reason=reason, rx=rx, ry=ry, yaw=self.robot_yaw,
                front=(front_x, front_y), start_node=start_node,
                goal_node=goal_node, task_idx=self.current_task_index,
                task_name=task_name, path=path, path_changed=path_changed,
            )

        if path:
            with self._wp_lock:
                self.full_path_world   = path
                self.current_wp_index = 0
                self.is_path_calculated = True
            print(f"{YESIL}>>> [ROTA] {len(path)} WP oluşturuldu.{SIFIRLA}")
        else:
            print(f"{KIRMIZI}!!! [HATA] Rota bulunamadı! "
                  f"{start_node} → {goal_node}{SIFIRLA}")

    # ==========================================
    #   HEDEF YAYINI
    # ==========================================
    def publish_current_waypoint(self) -> None:
        if not self.is_path_calculated or not self.full_path_world:
            return

        with self._wp_lock:
            # Get up to 5 waypoints ahead, pad with the goal node if we are near the end
            points = []
            for idx in range(1, 6): # wp1, wp2, wp3, wp4, wp5
                wp_idx = min(self.current_wp_index + idx, len(self.full_path_world) - 1)
                points.append(self.full_path_world[wp_idx])

        msg_parts = []
        for p in points:
            wx, wy = p[0], p[1]
            ntype = self.planner.node_types.get(p, 'intermediate')
            msg_parts.append(f"{wx:.2f},{wy:.2f},{ntype}")

        self.pub_hedef.publish("|".join(msg_parts))

    # ==========================================
    #   ÇİZİM
    # ==========================================
    def draw(self) -> None:
        if not ENABLE_GUI:
            return
        wp1_idx  = 0
        kalan_wp = 0

        # ── Renk paleti ─────────────────────────────────────────────
        BG        = '#2e2d2a'
        PANEL_BG  = '#272622'
        EDGE_COL  = '#3d3c38'
        NODE_COL  = '#4a4945'
        ROTA_MAIN = '#ff4d5e'
        ROTA_GLOW = '#e63946'
        ROTA_SHIN = '#ff9aa2'
        ROTA_PAST = '#e63946'
        DURAK_COL = '#00d9ff'
        WP1_COL   = '#f4d03f'
        WP2_COL   = '#e040fb'
        ARABA_COL = '#39ff14'
        TEXT_COL  = '#7a7a6e'

        if not self._static_drawn:
            self.fig.patch.set_facecolor(BG)
            self.ax.set_facecolor(PANEL_BG)
            self.ax.set_xlim([-15.0, 45.0])
            self.ax.set_ylim([-45.0, 25.0])

            # ── Graph kenarları ──────────────────────────────────────────
            if hasattr(self, 'G') and self.G is not None:
                for u, v, edge_data in self.G.edges(data=True):
                    p1 = self.G.nodes[u]['pos']
                    p2 = self.G.nodes[v]['pos']
                    etype = edge_data.get('type', 'lane')
                    if etype == 'slalom':
                        ecol = '#f28e2b'  # Orange for slalom
                        ew = 0.5
                        ealpha = 0.40
                    elif etype == 'connection':
                        ecol = '#4e79a7'  # Blue for connection
                        ew = 0.55
                        ealpha = 0.50
                    else:
                        ecol = EDGE_COL
                        ew = 0.65
                        ealpha = 0.85
                    self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                                 color=ecol, alpha=ealpha, linewidth=ew, zorder=1)
            else:
                for node, neighbors in self.planner.adj_list.items():
                    for n in neighbors:
                        self.ax.plot([node[0], n[0]], [node[1], n[1]],
                                     color=EDGE_COL, alpha=0.85, linewidth=0.65, zorder=1)

            # ── Graph düğümleri ──────────────────────────────────────────
            if self.planner.nodes:
                try:
                    nx_arr, ny_arr = zip(*self.planner.nodes)
                    self.ax.scatter(nx_arr, ny_arr,
                                    c=NODE_COL, s=5, alpha=0.65, zorder=2, linewidths=0)
                except ValueError:
                    pass

            # ── Ana duraklar ─────────────────────────────────────────────
            if self.geo_targets_world:
                try:
                    tx, ty = zip(*self.geo_targets_world)
                    self.ax.scatter(tx, ty, c=DURAK_COL, s=340,
                                    edgecolors='none', alpha=0.08, zorder=3)
                    self.ax.scatter(tx, ty, c='none', s=130,
                                    edgecolors=DURAK_COL, linewidths=1.4,
                                    alpha=0.55, zorder=4)
                    self.ax.scatter(tx, ty, c=DURAK_COL, s=30,
                                    edgecolors='none', zorder=5, label='Ana Duraklar')
                except ValueError:
                    pass

            # Initialize dynamic lines
            self.line_rota_past, = self.ax.plot([], [], color=ROTA_PAST, linewidth=1.2, alpha=0.20, zorder=6, solid_capstyle='round')
            self.line_rota_glow, = self.ax.plot([], [], color=ROTA_GLOW, linewidth=11, alpha=0.10, zorder=7, solid_capstyle='round', solid_joinstyle='round')
            self.line_rota_main, = self.ax.plot([], [], color=ROTA_MAIN, linewidth=2.8, alpha=1.0, zorder=8, solid_capstyle='round', solid_joinstyle='round', label='Rota')
            self.line_rota_shin, = self.ax.plot([], [], color=ROTA_SHIN, linewidth=0.7, alpha=0.45, zorder=9, solid_capstyle='round', solid_joinstyle='round')

            # Initialize dynamic scatters
            self.scatter_wp1 = self.ax.scatter([], [], c=WP1_COL, s=320, marker='*', edgecolors='none', zorder=11, label='WP1')
            self.scatter_wp2 = self.ax.scatter([], [], c=WP2_COL, s=180, marker='*', edgecolors='none', zorder=10, label='WP2')

            self.scatter_car_glow = self.ax.scatter([], [], c=ARABA_COL, s=420, edgecolors='none', alpha=0.10, zorder=12)
            self.scatter_car = self.ax.scatter([], [], c=ARABA_COL, s=200, marker='s', edgecolors=PANEL_BG, linewidths=1.5, zorder=13, label='Araç')

            # Legend
            leg = self.ax.legend(
                loc='upper right',
                prop={'size': 8, 'family': 'monospace'},
                framealpha=0.45,
                facecolor='#1e1d1b',
                edgecolor='#3d3c38'
            )
            for text in leg.get_texts():
                text.set_color(TEXT_COL)

            self.ax.axis('off')
            for spine in self.ax.spines.values():
                spine.set_visible(True)
                spine.set_edgecolor('#3d3c38')
                spine.set_linewidth(0.6)

            self._static_drawn = True

        # ── Rota ────────────────────────────────────────────────────
        with self._wp_lock:
            path_exists = bool(self.full_path_world)
            if self.is_path_calculated and path_exists:
                kalan_wp = len(self.full_path_world) - self.current_wp_index

                if self.current_wp_index > 0:
                    wx_past = [p[0] for p in self.full_path_world[:self.current_wp_index + 1]]
                    wy_past = [p[1] for p in self.full_path_world[:self.current_wp_index + 1]]
                    self.line_rota_past.set_data(wx_past, wy_past)
                else:
                    self.line_rota_past.set_data([], [])

                wx_ahead = [p[0] for p in self.full_path_world[self.current_wp_index:]]
                wy_ahead = [p[1] for p in self.full_path_world[self.current_wp_index:]]

                self.line_rota_glow.set_data(wx_ahead, wy_ahead)
                self.line_rota_main.set_data(wx_ahead, wy_ahead)
                self.line_rota_shin.set_data(wx_ahead, wy_ahead)

                wp1_idx = min(self.current_wp_index + 1, len(self.full_path_world) - 1)
                t1 = self.full_path_world[wp1_idx]
                self.scatter_wp1.set_offsets(np.array([[t1[0], t1[1]]]))
                self.scatter_wp1.set_visible(True)

                wp2_idx = min(wp1_idx + 1, len(self.full_path_world) - 1)
                if wp2_idx > wp1_idx:
                    t2 = self.full_path_world[wp2_idx]
                    self.scatter_wp2.set_offsets(np.array([[t2[0], t2[1]]]))
                    self.scatter_wp2.set_visible(True)
                else:
                    self.scatter_wp2.set_visible(False)
            else:
                self.line_rota_past.set_data([], [])
                self.line_rota_glow.set_data([], [])
                self.line_rota_main.set_data([], [])
                self.line_rota_shin.set_data([], [])
                self.scatter_wp1.set_visible(False)
                self.scatter_wp2.set_visible(False)

        # ── Araç ────────────────────────────────────────────────────
        if self.robot_x is not None and self.robot_y is not None:
            rx, ry = self.robot_x, self.robot_y
            self.scatter_car_glow.set_offsets(np.array([[rx, ry]]))
            self.scatter_car_glow.set_visible(True)
            self.scatter_car.set_offsets(np.array([[rx, ry]]))
            self.scatter_car.set_visible(True)
            if self.arrow_car is not None:
                self.arrow_car.remove()
                self.arrow_car = None
            if self.robot_yaw is not None:
                dx = 6.0 * math.cos(self.robot_yaw)
                dy = 6.0 * math.sin(self.robot_yaw)
                self.arrow_car = self.ax.annotate(
                    '', xy=(rx + dx, ry + dy), xytext=(rx, ry),
                    arrowprops=dict(arrowstyle='->', color=ROTA_MAIN,
                                   lw=2.0, mutation_scale=14)
                )
        else:
            self.scatter_car_glow.set_visible(False)
            self.scatter_car.set_visible(False)
            if self.arrow_car is not None:
                self.arrow_car.remove()
                self.arrow_car = None

        # ── Başlık ───────────────────────────────────────────────────
        self.ax.set_title(
            f"  DÜĞÜM {len(self.planner.nodes)}   "
            f"WP {wp1_idx} / {len(self.full_path_world)}   "
            f"KALAN {kalan_wp}   "
            f"GÖREV {self.current_task_index} / {len(self.geo_targets_world)}  ",
            fontsize=8, color=TEXT_COL, fontfamily='monospace',
            loc='left', pad=7,
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#1e1d1b',
                      edgecolor='#3d3c38', alpha=0.6)
        )

        self.fig.canvas.draw_idle()

    # ==========================================
    #   ANA DÖNGÜ
    # ==========================================
    def loop(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_path_calculated:
                self.publish_current_waypoint()
            if ENABLE_GUI and self.new_data_available:
                self.draw()
                self.new_data_available = False
            if ENABLE_GUI:
                try:
                    self.fig.canvas.flush_events()
                    plt.pause(0.001)
                except Exception:
                    pass
            rate.sleep()


if __name__ == '__main__':
    try:
        HedefYoneticisi().loop()
    except rospy.ROSInterruptException:
        pass