#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose2D
import math
import time
import threading
import json
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import heapq

# Matplotlib busy-loop'unda print()'lerin stdout buffer'da kalmamasi icin
# satir buffering'e gec; docker logs hedef_teslimi anlik mesajlari gostersin.
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except AttributeError:
    pass

YESIL  = "\033[92m"
KIRMIZI = "\033[91m"
SARI   = "\033[93m"
SIFIRLA = "\033[0m"

# Tek sabit dosya yolu. Tur degisimi: missions/gorev.geojson dosyasinin
# icerigini ilgili tur dosyasiyla degistir (cp veya ln -sf), konteyneri restart et.
GOREV_DOSYASI = '/missions/gorev.geojson'


def _yukle_gorev_geojson(yol=GOREV_DOSYASI):
    """GeoJSON dosyasindan gorev noktalarini yukle.

    Dosyada datum ve start ozel feature'lar; bunlar gorev listesinden cikarilir.
    Geriye kalan features (gorev_*, park_giris, ...) sirayla isleme alinir.
    Koordinat olarak properties.local_x ve properties.local_y kullanilir
    (datum-relative ENU metre); geometry.coordinates (GPS lon/lat) yok sayilir.
    """
    with open(yol) as f:
        data = json.load(f)

    gorevler = []
    for feat in data.get('features', []):
        props = feat.get('properties', {})
        name = props.get('name', '')
        if name in ('datum', 'start'):
            continue
        if 'local_x' not in props or 'local_y' not in props:
            raise ValueError(f"[gorev] {name} icin local_x/local_y eksik")
        gorevler.append({
            'name': name,
            'description': props.get('description', ''),
            'local_x': float(props['local_x']),
            'local_y': float(props['local_y']),
            'heading': float(props.get('heading', 0.0)),
        })
    if not gorevler:
        raise ValueError(f"[gorev] {yol} icinde gorev noktasi yok")
    return gorevler

# Sapma kontrolü için eşik değerleri
# FIX 3: 6.0 → 10.0 — eski değer Pure Pursuit lookahead (~5m) ile çok yakındı,
# false-positive recalculate üretiyordu. Literatür 3-4× lookahead öneriyor;
# 10m muhafazakar orta yol (15m reviewer'a göre çok geniş kaldı).
SAPMA_ESIK_METRE   = 10.0
GOREV_YAKINLIK_M   = 5.0   # bu mesafede görevi tamamlandı sayar
WP_GECIS_MESAFE_M  = 2.5   # bu mesafede WP geçildi sayılır
YON_FILTRE_ACIISI  = math.radians(70.0)    # geri yön filtresi açısı (70° → 86°+ dönüşler "geri" sayılır, D* alternatif rotayı tercih eder)


# ── Cross-track + project-to-path yardımcıları (FIX 3) ──────────────────
def _cross_track_distance(px, py, ax, ay, bx, by):
    """Robot (px,py)'nin (ax,ay)→(bx,by) segmentine dik (Euclidean) mesafesi.
    Segment dışı projeksiyonsa en yakın endpoint mesafesi döner."""
    seg_dx = bx - ax
    seg_dy = by - ay
    seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
    if seg_len_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    # Proj t ∈ [0,1] segment üzerinde
    t = ((px - ax) * seg_dx + (py - ay) * seg_dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = ax + t * seg_dx
    proj_y = ay + t * seg_dy
    return math.hypot(px - proj_x, py - proj_y)


def _project_robot_to_path_idx(robot_xy, path_xy):
    """Robot'a en yakın PATH ardışık segmentini bul, START indeksini döndür.
    path_xy: [(x,y), ...] world coord. Döner: 0 <= idx < len(path)-1.
    Eğer path uzunluğu < 2 ise 0 döner."""
    if len(path_xy) < 2:
        return 0
    px, py = robot_xy
    best_idx = 0
    best_d = float('inf')
    for i in range(len(path_xy) - 1):
        ax, ay = path_xy[i]
        bx, by = path_xy[i + 1]
        d = _cross_track_distance(px, py, ax, ay, bx, by)
        if d < best_d:
            best_d = d
            best_idx = i
    return best_idx


# ==========================================
#   D* LITE PLANNER
# ==========================================
class DLitePlanner:
    def __init__(self):
        self.adj_list: dict[tuple, list] = {}
        self.nodes: set[tuple] = set()
        self.g:   dict[tuple, float] = {}
        self.rhs: dict[tuple, float] = {}
        self.U:   list = []
        self.km:  float = 0.0
        self.s_start = None
        self.s_goal  = None

    # ── Graph yönetimi ──────────────────────────────────────────────
    def add_edge(self, p1: tuple, p2: tuple) -> None:
        for a, b in ((p1, p2), (p2, p1)):
            self.adj_list.setdefault(a, [])
            if b not in self.adj_list[a]:
                self.adj_list[a].append(b)
        self.nodes.add(p1)
        self.nodes.add(p2)

    def remove_edge_directed(self, src: tuple, dst: tuple) -> bool:
        """Tek yönlü kenar siler; kenar yoksa False döner."""
        try:
            self.adj_list[src].remove(dst)
            return True
        except (KeyError, ValueError):
            return False

    def restore_edge_directed(self, src: tuple, dst: tuple) -> None:
        """Tek yönlü kenarı geri ekler (varsa tekrar eklemez)."""
        self.adj_list.setdefault(src, [])
        if dst not in self.adj_list[src]:
            self.adj_list[src].append(dst)

    # ── Yardımcılar ─────────────────────────────────────────────────
    @staticmethod
    def dist(p1: tuple, p2: tuple) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def calculate_key(self, s: tuple) -> tuple:
        g_val  = self.g.get(s,   float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        min_val = min(g_val, rhs_val)
        return (min_val + self.dist(self.s_start, s) + self.km, min_val)

    # ── Çekirdek D* Lite ────────────────────────────────────────────
    def update_vertex(self, u: tuple) -> None:
        if u != self.s_goal:
            min_rhs = float('inf')
            for nb in self.adj_list.get(u, []):
                val = self.dist(u, nb) + self.g.get(nb, float('inf'))
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
                for s in self.adj_list.get(u, []):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.adj_list.get(u, []):
                    self.update_vertex(s)

    def find_path(self, start: tuple, goal: tuple):
        if start not in self.adj_list or goal not in self.adj_list:
            rospy.logwarn(f"[D*Lite] start veya goal graph'ta yok! "
                          f"start:{start} goal:{goal}")
            return None

        self.s_start = start
        self.s_goal  = goal
        self.km = 0.0
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
                key=lambda n: self.dist(curr, n) + self.g.get(n, float('inf'))
            )

            if best_next in seen:
                rospy.logwarn("[D*Lite] Döngü tespit edildi — rota geçersiz.")
                return None             # yarım rota yerine None dön

            seen.add(best_next)
            path.append(best_next)
            curr = best_next

        return path if len(path) > 1 and curr == self.s_goal else None


# ==========================================
#   KOORDİNAT DÖNÜŞÜM YARDIMCILARI
# ==========================================
def world_to_grid(wx: float, wy: float, map_info) -> tuple:
    if map_info is None:
        return (None, None)
    res = map_info.resolution
    ox  = map_info.origin.position.x
    oy  = map_info.origin.position.y
    return (int((wx - ox) / res), int((wy - oy) / res))


def grid_to_world(gx: int, gy: int, map_info) -> tuple:
    if map_info is None:
        return (0.0, 0.0)
    res = map_info.resolution
    ox  = map_info.origin.position.x
    oy  = map_info.origin.position.y
    return (gx * res + ox + res / 2.0, gy * res + oy + res / 2.0)


# ==========================================
#   YÖNETİCİ SINIFI
# ==========================================
class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')

        # ── Gorev dosyasini yukle ────────────────────────────────────
        try:
            self.gorev_listesi = _yukle_gorev_geojson()
        except Exception as e:
            rospy.logfatal(f"[gorev] {GOREV_DOSYASI} yuklenemedi: {e}")
            sys.exit(1)
        isimler = [g['name'] for g in self.gorev_listesi]
        print(f"{YESIL}>>> [GOREV] {len(self.gorev_listesi)} nokta yuklendi: "
              f"{', '.join(isimler)}{SIFIRLA}")

        # ── Harita & görselleştirme ──────────────────────────────────
        self.map_info        = None
        self.viz_data        = {}
        self.new_data_available = False

        # ── Robot durumu ─────────────────────────────────────────────
        self.robot_x         = None   # None: henüz konum gelmedi
        self.robot_y         = None
        self.robot_yaw       = None
        self.robot_grid_pos  = None
        self._ilk_konum_alindi = False  # FIX: ilk konuma kadar görev kontrolü yapma

        # ── Rota durumu ──────────────────────────────────────────────
        self.full_path_grid      = []
        self.current_wp_index    = 0
        self.current_task_index  = 0
        self.is_path_calculated  = False
        self.geo_targets_grid    = []
        self.geo_targets_built   = False  # FIX: tek seferlik build kontrolü

        # ── Zamanlayıcılar ───────────────────────────────────────────
        # FIX: 0.0 yerine time.time() → node başlar başlamaz cooldown aktif
        self.son_hesaplama_zamani = time.time()
        self._son_varildi_zamani  = time.time()
        self._son_gorev_zamani    = time.time()

        # FIX 1 (rate limiter): aynı (start_node, goal_node, graph_hash)
        # çiftine arka arkaya plan çağrılarını cooldown süresince engelle.
        # graph_hash key'e eklenir → marker_callback graph'ı yenilerse
        # (yeni edge gelirse) cooldown bypass olur, planlayıcı yeni edge'i
        # kullanabilir. Bu D* Lite'ın "thrashing" patolojisini söndürür.
        self._last_plan_key = None
        self._last_plan_t   = 0.0
        # Cooldown rosparam — production'da ROS launch'tan ayarlanabilir.
        # graph_hash = sum(len(v)) → marker_callback'in pop/restore döngüsü
        # FINALLY ile tamamlanır ve hash aynı kalır; rate-limit aynı çağrı
        # için tetiklenir. Sadece add_edge yapılıyor (silme yok) → toplam
        # entry sayısı monoton artar; bir kenar silinip yerine yeni eklense
        # hash sabit kalabilirdi ama mevcut akış için yeterli.
        try:
            self.REPLAN_COOLDOWN_S = rospy.get_param("~replan_cooldown", 3.0)
        except Exception:
            self.REPLAN_COOLDOWN_S = 3.0
        self._rate_limit_blocked_count = 0  # metric

        # FIX 5 (stuck → WP skip): 5 sn'de < STUCK_MOVE_M hareket ettiyse
        # ve WP1'den > STUCK_WP_DIST_M uzaktaysa current_wp_index++. **Görev
        # iptal YOK** — sadece path içinde sıkıştığımız WP'yi atlıyoruz.
        # Karar guard: yalnız karar='normal' iken atlama (DUR/SLOW/ACIL_DURUS
        # meşru beklemeler — yarış kuralları için kritik).
        import collections as _collections
        self._konum_history = _collections.deque(maxlen=100)  # (t, x, y)
        # FIX 5 reviewer-B: 0.0 ile başlamak → time.time()-0 = ~1.7e9 >> 10 →
        # cooldown ilk skip'i engellemiyor → sistem açıldıktan 2sn sonra
        # hemen WP atlayabilir. Constructor'da time.time() ile başlat ki
        # ilk 10 sn cooldown ile korunsun.
        self._last_stuck_skip = time.time()
        self._current_karar = "normal"   # /karar topic'inden güncellenir
        self.STUCK_WINDOW_S = 5.0
        self.STUCK_MOVE_M   = 0.5
        self.STUCK_WP_DIST_M = 2.0
        self.STUCK_SKIP_COOLDOWN_S = 10.0
        self._stuck_skip_count = 0  # metric

        # ── Thread güvenliği ─────────────────────────────────────────
        # FIX: varildi_callback & konum_callback çakışmasını önler
        self._wp_lock = threading.Lock()

        # ── Marker buffer ────────────────────────────────────────────
        self.pending_markers = []

        # ── Planner ─────────────────────────────────────────────────
        self.planner = DLitePlanner()

        # ── Görselleştirme ───────────────────────────────────────────
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')

        # ── ROS bağlantıları ─────────────────────────────────────────
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10)

        rospy.Subscriber('/map',           OccupancyGrid, self.map_callback)
        rospy.Subscriber('/waypoint',      MarkerArray,   self.marker_callback)
        rospy.Subscriber('/konum',         Pose2D,        self.konum_callback)
        rospy.Subscriber('/gorev_durumu',  String,        self.varildi_callback)
        rospy.Subscriber('/karar',         String,        self.karar_callback)

        print(f"{YESIL}>>> SİSTEM HAZIR. Bekleniyor: /map, /waypoint, /konum{SIFIRLA}")

    # ==========================================
    #   CALLBACKLER
    # ==========================================
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        self.viz_data['map'] = np.array(msg.data, dtype=np.int8).reshape((h, w))

        if self.pending_markers:
            rospy.loginfo(f"[map] {len(self.pending_markers)} bekleyen marker işleniyor...")
            for m in self.pending_markers:
                self._process_markers(m)
            self.pending_markers.clear()

        self.new_data_available = True

    def marker_callback(self, msg: MarkerArray) -> None:
        if self.map_info is None:
            rospy.logwarn_throttle(5.0, "[marker] /map henüz gelmedi, buffer'a alındı.")
            self.pending_markers.append(msg)
            return
        self._process_markers(msg)

    def _process_markers(self, msg: MarkerArray) -> None:
        # FIX (phantom edges): waypoint_pub iki marker yayınlar — SPHERE_LIST
        # (type=7, düğüm noktaları) ve LINE_LIST (type=5, edge segmentleri).
        # Eski kod tipi kontrol etmeden HER marker'ın noktalarını ardışık
        # çiftler halinde edge olarak ekliyordu → SPHERE_LIST'teki (0,1),
        # (2,3), ... çiftleri fake edge oluyordu. CUSTOM_EDGES'da olmayan
        # pairs (örn. (26,27)) "phantom edge" olarak graph'a girip D*'in
        # imkânsız rotaları (27→26) bulmasına yol açıyordu.
        # Düzeltme: yalnız LINE_LIST işle + her çağrıda sıfırdan inşa et.
        LINE_LIST = 5
        self.planner.adj_list = {}
        self.planner.nodes = set()

        for m in msg.markers:
            if m.type != LINE_LIST:
                continue
            pts = m.points
            for i in range(0, len(pts) - 1, 2):
                p1 = world_to_grid(pts[i].x,   pts[i].y,   self.map_info)
                p2 = world_to_grid(pts[i+1].x, pts[i+1].y, self.map_info)
                if None in p1 or None in p2 or p1 == p2:
                    continue
                self.planner.add_edge(p1, p2)

        rospy.loginfo_throttle(
            10.0,
            f"[graph] {len(self.planner.nodes)} düğüm, "
            f"{sum(len(v) for v in self.planner.adj_list.values()) // 2} kenar"
        )

        # FIX: geo_targets sadece bir kez inşa edilir;
        #      graph sonradan büyüse de snap zaten ilk tam graph üzerinden yapıldı.
        #      Eğer graph çok küçükken inşa edildiyse yeniden dene.
        if not self.geo_targets_built and len(self.planner.nodes) >= 10:
            self._build_geo_targets()

        # Graf her marker çağrısında sıfırdan inşa ediliyor; data hep "yeni".
        self.new_data_available = True

    def _build_geo_targets(self) -> None:
        """Her görev koordinatını graph'taki en yakın düğüme snap'ler."""
        self.geo_targets_grid.clear()
        for gorev in self.gorev_listesi:
            tg = world_to_grid(gorev['local_x'], gorev['local_y'], self.map_info)
            nearest = min(
                self.planner.nodes,
                key=lambda n: (n[0] - tg[0])**2 + (n[1] - tg[1])**2
            )
            snap = math.hypot(nearest[0] - tg[0], nearest[1] - tg[1])
            print(f"{YESIL}>>> [{gorev['name']}] "
                  f"snap:{snap:.1f} hücre → {nearest}{SIFIRLA}")
            self.geo_targets_grid.append(nearest)

        self.geo_targets_built = True

        # İlk konum zaten geldiyse hemen rota hesapla
        if self._ilk_konum_alindi and self.robot_grid_pos and self.robot_grid_pos[0] is not None:
            self.recalculate_path_from_robot()

    def konum_callback(self, msg: Pose2D) -> None:
        self.robot_x   = msg.x
        self.robot_y   = msg.y
        self.robot_yaw = msg.theta
        self.robot_grid_pos = world_to_grid(self.robot_x, self.robot_y, self.map_info)

        if self.robot_grid_pos is None or self.robot_grid_pos[0] is None:
            return

        # FIX 5: konum geçmişini güncelle + stuck check
        history = getattr(self, "_konum_history", None)
        if history is not None:
            now_t = time.time()
            history.append((now_t, self.robot_x, self.robot_y))
            # 5 sn'den eski kayıtları temizle (deque uzunluğu ile zaten sınırlı)
            cutoff = now_t - getattr(self, "STUCK_WINDOW_S", 5.0)
            while history and history[0][0] < cutoff:
                history.popleft()
            # Stuck check (cooldown ve karar guard'lı, _check_stuck içinde)
            self._check_stuck_and_skip_wp(now_t)

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
                and self.geo_targets_grid):
            self.recalculate_path_from_robot()

        if not self.is_path_calculated or not self.full_path_grid:
            self.new_data_available = True
            return

        now = time.time()

        # ── Otomatik WP geçişi (mesafe bazlı) ───────────────────────
        with self._wp_lock:
            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_grid) - 1)
            gx_wp, gy_wp = self.full_path_grid[wp1_idx]
            wx_wp, wy_wp = grid_to_world(gx_wp, gy_wp, self.map_info)
            dist_to_wp   = math.hypot(self.robot_x - wx_wp, self.robot_y - wy_wp)

            if dist_to_wp < WP_GECIS_MESAFE_M and wp1_idx < len(self.full_path_grid) - 1:
                self.current_wp_index = wp1_idx
                rospy.loginfo(f"[OTO] WP {self.current_wp_index} geçildi "
                              f"(d:{dist_to_wp:.1f}m)")

        # ── Ana hedef (durak) kontrolü ───────────────────────────────
        if self.current_task_index < len(self.geo_targets_grid):
            gx_g, gy_g = self.geo_targets_grid[self.current_task_index]
            wx_g, wy_g = grid_to_world(gx_g, gy_g, self.map_info)
            dist_to_goal = math.hypot(self.robot_x - wx_g, self.robot_y - wy_g)

            if dist_to_goal < GOREV_YAKINLIK_M and now - self._son_gorev_zamani > 5.0:
                self._son_gorev_zamani = now
                self.current_task_index += 1

                if self.current_task_index >= len(self.geo_targets_grid):
                    print(f"{YESIL}>>> TÜM GÖREVLER TAMAMLANDI!{SIFIRLA}")
                    self.is_path_calculated = False
                    self.full_path_grid     = []
                    self.new_data_available = True
                    return

                next_name = self.gorev_listesi[self.current_task_index]['name']
                print(f"{YESIL}>>> DURAK TAMAMLANDI! Yeni hedef: {next_name}{SIFIRLA}")
                self.recalculate_path_from_robot()

        # ── Sapma kontrolü ───────────────────────────────────────────
        # FIX 3: Min nokta mesafesi yerine cross-track distance (en yakın
        # path SEGMENT'ine dik). Pencere `[wp-2 : wp+10]` — geriye 2 segment
        # bak (geçilmiş WP yakınlığı), ileri 10 segment.
        # SAPMA_ESIK 6→10: lookahead ile çakışan false-positive azalır.
        start_idx = max(0, self.current_wp_index - 2)
        end_idx   = min(len(self.full_path_grid), self.current_wp_index + 11)
        window = self.full_path_grid[start_idx:end_idx]
        if len(window) >= 2:
            world_pts = [grid_to_world(g[0], g[1], self.map_info) for g in window]
            min_ct = min(
                _cross_track_distance(self.robot_x, self.robot_y,
                                      world_pts[i][0], world_pts[i][1],
                                      world_pts[i+1][0], world_pts[i+1][1])
                for i in range(len(world_pts) - 1)
            )
            if (min_ct > SAPMA_ESIK_METRE
                    and now - self.son_hesaplama_zamani > 5.0):
                print(f"{SARI}>>> [DİKKAT] Cross-track {min_ct:.1f}m > "
                      f"{SAPMA_ESIK_METRE}m! Güncelleniyor...{SIFIRLA}")
                self.son_hesaplama_zamani = now
                self.recalculate_path_from_robot()

        self.new_data_available = True

    def karar_callback(self, msg) -> None:
        """FIX 5: karar-node çıktısını yakalar — stuck-skip guard için."""
        try:
            self._current_karar = (msg.data or "").strip().lower()
        except Exception:
            pass

    def _check_stuck_and_skip_wp(self, now: float) -> None:
        """FIX 5: 5 sn'de < 0.5m hareket + WP1'den > 2m + karar='normal' →
        current_wp_index++. Görev iptal YOK. Cooldown 10 sn."""
        # Karar guard — meşru beklemelerde atlama yapma
        if getattr(self, "_current_karar", "normal") != "normal":
            return
        # Cooldown
        if now - getattr(self, "_last_stuck_skip", 0.0) < self.STUCK_SKIP_COOLDOWN_S:
            return
        # Konum geçmişi yeterli mi
        history = getattr(self, "_konum_history", None)
        if history is None or len(history) < 20:
            return
        # 5 sn pencere içindeki span
        cutoff = now - self.STUCK_WINDOW_S
        recent = [(x, y) for t, x, y in history if t >= cutoff]
        if len(recent) < 10:
            return
        xs = [p[0] for p in recent]
        ys = [p[1] for p in recent]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if span >= self.STUCK_MOVE_M:
            return
        # WP1'e mesafe kontrol
        if not self.is_path_calculated or not self.full_path_grid:
            return
        with self._wp_lock:
            wp1_idx = self.current_wp_index + 1
            if wp1_idx >= len(self.full_path_grid):
                return
            wp1 = self.full_path_grid[wp1_idx]
            wx, wy = grid_to_world(wp1[0], wp1[1], self.map_info)
            dist_to_wp = math.hypot(self.robot_x - wx, self.robot_y - wy)
            if dist_to_wp <= self.STUCK_WP_DIST_M:
                return  # WP zaten yakın, atlama gereksiz
            # Atlama yap
            self.current_wp_index = wp1_idx
            self._last_stuck_skip = now
            self._stuck_skip_count = getattr(self, "_stuck_skip_count", 0) + 1
            rospy.logwarn(
                f"[STUCK-SKIP] span={span:.2f}m dist_to_wp={dist_to_wp:.1f}m "
                f"karar=normal → WP atlandı (wp_idx={self.current_wp_index}, "
                f"skip_total={self._stuck_skip_count})"
            )
            # FIX 5 reviewer-F: tek başına WP atlama "hayalî path" yaratır
            # (engel hâlâ orada). Replan zorla → yeni bir path üret, böylece
            # 12 WP'yi 2 dk'da atlayıp puansız bitirme riskini önle.
            # NOT: recalculate kendi rate-limiter'ı (FIX 1) ile korunur, ancak
            # graph_hash sabit + start_node aynı olduğundan key aynı kalır.
            # Cooldown'u sıfırla ki bir sonraki konum_callback recalculate'i tetiklesin.
            self.son_hesaplama_zamani = 0.0

    def varildi_callback(self, msg: String) -> None:
        """
        /gorev_durumu 'varildi' gelince WP'yi ilerlet.
        FIX: mesaj içeriği kontrol ediliyor + mutex ile konum_callback çakışması önleniyor.
        """
        if msg.data.strip().lower() != 'varildi':
            return

        now = time.time()
        if now - self._son_varildi_zamani < 3.0:
            return
        self._son_varildi_zamani = now

        if not self.is_path_calculated or not self.full_path_grid:
            return

        with self._wp_lock:
            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_grid) - 1)
            if wp1_idx < len(self.full_path_grid) - 1:
                self.current_wp_index = wp1_idx
                rospy.loginfo(f"[varildi] WP → {self.current_wp_index}")

    # ==========================================
    #   ROTA HESAPLAMA
    # ==========================================
    def recalculate_path_from_robot(self) -> None:
        if not self.geo_targets_grid or not self.planner.nodes:
            rospy.logwarn("[recalculate] geo_targets veya planner.nodes boş!")
            return

        if self.current_task_index >= len(self.geo_targets_grid):
            print(f"{YESIL}>>> TÜM GÖREVLER BİTTİ!{SIFIRLA}")
            self.full_path_grid = []
            return

        if self.robot_grid_pos is None or self.robot_grid_pos[0] is None:
            rospy.logwarn("[recalculate] Robot konumu henüz yok!")
            return

        rx, ry = self.robot_grid_pos
        start_node = min(
            self.planner.nodes,
            key=lambda n: (n[0] - rx)**2 + (n[1] - ry)**2
        )
        goal_node = self.geo_targets_grid[self.current_task_index]

        # FIX 1 (rate limiter): aynı (start, goal, graph_hash) için cooldown.
        # graph_hash = adj_list entry count → graf değişirse key değişir, bypass.
        # check+set bloğu _wp_lock altında — recalculate_path_from_robot
        # birden çok ROS callback thread'inden çağrılabilir (marker_callback,
        # konum_callback, varildi_callback). Lock olmadan iki thread aynı
        # anda last_key'i okuyup `None` görüp ikisi de plan yapardı.
        # getattr fallback: test scriptleri __init__'i bypass ediyor.
        graph_hash = sum(len(v) for v in self.planner.adj_list.values())
        plan_key = (start_node, goal_node, graph_hash)
        now = time.time()
        cooldown = getattr(self, "REPLAN_COOLDOWN_S", 3.0)
        with self._wp_lock:
            last_key = getattr(self, "_last_plan_key", None)
            last_t   = getattr(self, "_last_plan_t", 0.0)
            if plan_key == last_key and (now - last_t) < cooldown:
                self._rate_limit_blocked_count = getattr(
                    self, "_rate_limit_blocked_count", 0) + 1
                rospy.loginfo_throttle(
                    3.0,
                    f"[RATE-LIMIT] {start_node} → {goal_node} cooldown "
                    f"({now - last_t:.1f}s < {cooldown}s); "
                    f"blocked_total={self._rate_limit_blocked_count}"
                )
                return
            self._last_plan_key = plan_key
            self._last_plan_t   = now

        rospy.loginfo(f"[recalculate] {start_node} → {goal_node}")

        # DEBUG: ileri/geri komşu sayısı (yaw varsa)
        if self.robot_yaw is not None:
            sxd, syd = start_node
            fwd_count = sum(
                1 for n in self.planner.adj_list.get(start_node, [])
                if (n[0] != sxd or n[1] != syd) and abs(
                    ((math.atan2(n[1] - syd, n[0] - sxd)
                      - self.robot_yaw + math.pi) % (2 * math.pi) - math.pi)
                ) <= YON_FILTRE_ACIISI
            )
            total = len(self.planner.adj_list.get(start_node, []))
            rospy.loginfo(
                f"[FILTER-DEBUG] start_node={start_node} fwd={fwd_count}/{total} "
                f"robot_yaw={math.degrees(self.robot_yaw):.0f}°"
            )

        # ── İleri yönlü filtre — DÜZELTİLMİŞ ────────────────────────
        # Hata #1: tüm komşular geri yönlüyse "en az ters" olanı bırak.
        # Hata #2: açıyı robot'tan değil start_node'dan ölç.
        sx, sy = start_node
        removed_fwd  = []   # start_node → n silinen kenarlar
        removed_back = []   # n → start_node silinen kenarlar

        try:
            if self.robot_yaw is not None:
                neighbors = list(self.planner.adj_list.get(start_node, []))
                fwd, back = [], []   # back: [(neighbor, abs_diff)]
                for n in neighbors:
                    dx, dy = n[0] - sx, n[1] - sy
                    if dx == 0 and dy == 0:
                        continue
                    diff = (math.atan2(dy, dx) - self.robot_yaw + math.pi) \
                           % (2 * math.pi) - math.pi
                    if abs(diff) > YON_FILTRE_ACIISI:
                        back.append((n, abs(diff)))
                    else:
                        fwd.append(n)

                if fwd:
                    to_remove = [n for n, _ in back]
                else:
                    # Hata #1: hiç forward edge yoksa hepsini silme — en az
                    # ters olanı bırak ki D* Lite "döne döne ilerle" sinyali
                    # bulabilsin.
                    back.sort(key=lambda x: x[1])
                    to_remove = [n for n, _ in back[1:]]
                    if back:
                        rospy.logwarn(
                            f"[U-DONUS-PLAN] {start_node}: tüm komşular geri "
                            f"yönlü; en az ters komşu bırakıldı "
                            f"(kalan diff={back[0][1]:.2f} rad ≈ "
                            f"{math.degrees(back[0][1]):.0f}°)"
                        )
                    if len(back) == 1:
                        rospy.logwarn(
                            "[U-DONUS-PLAN] Tek komşu mevcut, U-dönüş "
                            "kaçınılmaz (control.py mesafe < 9 m'de devreye girer)"
                        )

                for n in to_remove:
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

        if path:
            # FIX 3 (project-to-path): wp_index'i 0 yerine robot'un en yakın
            # path segmentinin başlangıç indeksine ata. Eski "current_wp_index=0"
            # robot path ortasında olsa bile WP0'a dönüş yayını yapıyordu →
            # control.py "geriye git" anlıyordu → U-dönüş guard tetikleniyor,
            # araç stuck. Şimdi recalculate sonrası robot doğru WP'den devam.
            # getattr ile koruma: test scriptleri __init__'i bypass ediyor,
            # map_info / robot_x set olmayabilir.
            map_info = getattr(self, "map_info", None)
            r_x = getattr(self, "robot_x", None)
            r_y = getattr(self, "robot_y", None)
            if map_info is not None and r_x is not None:
                path_world = [grid_to_world(g[0], g[1], map_info) for g in path]
                proj_idx = _project_robot_to_path_idx((r_x, r_y), path_world)
            elif r_x is None:
                # Test ortamı: grid = world varsayımı (test'ler bu yolu kullanır)
                proj_idx = 0
            else:
                proj_idx = 0
            with self._wp_lock:
                self.full_path_grid   = path
                self.current_wp_index = proj_idx
                self.is_path_calculated = True
            print(f"{YESIL}>>> [ROTA] {len(path)} WP oluşturuldu "
                  f"(proj_idx={proj_idx}).{SIFIRLA}")
        else:
            print(f"{KIRMIZI}!!! [HATA] Rota bulunamadı! "
                  f"{start_node} → {goal_node}{SIFIRLA}")

    # ==========================================
    #   HEDEF YAYINI
    # ==========================================
    def publish_current_waypoint(self) -> None:
        # FIX: full_path_grid boş kontrolü eklendi
        if not self.is_path_calculated or not self.full_path_grid:
            return

        with self._wp_lock:
            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_grid) - 1)
            wp2_idx = min(wp1_idx + 1,               len(self.full_path_grid) - 1)
            p1 = self.full_path_grid[wp1_idx]
            p2 = self.full_path_grid[wp2_idx]

        wx1, wy1 = grid_to_world(p1[0], p1[1], self.map_info)
        wx2, wy2 = grid_to_world(p2[0], p2[1], self.map_info)
        self.pub_hedef.publish(f"{wx1:.2f},{wy1:.2f}|{wx2:.2f},{wy2:.2f}")

    # ==========================================
    #   ÇİZİM
    # ==========================================
    def draw(self) -> None:
        if 'map' not in self.viz_data:
            return

        self.ax.clear()
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

        self.fig.patch.set_facecolor(BG)
        self.ax.set_facecolor(PANEL_BG)

        # ── Harita ──────────────────────────────────────────────────
        self.ax.imshow(
            self.viz_data['map'], cmap='Greys_r',
            origin='lower', vmin=0, vmax=100, alpha=0.13
        )

        # ── Graph kenarları ──────────────────────────────────────────
        for node, neighbors in self.planner.adj_list.items():
            for n in neighbors:
                self.ax.plot([node[0], n[0]], [node[1], n[1]],
                             color=EDGE_COL, alpha=0.85, linewidth=0.65, zorder=1)

        # ── Graph düğümleri ──────────────────────────────────────────
        # FIX: boş set kontrolü
        if self.planner.nodes:
            try:
                nx_arr, ny_arr = zip(*self.planner.nodes)
                self.ax.scatter(nx_arr, ny_arr,
                                c=NODE_COL, s=5, alpha=0.65, zorder=2, linewidths=0)
            except ValueError:
                pass

        # ── Ana duraklar ─────────────────────────────────────────────
        if self.geo_targets_grid:
            try:
                tx, ty = zip(*self.geo_targets_grid)
                self.ax.scatter(tx, ty, c=DURAK_COL, s=340,
                                edgecolors='none', alpha=0.08, zorder=3)
                self.ax.scatter(tx, ty, c='none', s=130,
                                edgecolors=DURAK_COL, linewidths=1.4,
                                alpha=0.55, zorder=4)
                self.ax.scatter(tx, ty, c=DURAK_COL, s=30,
                                edgecolors='none', zorder=5, label='Ana Duraklar')
            except ValueError:
                pass

        # ── Rota ────────────────────────────────────────────────────
        if self.is_path_calculated and self.full_path_grid:
            kalan_wp = len(self.full_path_grid) - self.current_wp_index

            if self.current_wp_index > 0:
                gx_past = [p[0] for p in self.full_path_grid[:self.current_wp_index + 1]]
                gy_past = [p[1] for p in self.full_path_grid[:self.current_wp_index + 1]]
                self.ax.plot(gx_past, gy_past,
                             color=ROTA_PAST, linewidth=1.2, alpha=0.20,
                             zorder=6, solid_capstyle='round')

            gx_ahead = [p[0] for p in self.full_path_grid[self.current_wp_index:]]
            gy_ahead = [p[1] for p in self.full_path_grid[self.current_wp_index:]]

            self.ax.plot(gx_ahead, gy_ahead,
                         color=ROTA_GLOW, linewidth=11, alpha=0.10,
                         zorder=7, solid_capstyle='round', solid_joinstyle='round')
            self.ax.plot(gx_ahead, gy_ahead,
                         color=ROTA_MAIN, linewidth=2.8, alpha=1.0,
                         zorder=8, solid_capstyle='round', solid_joinstyle='round',
                         label='Rota')
            self.ax.plot(gx_ahead, gy_ahead,
                         color=ROTA_SHIN, linewidth=0.7, alpha=0.45,
                         zorder=9, solid_capstyle='round', solid_joinstyle='round')

            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_grid) - 1)
            t1 = self.full_path_grid[wp1_idx]
            self.ax.scatter(t1[0], t1[1],
                            c=WP1_COL, s=320, marker='*',
                            edgecolors='none', zorder=11, label='WP1')

            wp2_idx = min(wp1_idx + 1, len(self.full_path_grid) - 1)
            if wp2_idx > wp1_idx:
                t2 = self.full_path_grid[wp2_idx]
                self.ax.scatter(t2[0], t2[1],
                                c=WP2_COL, s=180, marker='*',
                                edgecolors='none', zorder=10, label='WP2')

        # ── Araç ────────────────────────────────────────────────────
        if self.robot_grid_pos and self.robot_grid_pos[0] is not None:
            rx, ry = self.robot_grid_pos
            self.ax.scatter(rx, ry, c=ARABA_COL, s=420,
                            edgecolors='none', alpha=0.10, zorder=12)
            self.ax.scatter(rx, ry, c=ARABA_COL, s=200,
                            marker='s', edgecolors=PANEL_BG,
                            linewidths=1.5, zorder=13, label='Araç')
            if self.robot_yaw is not None:
                dx = 6.0 * math.cos(self.robot_yaw)
                dy = 6.0 * math.sin(self.robot_yaw)
                self.ax.annotate(
                    '', xy=(rx + dx, ry + dy), xytext=(rx, ry),
                    arrowprops=dict(arrowstyle='->', color=ROTA_MAIN,
                                   lw=2.0, mutation_scale=14)
                )

        # ── Legend ──────────────────────────────────────────────────
        leg = self.ax.legend(
            loc='upper right',
            prop={'size': 8, 'family': 'monospace'},
            framealpha=0.45,
            facecolor='#1e1d1b',
            edgecolor='#3d3c38'
        )
        for text in leg.get_texts():
            text.set_color(TEXT_COL)

        # ── Başlık ───────────────────────────────────────────────────
        self.ax.set_title(
            f"  DÜĞÜM {len(self.planner.nodes)}   "
            f"WP {wp1_idx} / {len(self.full_path_grid)}   "
            f"KALAN {kalan_wp}   "
            f"GÖREV {self.current_task_index} / {len(self.geo_targets_grid)}  ",
            fontsize=8, color=TEXT_COL, fontfamily='monospace',
            loc='left', pad=7,
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#1e1d1b',
                      edgecolor='#3d3c38', alpha=0.6)
        )

        self.ax.axis('off')
        for spine in self.ax.spines.values():
            spine.set_visible(True)
            spine.set_edgecolor('#3d3c38')
            spine.set_linewidth(0.6)

        self.fig.canvas.draw_idle()

    # ==========================================
    #   ANA DÖNGÜ
    # ==========================================
    def loop(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_path_calculated:
                self.publish_current_waypoint()
            if self.new_data_available:
                self.draw()
                self.new_data_available = False
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