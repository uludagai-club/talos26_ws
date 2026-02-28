#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray 
from geometry_msgs.msg import Pose2D
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import numpy as np
import heapq

# ==========================================
# 1- HEDEFLER (GeoJSON - Ana Duraklar)
# ==========================================
GOREV_GEOJSON = {
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": { "name": "gorev_1", "description": "1. Durak" },
      "geometry": { "type": "Point", "coordinates": [-5.0, -34.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_2", "description": "2. Durak" },
      "geometry": { "type": "Point", "coordinates": [11.0, -25.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_3", "description": "3. Durak" },
      "geometry": { "type": "Point", "coordinates": [20.0, -22.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_4", "description": "4. Durak (FİNİSH)" },
      "geometry": { "type": "Point", "coordinates": [25.0, -6.0] }
    }
  ]
}

# ==========================================
#   D* LITE PLANNER
# ==========================================
class DLitePlanner:
    def __init__(self):
        self.adj_list = {}
        self.nodes = set()
        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0
        self.s_start = None
        self.s_goal = None

    def add_edge(self, p1, p2):
        if p1 not in self.adj_list: self.adj_list[p1] = []
        if p2 not in self.adj_list: self.adj_list[p2] = []
        if p2 not in self.adj_list[p1]: self.adj_list[p1].append(p2)
        if p1 not in self.adj_list[p2]: self.adj_list[p2].append(p1)
        self.nodes.add(p1)
        self.nodes.add(p2)

    def dist(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def calculate_key(self, s):
        g_val = self.g.get(s, float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        min_val = min(g_val, rhs_val)
        return (min_val + self.dist(self.s_start, s) + self.km, min_val)

    def update_vertex(self, u):
        if u != self.s_goal:
            min_rhs = float('inf')
            if u in self.adj_list:
                for neighbor in self.adj_list[u]:
                    curr_rhs = self.dist(u, neighbor) + self.g.get(neighbor, float('inf'))
                    if curr_rhs < min_rhs: min_rhs = curr_rhs
            self.rhs[u] = min_rhs
        
        g_u = self.g.get(u, float('inf'))
        rhs_u = self.rhs.get(u, float('inf'))
        if g_u != rhs_u:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U:
            if not self.U: break
            k_old, u = self.U[0]
            k_new = self.calculate_key(u)
            start_key = self.calculate_key(self.s_start)
            if self.rhs.get(self.s_start, float('inf')) == self.g.get(self.s_start, float('inf')) and k_old >= start_key:
                break
            heapq.heappop(self.U)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                if u in self.adj_list:
                    for s in self.adj_list[u]: self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                if u in self.adj_list:
                    for s in self.adj_list[u]: self.update_vertex(s)

    def find_path(self, start, goal):
        if start not in self.adj_list or goal not in self.adj_list: return None
        self.s_start = start; self.s_goal = goal
        self.km = 0; self.U = []; self.g = {}; self.rhs = {}
        self.rhs[self.s_goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.s_goal), self.s_goal))
        self.compute_shortest_path()
        if self.g.get(self.s_start, float('inf')) == float('inf'): return None
        path = [self.s_start]
        curr = self.s_start
        while curr != self.s_goal:
            neighbors = self.adj_list.get(curr, [])
            if not neighbors: break
            best_next = min(neighbors, key=lambda n: self.dist(curr, n) + self.g.get(n, float('inf')))
            if best_next in path: break
            path.append(best_next)
            curr = best_next
        return path

# ==========================================
#          YÖNETİCİ SINIFI
# ==========================================

class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')
        self.np_map = None; self.map_info = None; self.viz_data = {}; self.new_data_available = False
        
        self.robot_x = 0.0; self.robot_y = 0.0; self.robot_yaw = 0.0
        self.robot_grid_pos = None
        
        self.full_path_grid = []
        self.current_wp_index = 0
        self.current_task_index = 0 
        self.is_path_calculated = False
        self.geo_targets_grid = []
        
        self.planner = DLitePlanner()
        self.mission_complete = False
        self.last_task_advance_time = 0.0
        self.son_hesaplama_zamani = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')

        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/waypoint', MarkerArray, self.marker_callback)
        rospy.Subscriber('/konum', Pose2D, self.konum_callback)
        rospy.Subscriber('/gorev_durumu', String, self.varildi_callback)

        print(">>> SISTEM HAZIR. Bekleniyor: /map, /waypoint, /konum")

    def loop(self):
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
            except: pass
            rate.sleep()

    def recalculate_path_from_robot(self):
        if not self.geo_targets_grid or not self.planner.nodes:
            return
        
        # Robot pozisyonu kontrolü
        use_robot_pos = False
        if self.robot_grid_pos is not None:
             if self.robot_grid_pos[0] is not None:
                 use_robot_pos = True
        
        if not use_robot_pos:
            if self.geo_targets_grid: start_node = self.geo_targets_grid[0]
            else: return
        else:
            rx, ry = self.robot_grid_pos

            # Aracin tam oldugu konuma en yakin graph dugumunu bul
            start_node = min(self.planner.nodes,
                             key=lambda n: (n[0] - rx)**2 + (n[1] - ry)**2)
        
        if self.current_task_index >= len(self.geo_targets_grid):
            print(">>> TÜM GÖREVLER BİTTİ!")
            self.full_path_grid = []
            return

        goal_node = self.geo_targets_grid[self.current_task_index]

        # --- İLERİ YÖNLÜ (FORWARD-ONLY) BAŞLANGIÇ FİLTRESİ ---
        removed_edges = []
        if getattr(self, 'robot_yaw', None) is not None:
            yaw = self.robot_yaw
            neighbors = list(self.planner.adj_list.get(start_node, []))
            
            for n in neighbors:
                # Düğümün araca olan açısı (grid koordinatlarında)
                dx = n[0] - rx
                dy = n[1] - ry
                if dx == 0 and dy == 0: continue
                
                angle_to_n = math.atan2(dy, dx)
                diff = (angle_to_n - yaw + math.pi) % (2*math.pi) - math.pi
                
                # 90 dereceden fazla fark varsa (arkasındaysa) kenarı listeye ekle
                if abs(diff) > (math.pi / 2.0 + 0.1):  
                    removed_edges.append(n)
                    
            # Bütün komşuları silmemek şartıyla (çıkmaz sokak koruması)
            if len(neighbors) > len(removed_edges):
                for n in removed_edges:
                    if n in self.planner.adj_list[start_node]:
                        self.planner.adj_list[start_node].remove(n)
                    if start_node in self.planner.adj_list[n]:
                        self.planner.adj_list[n].remove(start_node)
            else:
                removed_edges = [] # Silme işlemi iptal

        # Rotayı hesapla
        path = self.planner.find_path(start_node, goal_node)
        
        # Silinen kenarları grafiğe geri yükle
        for n in removed_edges:
            if n not in self.planner.adj_list[start_node]:
                self.planner.adj_list[start_node].append(n)
            if start_node not in self.planner.adj_list[n]:
                self.planner.adj_list[n].append(start_node)
            
        if path:
            self.full_path_grid = path
            self.current_wp_index = 0
            self.is_path_calculated = True
            print(f">>> [ROTA] {len(path)} adet Waypoint oluşturuldu.")
        else:
            print("!!! [HATA] Rota bulunamadı!")
            self.is_path_calculated = False
            self.full_path_grid = []

    def publish_current_waypoint(self):
        if not self.is_path_calculated or not self.full_path_grid:
            return

        # WP1: Mevcut hedef waypoint
        wp1_index = min(self.current_wp_index, len(self.full_path_grid) - 1)
        # WP2: Bir sonraki waypoint (bakış hedefi)
        wp2_index = min(wp1_index + 1, len(self.full_path_grid) - 1)

        wx1, wy1 = self.grid_to_world(self.full_path_grid[wp1_index][0], self.full_path_grid[wp1_index][1])
        wx2, wy2 = self.grid_to_world(self.full_path_grid[wp2_index][0], self.full_path_grid[wp2_index][1])

        msg = f"{wx1:.2f},{wy1:.2f}|{wx2:.2f},{wy2:.2f}"
        self.pub_hedef.publish(msg)


    def konum_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_yaw = msg.theta
        self.robot_grid_pos = self.world_to_grid(self.robot_x, self.robot_y)

        pos_valid = (self.robot_grid_pos is not None and self.robot_grid_pos[0] is not None)
        if not pos_valid or self.mission_complete:
            self.new_data_available = True
            return

        # 1. Durum: Henüz hiç rota çizilmemişse
        if not self.is_path_calculated and self.planner.nodes and self.geo_targets_grid:
            self.recalculate_path_from_robot()

        # 2. Durum: Rota var ama araç rotadan koptuysa
        elif self.is_path_calculated and self.full_path_grid:
            min_dist = float('inf')
            for i in range(self.current_wp_index, len(self.full_path_grid)):
                gx, gy = self.full_path_grid[i]
                wx, wy = self.grid_to_world(gx, gy)
                d = math.hypot(self.robot_x - wx, self.robot_y - wy)
                if d < min_dist:
                    min_dist = d

            sapma_esigi = 5.0
            if min_dist > sapma_esigi and (time.time() - self.son_hesaplama_zamani > 5.0):
                print(f">>> [DİKKAT] Araç rotadan {min_dist:.1f}m uzaklaştı! Rota güncelleniyor...")
                self.recalculate_path_from_robot()
                self.son_hesaplama_zamani = time.time()

        self.new_data_available = True
    
    def _advance_wp_index(self):
        """Arabanın konumuna en yakın waypoint'i bul ve 2 ileri kaydır"""
        if not self.full_path_grid:
            return
        closest_idx = self.current_wp_index
        min_dist = float('inf')
        for i in range(self.current_wp_index, len(self.full_path_grid)):
            gx, gy = self.full_path_grid[i]
            wx, wy = self.grid_to_world(gx, gy)
            d = math.hypot(self.robot_x - wx, self.robot_y - wy)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        self.current_wp_index = min(closest_idx + 2, len(self.full_path_grid) - 1)

    def varildi_callback(self, msg):
        """
        /gorev_durumu topic'inden 'varildi' gelince:
        """
        if not self.is_path_calculated or self.mission_complete:
            return

        # Görev değişikliği cooldown - çoklu varildi ile durak atlama önleme (5 sn)
        now = time.time()
        if now - self.last_task_advance_time < 5.0:
            self._advance_wp_index()
            return

        # Robot'un anlik hedef noktaya olan kus ucusu mesafesini kontrol et
        gx, gy = self.geo_targets_grid[self.current_task_index]
        wx, wy = self.grid_to_world(gx, gy)
        mesafe = math.hypot(self.robot_x - wx, self.robot_y - wy)

        # Hedef duraga 5.0 metreden yakinsak gercekten varildi sayilir
        if mesafe < 5.0:
            self.current_task_index += 1
            self.last_task_advance_time = now
            if self.current_task_index >= len(self.geo_targets_grid):
                print(">>> TÜM GÖREVLER TAMAMLANDI!")
                self.is_path_calculated = False
                self.full_path_grid = []
                self.mission_complete = True
                return
            next_name = GOREV_GEOJSON['features'][self.current_task_index]['properties']['name']
            print(f">>> DURAK TAMAMLANDI! Yeni hedef: {next_name} (index: {self.current_task_index})")
            self.recalculate_path_from_robot()
        else:
            self._advance_wp_index()
            print(f">>> ARA HEDEFE VARILDI: Mesafe {mesafe:.1f}m. Index: {self.current_wp_index}")

    def map_callback(self, msg):
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.viz_data['map'] = raw_data
        self.new_data_available = True

    def marker_callback(self, msg):
        if self.np_map is None and 'map' not in self.viz_data: return
        updated = False
        
        for m in msg.markers:
            pts = m.points
            # LINE_LIST mantığına uygun olarak 2'şerli atlayarak oku (step=2)
            for i in range(0, len(pts) - 1, 2):
                p1 = self.world_to_grid(pts[i].x, pts[i].y)
                p2 = self.world_to_grid(pts[i+1].x, pts[i+1].y)
                
                if p1[0] is not None and p2[0] is not None:
                    if p1 not in self.planner.adj_list:
                        self.planner.add_edge(p1, p2); updated = True
                    elif p2 not in self.planner.adj_list[p1]:
                        self.planner.add_edge(p1, p2); updated = True
        
        if not self.geo_targets_grid and self.planner.nodes:
            for feature in GOREV_GEOJSON['features']:
                coords = feature['geometry']['coordinates']
                tg_grid = self.world_to_grid(coords[0], coords[1])
                nearest = min(self.planner.nodes, key=lambda n: (n[0]-tg_grid[0])**2 + (n[1]-tg_grid[1])**2)
                self.geo_targets_grid.append(nearest)
                
            if self.robot_grid_pos and self.robot_grid_pos[0] is not None:
                self.recalculate_path_from_robot()
                
        if updated: 
            self.new_data_available = True

    # ==========================================
    #   ÇİZİM - WAYPOINT GÖRÜNÜMÜ
    # ==========================================
    def draw(self):
        if 'map' not in self.viz_data: return
        self.ax.clear()
        
        # Hata önleme için varsayılanlar
        wp1_idx = 0
        kalan_wp = 0
        
        # 1. Arka Plan Haritası
        self.ax.imshow(self.viz_data['map'], cmap='gray_r', origin='lower', vmin=0, vmax=100)
        
        # 2. TÜM GRAPH NOKTALARI (Düğümler)
        # Haritadaki tüm olası yol ayrım noktalarını küçük gri noktalar olarak basar
        if self.planner.nodes:
            nx, ny = zip(*self.planner.nodes)
            self.ax.scatter(nx, ny, c='black', s=5, alpha=0.3, label='Yol Noktaları', zorder=1)
        
        # 3. Tüm Yol Ağı (Kenarlar - Silik Gri Çizgiler)
        for node, neighbors in self.planner.adj_list.items():
            for n in neighbors:
                self.ax.plot([node[0], n[0]], [node[1], n[1]], color='gray', alpha=0.1, linewidth=0.5, zorder=1)

        # 4. Ana Görev Durakları (GeoJSON - Hedefler)
        if self.geo_targets_grid:
            tx, ty = zip(*self.geo_targets_grid)
            self.ax.scatter(tx, ty, c='cyan', s=150, edgecolors='black', marker='o', label='Ana Duraklar', zorder=3)

        # 5. EN KISA YOL (Planlanan Rota)
        if self.is_path_calculated and self.full_path_grid:
            px, py = zip(*self.full_path_grid)
            kalan_wp = len(self.full_path_grid) - self.current_wp_index
            
            # Rota Hattı: Kalın ve belirgin kırmızı
            self.ax.plot(px, py, color='red', linewidth=3, alpha=0.7, label='En Kısa Yol', zorder=2)
            
            # WP1 (Sarı Yıldız - Aktif Hedef)
            wp1_idx = min(self.current_wp_index, len(self.full_path_grid) - 1)
            t1 = self.full_path_grid[wp1_idx]
            self.ax.scatter(t1[0], t1[1], c='yellow', s=350, marker='*', edgecolors='red', label='WP1 (Hedef)', zorder=8)

            # WP2 (Magenta Yıldız - Bakış Hedefi)
            wp2_idx = min(wp1_idx + 1, len(self.full_path_grid) - 1)
            t2 = self.full_path_grid[wp2_idx]
            if wp2_idx > wp1_idx:
                self.ax.scatter(t2[0], t2[1], c='magenta', s=200, marker='*', edgecolors='black', label='WP2 (Bakis)', zorder=7)

        # 6. Robot (ARABA)
        if self.robot_grid_pos and self.robot_grid_pos[0] is not None:
            rx, ry = self.robot_grid_pos
            self.ax.scatter(rx, ry, c='lime', s=250, marker='s', edgecolors='black', label='Araba', zorder=10)
            
            # Yön Oku
            if self.robot_yaw is not None:
                dx = 6.0 * math.cos(self.robot_yaw)
                dy = 6.0 * math.sin(self.robot_yaw)
                self.ax.arrow(rx, ry, dx, dy, color='red', width=1.2, head_width=4, zorder=11)

        # 7. LEJANT (Grafik Notları)
        self.ax.legend(loc='upper right', prop={'size': 7}, framealpha=0.5)

        # Başlık ve Bilgi
        self.ax.set_title(f"Rota Takibi | Hedef Index: {wp1_idx} | Kalan: {kalan_wp}", fontsize=12, fontweight='bold')
        self.ax.axis('off')
        
        plt.draw()

    def grid_to_world(self, gx, gy):
        if not self.map_info: return 0.0, 0.0
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        wx = (gx * res) + ox + (res / 2.0)
        wy = (gy * res) + oy + (res / 2.0)
        return wx, wy

    def world_to_grid(self, wx, wy):
        if not self.map_info: return None, None
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        return (int((wx-ox)/res), int((wy-oy)/res))

if __name__ == '__main__':
    try:
        HedefYoneticisi().loop()
    except rospy.ROSInterruptException:
        pass