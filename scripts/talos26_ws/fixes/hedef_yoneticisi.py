#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray 
from geometry_msgs.msg import Pose2D
import math
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
    # --- GUNEY BOLGE (mevcut rota) ---
    {
      "type": "Feature",
      "properties": { "name": "gorev_1", "description": "1. Durak - Baslangic" },
      "geometry": { "type": "Point", "coordinates": [-5.0, -34.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_2", "description": "2. Durak - Viraj oncesi" },
      "geometry": { "type": "Point", "coordinates": [9.0, -34.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_3", "description": "3. Durak - Kuzey yol" },
      "geometry": { "type": "Point", "coordinates": [11.0, -25.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_4", "description": "4. Durak - Dogu yol" },
      "geometry": { "type": "Point", "coordinates": [20.0, -22.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_5", "description": "5. Durak - Kuzey viraj" },
      "geometry": { "type": "Point", "coordinates": [25.0, -12.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_6", "description": "6. Durak - Ust yol" },
      "geometry": { "type": "Point", "coordinates": [25.0, -6.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_7", "description": "7. Durak - Bati donus" },
      "geometry": { "type": "Point", "coordinates": [15.0, -4.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_8", "description": "8. Durak - Orta kavsakk" },
      "geometry": { "type": "Point", "coordinates": [11.0, -7.0] }
    },
    # --- KUZEY BOLGE (yeni - haritanin uzak noktalari) ---
    {
      "type": "Feature",
      "properties": { "name": "gorev_9", "description": "9. Durak - Kuzeybati giris" },
      "geometry": { "type": "Point", "coordinates": [5.0, 5.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_10", "description": "10. Durak - Bati kenar" },
      "geometry": { "type": "Point", "coordinates": [-3.0, 15.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_11", "description": "11. Durak - Kuzeybati kose" },
      "geometry": { "type": "Point", "coordinates": [5.0, 25.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_12", "description": "12. Durak - Kuzey merkez" },
      "geometry": { "type": "Point", "coordinates": [15.0, 30.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_13", "description": "13. Durak - Kuzeydogu kose" },
      "geometry": { "type": "Point", "coordinates": [25.0, 25.0] }
    },
    {
      "type": "Feature",
      "properties": { "name": "gorev_14", "description": "14. Durak - Dogu kenar (FINISH)" },
      "geometry": { "type": "Point", "coordinates": [28.0, 8.0] }
    }
  ]
}

# Ana hedeflerde durma suresi (saniye)
DURAK_BEKLEME_SURESI = 3.0

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
        
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10)
        self.pub_karar = rospy.Publisher('/karar', String, queue_size=10)

        # Ana hedefte bekleme durumu
        self.durak_waiting = False
        self.durak_wait_start = 0.0

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/waypoint', MarkerArray, self.marker_callback)
        rospy.Subscriber('/konum', Pose2D, self.konum_callback)
        rospy.Subscriber('/gorev_durumu', String, self.varildi_callback)

        print(">>> SISTEM HAZIR. Bekleniyor: /map, /waypoint, /konum")

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Ana hedefte bekleme kontrolu
            if self.durak_waiting:
                import time
                elapsed = time.time() - self.durak_wait_start
                self.pub_karar.publish("dur")
                if elapsed >= DURAK_BEKLEME_SURESI:
                    self.durak_waiting = False
                    self.pub_karar.publish("normal")
                    print(f">>> [DURAK] Bekleme bitti, devam ediliyor.")
                    self.recalculate_path_from_robot()
            elif self.is_path_calculated:
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
            start_node = min(self.planner.nodes, key=lambda n: (n[0]-rx)**2 + (n[1]-ry)**2)
        
        if self.current_task_index >= len(self.geo_targets_grid):
            print(">>> TÜM GÖREVLER BİTTİ!")
            self.full_path_grid = []
            return

        goal_node = self.geo_targets_grid[self.current_task_index]
        path = self.planner.find_path(start_node, goal_node)
        
        if path:
            self.full_path_grid = path
            self.current_wp_index = 0
            self.is_path_calculated = True
            print(f">>> [ROTA] {len(path)} adet Waypoint oluşturuldu.")
        else:
            print("!!! [HATA] Rota bulunamadı!")

    def publish_current_waypoint(self):
        if self.current_wp_index >= len(self.full_path_grid): return
        gx, gy = self.full_path_grid[self.current_wp_index]
        wx, wy = self.grid_to_world(gx, gy)
        msg = f"{wx:.2f},{wy:.2f}"
        self.pub_hedef.publish(msg)

    def varildi_callback(self, msg):
        import time
        if self.durak_waiting:
            return  # Bekleme sirasinda yeni varildi'lari yoksay

        if self.is_path_calculated and self.current_wp_index < len(self.full_path_grid) - 1:
            # Ara waypoint - sadece index ilerle, durma yok
            self.current_wp_index += 1
        elif self.is_path_calculated and self.current_wp_index >= len(self.full_path_grid) - 1:
            # ANA HEDEF tamamlandi - dur ve bekle
            self.current_task_index += 1
            self.is_path_calculated = False
            if self.current_task_index < len(self.geo_targets_grid):
                gorev_no = self.current_task_index  # 0-indexed, onceki gorev
                print(f">>> [DURAK TAMAM] Durak {gorev_no} bitti. {DURAK_BEKLEME_SURESI:.0f}s bekleniyor...")
                self.durak_waiting = True
                self.durak_wait_start = time.time()
                self.pub_karar.publish("dur")
            else:
                print(">>> TEBRIKLER! TUM DURAKLAR GEZILDI.")
                self.pub_karar.publish("dur")

    def konum_callback(self, msg):
        self.robot_x = msg.x; self.robot_y = msg.y; self.robot_yaw = msg.theta
        self.robot_grid_pos = self.world_to_grid(self.robot_x, self.robot_y)
        
        pos_valid = (self.robot_grid_pos is not None and self.robot_grid_pos[0] is not None)
        
        if not self.is_path_calculated and self.planner.nodes and self.geo_targets_grid and pos_valid:
             self.recalculate_path_from_robot()
        self.new_data_available = True

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
            if m.ns != "edges":
                continue
            pts = m.points
            # LINE_LIST: noktalar cift cift gelir [start1,end1, start2,end2, ...]
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
        if updated: self.new_data_available = True

    # ==========================================
    #   ÇİZİM - WAYPOINT GÖRÜNÜMÜ
    # ==========================================
    def draw(self):
        if 'map' not in self.viz_data: return
        self.ax.clear()
        
        # Harita
        self.ax.imshow(self.viz_data['map'], cmap='gray_r', origin='lower', vmin=0, vmax=100)
        
        # Altyapı Graph'ı (Silik)
        for node, neighbors in self.planner.adj_list.items():
            for n in neighbors:
                self.ax.plot([node[0], n[0]], [node[1], n[1]], color='gray', alpha=0.3, linewidth=1, zorder=1)

        # Ana Görev Noktalarını Mavi Yap (Duraklar)
        if self.geo_targets_grid:
            tx, ty = zip(*self.geo_targets_grid)
            self.ax.scatter(tx, ty, c='cyan', s=120, edgecolors='black', linewidth=1.5, zorder=3)

        # --- ROTA: NOKTA NOKTA (WAYPOINT) GÖRÜNÜMÜ ---
        if self.is_path_calculated and self.full_path_grid:
            px, py = zip(*self.full_path_grid)
            
            # 1. Rota Çizgisi (Çok silik, sadece göz takibi için)
            self.ax.plot(px, py, color='red', linewidth=1, alpha=0.3, zorder=2)
            
            # 2. Waypoint Noktaları (Belirgin Kırmızı Boncuklar)
            self.ax.scatter(px, py, c='red', s=40, alpha=0.9, zorder=4)
            
            # 3. Aktif Hedef (Robot -> Sıradaki Waypoint)
            if self.current_wp_index < len(self.full_path_grid):
                target_node = self.full_path_grid[self.current_wp_index]
                
                # Başlangıç noktası belirle
                start_node = None
                if self.robot_grid_pos and self.robot_grid_pos[0] is not None:
                    start_node = self.robot_grid_pos
                elif self.current_wp_index > 0:
                    start_node = self.full_path_grid[self.current_wp_index - 1]
                
                # Turuncu Çizgi ve Sarı Yıldız
                if start_node:
                    self.ax.plot([start_node[0], target_node[0]], 
                                 [start_node[1], target_node[1]], 
                                 color='orange', linewidth=4, alpha=0.9, zorder=5)
                
                self.ax.scatter(target_node[0], target_node[1], c='yellow', s=250, marker='*', edgecolors='red', zorder=6)

        # Robot
        if self.robot_grid_pos and self.robot_grid_pos[0] is not None:
            rx, ry = self.robot_grid_pos
            self.ax.scatter(rx, ry, c='lime', s=200, marker='s', edgecolors='black', linewidth=2, zorder=10)
            if self.robot_yaw is not None:
                dx = 4.0 * math.cos(self.robot_yaw)
                dy = 4.0 * math.sin(self.robot_yaw)
                self.ax.arrow(rx, ry, dx, dy, color='red', width=0.8, head_width=2, zorder=11)
            
            txt = self.ax.text(rx+2, ry+2, "ARABA", color='lime', fontweight='bold', fontsize=10, zorder=12)
            txt.set_path_effects([PathEffects.withStroke(linewidth=2, foreground='black')])

        self.ax.set_title(f"Rota: {self.current_wp_index}/{len(self.full_path_grid)} Waypoint", color='black')
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