# -*- coding: utf-8 -*-
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Arc
from config import (
    ENABLE_GUI, BG, PANEL_BG, EDGE_COL, NODE_COL, ROTA_MAIN, ROTA_GLOW, ROTA_SHIN,
    ROTA_PAST, DURAK_COL, WP1_COL, WP2_COL, ARABA_COL, TEXT_COL, HESAP_KILIDI_AKTIF
)

class GraphVisualizer:
    def __init__(self, manager, white_bg=False):
        self.manager = manager
        self.white_bg = white_bg
        self._static_drawn = False
        if not ENABLE_GUI:
            return

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_aspect('equal')
        try:
            plt.show(block=False)
        except Exception:
            pass

        self.line_rota_past = None
        self.line_rota_glow = None
        self.line_rota_main = None
        self.line_rota_shin = None
        self.scatter_wp1 = None
        self.scatter_wp2 = None
        self.scatter_car_glow = None
        self.scatter_car = None
        self.arrow_car = None
        
        # HESAPLAMA KİLİDİ gösterge patch'leri
        self.lock_body = None
        self.lock_shackle = None
        self.lock_text = None

    def draw(self) -> None:
        if not ENABLE_GUI:
            return
        wp1_idx  = 0
        kalan_wp = 0

        # ── Renk paleti ─────────────────────────────────────────────
        if getattr(self, 'white_bg', False):
            BG        = '#ffffff'
            PANEL_BG  = '#ffffff'
            EDGE_COL  = '#d0d0d0'
            NODE_COL  = '#555555'
            ROTA_MAIN = '#ff4d5e'
            ROTA_GLOW = '#e63946'
            ROTA_SHIN = '#ff9aa2'
            ROTA_PAST = '#e63946'
            DURAK_COL = '#00a8cc'
            WP1_COL   = '#f4d03f'
            WP2_COL   = '#e040fb'
            ARABA_COL = '#2ca02c'
            TEXT_COL  = '#333333'
        else:
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
            self.ax.set_xlim([-25.0, 45.0])
            self.ax.set_ylim([-45.0, 25.0])

            # ── Graph kenarları ──────────────────────────────────────────
            if hasattr(self, 'G') and self.manager.G is not None:
                for u, v, edge_data in self.manager.G.edges(data=True):
                    p1 = self.manager.G.nodes[u]['pos']
                    p2 = self.manager.G.nodes[v]['pos']
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
                for node, neighbors in self.manager.planner.adj_list.items():
                    for n in neighbors:
                        self.ax.plot([node[0], n[0]], [node[1], n[1]],
                                     color=EDGE_COL, alpha=0.85, linewidth=0.65, zorder=1)

            # ── Graph düğümleri ──────────────────────────────────────────
            if self.manager.planner.nodes:
                try:
                    nx_arr, ny_arr = zip(*self.manager.planner.nodes)
                    self.ax.scatter(nx_arr, ny_arr,
                                    c=NODE_COL, s=5, alpha=0.65, zorder=2, linewidths=0)
                except ValueError:
                    pass

            # ── Ana duraklar ─────────────────────────────────────────────
            if self.manager.geo_targets_world:
                try:
                    tx, ty = zip(*self.manager.geo_targets_world)
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
                facecolor='#ffffff' if getattr(self, 'white_bg', False) else '#1e1d1b',
                edgecolor='#d0d0d0' if getattr(self, 'white_bg', False) else '#3d3c38'
            )
            for text in leg.get_texts():
                text.set_color(TEXT_COL)

            self.ax.axis('off')
            for spine in self.ax.spines.values():
                spine.set_visible(True)
                spine.set_edgecolor('#d0d0d0' if getattr(self, 'white_bg', False) else '#3d3c38')
                spine.set_linewidth(0.6)

            # ── HESAPLAMA KİLİDİ göstergesi: çizili asma kilit + metin (sol-üst) ──
            # Emoji DejaVu'da tofu → kilidi patch ile çiziyoruz (gövde + halka). Renk
            # durumu verir: KIRMIZI=kilitli, YEŞİL=açık. figure-fraction (kare figür → bozulmaz).
            from matplotlib.patches import FancyBboxPatch, Arc
            lx, ly, bw, bh = 0.05, 0.80, 0.055, 0.05   # başlığın altı, sol-üst
            self.lock_body = FancyBboxPatch(
                (lx, ly), bw, bh, transform=self.fig.transFigure,
                boxstyle='round,pad=0.004,rounding_size=0.012',
                facecolor='#ff4d5e', edgecolor='none', zorder=30, clip_on=False)
            self.fig.add_artist(self.lock_body)
            self.lock_shackle = Arc(
                (lx + bw / 2, ly + bh), bw * 0.62, bh * 0.95, angle=0, theta1=0, theta2=180,
                transform=self.fig.transFigure, lw=2.2, edgecolor='#ff4d5e', zorder=29, clip_on=False)
            self.fig.add_artist(self.lock_shackle)
            self.lock_text = self.fig.text(
                lx + bw + 0.02, ly + bh / 2, '', transform=self.fig.transFigure,
                fontsize=9, fontfamily='monospace', fontweight='bold',
                va='center', ha='left', color='#ff4d5e', zorder=30)

            self._static_drawn = True

        # ── Rota ────────────────────────────────────────────────────
        with self.manager._wp_lock:
            path_exists = bool(self.manager.full_path_world)
            if self.manager.is_path_calculated and path_exists:
                kalan_wp = len(self.manager.full_path_world) - self.manager.current_wp_index

                if self.manager.current_wp_index > 0:
                    wx_past = [p[0] for p in self.manager.full_path_world[:self.manager.current_wp_index + 1]]
                    wy_past = [p[1] for p in self.manager.full_path_world[:self.manager.current_wp_index + 1]]
                    self.line_rota_past.set_data(wx_past, wy_past)
                else:
                    self.line_rota_past.set_data([], [])

                wx_ahead = [p[0] for p in self.manager.full_path_world[self.manager.current_wp_index:]]
                wy_ahead = [p[1] for p in self.manager.full_path_world[self.manager.current_wp_index:]]

                self.line_rota_glow.set_data(wx_ahead, wy_ahead)
                self.line_rota_main.set_data(wx_ahead, wy_ahead)
                self.line_rota_shin.set_data(wx_ahead, wy_ahead)

                wp1_idx = min(self.manager.current_wp_index + 1, len(self.manager.full_path_world) - 1)
                t1 = self.manager.full_path_world[wp1_idx]
                self.scatter_wp1.set_offsets(np.array([[t1[0], t1[1]]]))
                self.scatter_wp1.set_visible(True)

                wp2_idx = min(wp1_idx + 1, len(self.manager.full_path_world) - 1)
                if wp2_idx > wp1_idx:
                    t2 = self.manager.full_path_world[wp2_idx]
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
        if self.manager.robot_x is not None and self.manager.robot_y is not None:
            rx, ry = self.manager.robot_x, self.manager.robot_y
            self.scatter_car_glow.set_offsets(np.array([[rx, ry]]))
            self.scatter_car_glow.set_visible(True)
            self.scatter_car.set_offsets(np.array([[rx, ry]]))
            self.scatter_car.set_visible(True)
            if self.arrow_car is not None:
                self.arrow_car.remove()
                self.arrow_car = None
            if self.manager.robot_yaw is not None:
                dx = 6.0 * math.cos(self.manager.robot_yaw)
                dy = 6.0 * math.sin(self.manager.robot_yaw)
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
            f"  DÜĞÜM {len(self.manager.planner.nodes)}   "
            f"WP {wp1_idx} / {len(self.manager.full_path_world)}   "
            f"KALAN {kalan_wp}   "
            f"GÖREV {self.manager.current_task_index} / {len(self.manager.geo_targets_world)}  ",
            fontsize=8, color=TEXT_COL, fontfamily='monospace',
            loc='left', pad=7,
            bbox=dict(boxstyle='round,pad=0.4',
                      facecolor='#ffffff' if getattr(self, 'white_bg', False) else '#1e1d1b',
                      edgecolor='#d0d0d0' if getattr(self, 'white_bg', False) else '#3d3c38',
                      alpha=0.6)
        )

        # ── HESAPLAMA KİLİDİ göstergesi (çizili kilit + metin) ───────
        if getattr(self, 'lock_body', None) is not None:
            kilitli = bool(HESAP_KILIDI_AKTIF and self.manager._hesap_kilitli)
            col = '#ff4d5e' if kilitli else '#39ff14'   # kırmızı=kilitli, yeşil=açık
            self.lock_body.set_facecolor(col)
            self.lock_shackle.set_edgecolor(col)
            self.lock_text.set_color(col)
            self.lock_text.set_text('KİLİTLİ' if kilitli else 'AÇIK')

        self.fig.canvas.draw_idle()


    def flush(self) -> None:
        if not ENABLE_GUI:
            return
        try:
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception:
            pass


if __name__ == "__main__":
    import sys
    import os
    import threading
    
    # Ensure current directory is in sys.path so modules can be imported
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    
    from graph_builder import build_track_graph
    from config import GOREV_GEOJSON, HESAP_KILIDI_AKTIF
    import visualizer as vis_mod
    
    vis_mod.ENABLE_GUI = True  # Force GUI for standalone mode
    
    class MockPlanner:
        def __init__(self, G):
            self.nodes = [G.nodes[n]['pos'] for n in G.nodes() if 'pos' in G.nodes[n]]
            self.node_types = {G.nodes[n]['pos']: G.nodes[n].get('type', 'intermediate') for n in G.nodes() if 'pos' in G.nodes[n]}
            self.adj_list = {}
            for u in G.nodes():
                p_u = G.nodes[u].get('pos')
                if p_u:
                    self.adj_list[p_u] = []
                    for v in G.neighbors(u):
                        p_v = G.nodes[v].get('pos')
                        if p_v:
                            self.adj_list[p_u].append(p_v)

    class MockManager:
        def __init__(self, G):
            self.G = G
            self.planner = MockPlanner(G)
            self.geo_targets_world = []
            self.full_path_world = []
            self.current_wp_index = 0
            self.robot_x = None
            self.robot_y = None
            self.robot_yaw = None
            self.current_task_index = 0
            self._wp_lock = threading.Lock()
            self.is_path_calculated = False
            self._hesap_kilitli = False

    print("=== Standalone White Background Visualization ===")
    print("Building track graph...")
    G = build_track_graph()
    
    manager = MockManager(G)
    
    # Load targets from config GOREV_GEOJSON and snap to the nearest node
    targets = []
    for feat in GOREV_GEOJSON.get("features", []):
        coords = feat.get("geometry", {}).get("coordinates")
        if coords:
            tx, ty = coords[0], coords[1]
            nearest = min(
                manager.planner.nodes,
                key=lambda n: (n[0] - tx)**2 + (n[1] - ty)**2
            )
            targets.append(nearest)
    manager.geo_targets_world = targets
    
    visualizer = GraphVisualizer(manager, white_bg=True)
    visualizer.draw()
    
    output_path = "graph_visualized_white.png"
    plt.savefig(output_path, dpi=300, facecolor='#ffffff', edgecolor='none')
    print(f"White background visualization saved to {output_path}")
    
    try:
        print("Displaying window... Close it to finish.")
        plt.show(block=True)
    except Exception as e:
        print(f"Could not open GUI window (headless environment): {e}")
