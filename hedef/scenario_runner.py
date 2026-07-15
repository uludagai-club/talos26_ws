#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==============================================================================
# None bırakırsanız config.py'deki aktif senaryo profilinin değerleri kullanılır.
A_TURN_AWARE         = None    # Viraja duyarlı bisiklet modeli arama
B_STRICT_BLOCK       = None    # Sert engel çemberi silme (True=Sert Sil, False=Ağırlık Şişir)
C_TWO_WAY_STOP       = None    # Duraklara iki yönlü yaklaşım izni
D_SLALOM_INJECT      = None    # Sollama/Slalom crossing enjeksiyonu
E_SLALOM_ONLY_NEEDED = None    # Yalnızca kendi şeridi tıkalıysa sollama yap
F_B_PLAN_BACKUP      = None    # B Planı sentetik kaçış planı
G_HESAP_KILIDI       = None    # Sollama anında rota kilitleme
H_SADECE_ENGELDE_YENIDEN_PLANLA = None  # Yalnızca engelde yeniden planlama (3sn sonra kilit)
I_SNAP_YAW_AGIRLIK   = None  # Açısal snap ceza ağırlığı (ters yönlü düğüm engelleme)
J_KONUM_FILTRE_AKTIF  = None  # Konum filtreleme aktif (EMA + Outlier)
K_KONUM_FILTRE_ALPHA  = None  # Yumuşatma katsayısı (EMA alpha, örn: 0.85)
L_KONUM_JUMP_LIMIT_MS = None  # Outlier sıçrama hız limiti (m/s)

SIM_START_POS        = "35.20,-32.19,1.57"  # Başlangıç konumu: x,y,yaw
SIM_GOAL_STOP        = "0"                  # Hedef durak (0, 1, 2, 3 veya "x,y")
SIM_OBSTACLES        = []                   # Engeller, Örn: ["35.2,-10.0,1.0,sol"]
# ==============================================================================

import os
import sys
import argparse
import time
import math
import numpy as np

# Ensure current directory is in sys.path so modules can be imported
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# ---------------------------------------------------------
#   MOCK ROS ENVIRONMENT
# ---------------------------------------------------------
from unittest.mock import MagicMock
mock_rospy = MagicMock()
mock_rospy.loginfo = lambda msg, *args: print(f"[INFO] {msg}")
mock_rospy.logwarn = lambda msg, *args: print(f"[WARN] {msg}")
mock_rospy.logerr = lambda msg, *args: print(f"[ERROR] {msg}")
mock_rospy.loginfo_throttle = lambda interval, msg, *args: print(f"[INFO] {msg}")
mock_rospy.logwarn_throttle = lambda interval, msg, *args: print(f"[WARN] {msg}")

sys.modules['rospy'] = mock_rospy
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['hedef_logger'] = MagicMock()

# Force GUI to False during initial import to prevent canvas issues
import config as config
config.ENABLE_GUI = False

from manager import HedefYoneticisi
from dstar_lite import rota_donus_metrigi

# ---------------------------------------------------------
#   CLI PARAMETERS
# ---------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="TALOS Yol Planlayıcı Senaryo Test ve Simülatör Aracı")

    # Start and Goal Target
    parser.add_argument("--start", type=str, default=SIM_START_POS, 
                        help="Başlangıç konumu: 'x,y,yaw' (Varsayılan: A Şeridi Girişi)")
    parser.add_argument("--goal", type=str, default=SIM_GOAL_STOP, 
                        help="Hedef durak indeksi veya koordinat: 'x,y'")
    
    # Obstacles
    parser.add_argument("--obstacles", type=str, nargs="*", default=SIM_OBSTACLES,
                        help="Simüle edilecek engeller: 'x,y,radius,taraf'")

    # Config parameters switches (True/False)
    parser.add_argument("--turn-aware", action="store_true", default=None, help="Viraja duyarlı bisiklet modeli rota planlamasını AKTİF yap")
    parser.add_argument("--no-turn-aware", action="store_false", dest="turn_aware", help="Viraja duyarlı bisiklet modeli rota planlamasını KAPAT")
    
    parser.add_argument("--strict-block", action="store_true", default=None, help="Sert engel çemberi silmeyi AKTİF yap")
    parser.add_argument("--no-strict-block", action="store_false", dest="strict_block", help="Sert engel çemberi silmeyi KAPAT (Ağırlık şişirme kullan)")

    parser.add_argument("--two-way-stop", action="store_true", default=None, help="Duraklara iki yönlü yaklaşım iznini AKTİF yap")
    parser.add_argument("--no-two-way-stop", action="store_false", dest="two_way_stop", help="Duraklara iki yönlü yaklaşım iznini KAPAT")

    parser.add_argument("--slalom-inject", action="store_true", default=None, help="Sollama/Slalom crossing enjeksiyonunu AKTİF yap")
    parser.add_argument("--no-slalom-inject", action="store_false", dest="slalom_inject", help="Sollama/Slalom crossing enjeksiyonunu KAPAT")

    parser.add_argument("--slalom-only-needed", action="store_true", default=None, help="Yalnızca kendi şeridi tıkalıysa sollama yapmayı AKTİF yap")
    parser.add_argument("--no-slalom-only-needed", action="store_false", dest="slalom_only_needed", help="Yalnızca kendi şeridi tıkalıysa sollama yapmayı KAPAT")

    parser.add_argument("--b-plan-backup", action="store_true", default=None, help="B Planı (Sentetik Yaw-tabanlı) kaçışı AKTİF yap")
    parser.add_argument("--no-b-plan-backup", action="store_false", dest="b_plan_backup", help="B Planı (Sentetik Yaw-tabanlı) kaçışı KAPAT")

    parser.add_argument("--hesap-kilidi", action="store_true", default=None, help="Sollama/Overtake anında hesaplama kilidini AKTİF yap")
    parser.add_argument("--no-hesap-kilidi", action="store_false", dest="hesap_kilidi", help="Sollama/Overtake anında hesaplama kilidini KAPAT")

    parser.add_argument("--only-on-obstacle", action="store_true", default=None, help="Yalnızca engelde yeniden planlamayı (rota kilit) AKTİF yap")
    parser.add_argument("--no-only-on-obstacle", action="store_false", dest="only_on_obstacle", help="Yalnızca engelde yeniden planlamayı (rota kilit) KAPAT")

    parser.add_argument("--snap-yaw-weight", type=float, default=None, help="Açısal snap ceza ağırlığı (Örn: 15.0)")

    parser.add_argument("--konum-filtre", action="store_true", default=None, help="Konum filtrelemeyi (EMA+Outlier) AKTİF yap")
    parser.add_argument("--no-konum-filtre", action="store_false", dest="konum_filtre", help="Konum filtrelemeyi (EMA+Outlier) KAPAT")
    parser.add_argument("--konum-filtre-alpha", type=float, default=None, help="Filtre alpha yumuşatma katsayısı (Örn: 0.85)")
    parser.add_argument("--konum-jump-limit", type=float, default=None, help="Outlier sıçrama hız limiti m/s (Örn: 6.0)")

    # Output and Display options
    parser.add_argument("--output", type=str, default="scenario_output.png", help="Görsel çıktının kaydedileceği dosya (Varsayılan: scenario_output.png)")
    parser.add_argument("--show", action="store_true", help="Hesaplamadan sonra görselleştirme penceresini ekranda aç (GUI)")

    return parser.parse_args()

# ---------------------------------------------------------
#   CONFIG UPDATE HELPER
# ---------------------------------------------------------
def apply_settings(settings):
    import config as config
    import manager as manager
    import dstar_lite as dstar_lite
    
    for key, val in settings.items():
        if val is None:
            continue
        config_name = config_key_map.get(key, key)
        # Set in config
        if hasattr(config, config_name):
            setattr(config, config_name, val)
        # Set in manager
        if hasattr(manager, config_name):
            setattr(manager, config_name, val)
        # Set in dstar_lite
        if hasattr(dstar_lite, config_name):
            setattr(dstar_lite, config_name, val)

config_key_map = {
    "turn_aware": "TURN_AWARE_AKTIF",
    "strict_block": "BLOK_SERT_AKTIF",
    "two_way_stop": "IKI_YONLU_DURAK_AKTIF",
    "slalom_inject": "SLALOM_ENJEKSIYON_AKTIF",
    "slalom_only_needed": "SLALOM_YALNIZ_GEREKINCE",
    "b_plan_backup": "B_PLAN_YAW_ROTA_AKTIF",
    "hesap_kilidi": "HESAP_KILIDI_AKTIF",
    "only_on_obstacle": "SADECE_ENGELDE_YENIDEN_PLANLA",
    "snap_yaw_weight": "SNAP_YAW_AGIRLIK",
    "konum_filtre": "KONUM_FILTRE_AKTIF",
    "konum_filtre_alpha": "KONUM_FILTRE_ALPHA",
    "konum_jump_limit": "KONUM_JUMP_LIMIT_MS"
}

# Mock message class for simulating ROS callbacks
class StringMsg:
    def __init__(self, data):
        self.data = data

# ---------------------------------------------------------
#   MAIN EXECUTION
# ---------------------------------------------------------
def main():
    args = parse_args()

    # 1. Parse start state
    try:
        sp = [float(x) for x in args.start.split(",")]
        start_x, start_y, start_yaw = sp[0], sp[1], sp[2]
    except Exception as e:
        print(f"[ERROR] Başlangıç konumu çözümlenemedi (Örn: --start '35.20,-32.19,1.57'): {e}")
        return

    # 2. Parse Goal State
    goal_coord = None
    goal_geojson_idx = None
    if "," in args.goal:
        try:
            gp = [float(x) for x in args.goal.split(",")]
            goal_coord = (gp[0], gp[1])
        except Exception as e:
            print(f"[ERROR] Hedef konumu çözümlenemedi (Örn: --goal '7.31,-14.50'): {e}")
            return
    else:
        try:
            goal_geojson_idx = int(args.goal)
        except ValueError:
            print("[ERROR] Hedef indeks veya koordinat olmalıdır.")
            return

    # 3. Parse Obstacles
    obstacles = []
    for obs_str in args.obstacles:
        try:
            parts = obs_str.split(",")
            ox = float(parts[0])
            oy = float(parts[1])
            r = float(parts[2]) if len(parts) > 2 else 1.0
            taraf = parts[3].strip() if len(parts) > 3 else "sol"
            obstacles.append((ox, oy, r, taraf))
        except Exception as e:
            print(f"[WARN] Engel tanımı geçersiz ('{obs_str}'). Format 'x,y,radius,taraf' olmalıdır. Hata: {e}")

    # 4. Map Arguments to Config Keys and apply
    settings = {
        "turn_aware": args.turn_aware if args.turn_aware is not None else A_TURN_AWARE,
        "strict_block": args.strict_block if args.strict_block is not None else B_STRICT_BLOCK,
        "two_way_stop": args.two_way_stop if args.two_way_stop is not None else C_TWO_WAY_STOP,
        "slalom_inject": args.slalom_inject if args.slalom_inject is not None else D_SLALOM_INJECT,
        "slalom_only_needed": args.slalom_only_needed if args.slalom_only_needed is not None else E_SLALOM_ONLY_NEEDED,
        "b_plan_backup": args.b_plan_backup if args.b_plan_backup is not None else F_B_PLAN_BACKUP,
        "hesap_kilidi": args.hesap_kilidi if args.hesap_kilidi is not None else G_HESAP_KILIDI,
        "only_on_obstacle": args.only_on_obstacle if args.only_on_obstacle is not None else H_SADECE_ENGELDE_YENIDEN_PLANLA,
        "snap_yaw_weight": args.snap_yaw_weight if args.snap_yaw_weight is not None else I_SNAP_YAW_AGIRLIK,
        "konum_filtre": args.konum_filtre if args.konum_filtre is not None else J_KONUM_FILTRE_AKTIF,
        "konum_filtre_alpha": args.konum_filtre_alpha if args.konum_filtre_alpha is not None else K_KONUM_FILTRE_ALPHA,
        "konum_jump_limit": args.konum_jump_limit if args.konum_jump_limit is not None else L_KONUM_JUMP_LIMIT_MS
    }
    
    # We display what settings are overridden
    print("\n=== Senaryo Konfigürasyonu Overrides ===")
    for k, v in settings.items():
        if v is not None:
            config_var_name = config_key_map[k]
            print(f"  {config_var_name} -> {v}")
    print("========================================")

    apply_settings(settings)

    # 5. Instantiate mock manager
    print("\nHedefYoneticisi yükleniyor (Standalone mod)...")
    manager = HedefYoneticisi()

    # Apply parsed goal
    if goal_geojson_idx is not None:
        if goal_geojson_idx < 0 or goal_geojson_idx >= len(manager.geo_targets_world):
            print(f"[ERROR] Hedef indeks 0 ile {len(manager.geo_targets_world)-1} arasında olmalıdır!")
            return
        manager.current_task_index = goal_geojson_idx
        goal_pos = manager.geo_targets_world[goal_geojson_idx]
    else:
        # We find the nearest node to custom goal coordinates
        goal_pos = min(
            manager.planner.nodes,
            key=lambda n: (n[0] - goal_coord[0])**2 + (n[1] - goal_coord[1])**2
        )
        manager.geo_targets_world = list(manager.geo_targets_world)
        # Inject custom goal into targets list
        if manager.current_task_index < len(manager.geo_targets_world):
            manager.geo_targets_world[manager.current_task_index] = goal_pos
        else:
            manager.geo_targets_world.append(goal_pos)
            manager.current_task_index = len(manager.geo_targets_world) - 1

    # Place mock robot
    manager.robot_x = start_x
    manager.robot_y = start_y
    manager.robot_yaw = start_yaw
    manager._ilk_konum_alindi = True

    # Inject obstacles
    if obstacles:
        print(f"\nEngeller enjekte ediliyor ({len(obstacles)} adet)...")
        for idx, (ox, oy, r, taraf) in enumerate(obstacles):
            msg_str = f"sollama;{taraf};{ox};{oy};obs_{idx};{r}"
            print(f"  Engel: {msg_str}")
            manager.hedef_komut_callback(StringMsg(msg_str))

    # 6. Execute path planning
    print("\nRota hesaplanıyor...")
    start_time = time.time()
    manager.recalculate_path_from_robot("simulation_trigger")
    elapsed = (time.time() - start_time) * 1000.0

    path = manager.full_path_world
    print("\n=== Rota Planlama Sonuçları ===")
    print(f"  Hesaplama Süresi: {elapsed:.2f} ms")
    if path:
        # Calculate total distance
        dist = 0.0
        for i in range(len(path) - 1):
            dist += math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
        
        print(f"  Rota Durumu: BAŞARILI")
        print(f"  Toplam Mesafe: {dist:.2f} metre")
        print(f"  Düğüm Sayısı: {len(path)} adet")
        
        # Curvature metrics
        max_deg, min_r = rota_donus_metrigi(path, start_yaw)
        print(f"  Maks Dönüş Açısı: {max_deg:.1f}°")
        print(f"  Minimum Dönüş Yarıçapı: {min_r:.2f} metre")
    else:
        print("  Rota Durumu: ROTA BULUNAMADI! (Graf kopuk veya engeller tarafından tamamen bloke edilmiş)")
    print("================================")

    # 7. Draw and save visualization
    print(f"\nVisualizing to {args.output}...")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    # Set style
    BG        = '#2e2d2a'
    PANEL_BG  = '#272622'
    EDGE_COL  = '#3d3c38'
    NODE_COL  = '#4a4945'
    ROTA_MAIN = '#ff4d5e'
    ROTA_GLOW = '#e63946'
    DURAK_COL = '#00d9ff'
    ARABA_COL = '#39ff14'

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(PANEL_BG)
    ax.set_aspect('equal')
    ax.set_xlim([-25.0, 45.0])
    ax.set_ylim([-45.0, 25.0])

    # Draw graph edges
    for u, v, edge_data in manager.G.edges(data=True):
        p1 = manager.G.nodes[u]['pos']
        p2 = manager.G.nodes[v]['pos']
        etype = edge_data.get('type', 'lane')
        if etype == 'connection':
            ecol = '#4e79a7'
            ew = 0.55
            alpha = 0.5
        else:
            ecol = EDGE_COL
            ew = 0.5
            alpha = 0.8
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=ecol, alpha=alpha, linewidth=ew, zorder=1)

    # Draw active enjected slalom connections if any are currently present in G
    for u, v in list(manager.planner.edge_weights.keys()):
        # Check if this edge is in the G graph. If not, it means it is a dynamically enjected crossing edge!
        if not manager.G.has_edge(manager.pos_to_node.get(u, ""), manager.pos_to_node.get(v, "")):
            ax.plot([u[0], v[0]], [u[1], v[1]], color='#2ca02c', alpha=0.6, linestyle='--', linewidth=0.8, zorder=2)

    # Draw nodes
    nx_arr = [n[0] for n in manager.planner.nodes]
    ny_arr = [n[1] for n in manager.planner.nodes]
    ax.scatter(nx_arr, ny_arr, c=NODE_COL, s=5, alpha=0.5, zorder=2, edgecolors='none')

    # Draw obstacles
    for (ox, oy, r, taraf) in obstacles:
        # Draw radius circle
        circ_r = max(r, config.BLOK_YARICAP_M) if hasattr(config, 'BLOK_YARICAP_M') else r
        circle = Circle((ox, oy), circ_r, facecolor='#ff4d5e', edgecolor='#ff4d5e', alpha=0.25, zorder=3)
        ax.add_patch(circle)
        ax.scatter(ox, oy, c='#ff4d5e', s=25, marker='x', zorder=4)

    # Draw target goal
    ax.scatter(goal_pos[0], goal_pos[1], c=DURAK_COL, s=150, edgecolors='none', zorder=5)
    ax.scatter(goal_pos[0], goal_pos[1], c='none', s=60, edgecolors=DURAK_COL, linewidths=1.5, zorder=5)

    # Draw robot start position
    ax.scatter(start_x, start_y, c=ARABA_COL, s=100, marker='s', edgecolors=PANEL_BG, linewidths=1.0, zorder=6)
    dx = 3.5 * math.cos(start_yaw)
    dy = 3.5 * math.sin(start_yaw)
    ax.annotate('', xy=(start_x + dx, start_y + dy), xytext=(start_x, start_y),
                arrowprops=dict(arrowstyle='->', color=ARABA_COL, lw=2.0, mutation_scale=12), zorder=7)

    # Draw path
    if path:
        wx = [p[0] for p in path]
        wy = [p[1] for p in path]
        ax.plot(wx, wy, color=ROTA_GLOW, linewidth=7.0, alpha=0.2, zorder=8)
        ax.plot(wx, wy, color=ROTA_MAIN, linewidth=2.0, alpha=1.0, zorder=9, label="Hesaplanan Rota")

    # Legend and title
    ax.legend(facecolor='#1e1d1b', edgecolor='#3d3c38', labelcolor='#7a7a6e', loc='upper right')
    ax.axis('off')
    
    # Construct scenario description for the title
    title_parts = []
    if args.turn_aware: title_parts.append("TurnAware")
    if args.strict_block: title_parts.append("StrictBlock")
    if args.two_way_stop: title_parts.append("TwoWayStop")
    if obstacles: title_parts.append(f"{len(obstacles)} Obstacles")
    
    title_str = "TALOS Senaryo Test Çıktısı"
    if title_parts:
        title_str += " (" + ", ".join(title_parts) + ")"
        
    plt.title(title_str, color='#7a7a6e', fontfamily='monospace', fontsize=11, loc='left')
    plt.tight_layout()

    plt.savefig(args.output, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Visualization saved to {args.output}")

    if args.show:
        try:
            print("Displaying plot... Close window to exit.")
            plt.show()
        except Exception as e:
            print(f"Could not open visual window: {e}")

if __name__ == "__main__":
    main()
