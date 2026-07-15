#!/usr/bin/env python3

import can
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from matplotlib.patches import Polygon as MplPolygon, Circle
import numpy as np
from collections import deque
import time
import sys
import threading
import os
import yaml
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from PIL import Image
# Import CANDecoder
try:
    from can_decoder import CANDecoder, CANMessageID
except ImportError:
    print("Hata: can_decoder.py bulunamadı.")
    sys.exit(1)

# Konfigürasyon
CAN_INTERFACE = 'vcan0'
MAP_WINDOW_SIZE = 25.0  # Metre (veri yokken kullanılan fallback görüş yarıçapı)
VIEW_MARGIN = 4.0       # Metre — tüm-harita görünümünde kenar payı

# Araç boyutları (TALOS golf arabası, metre)
VEHICLE_LENGTH = 2.5
VEHICLE_WIDTH  = 1.2
WHEELBASE      = 1.86  # Bee1 dingil (2026-07-04 hizalama; control.py ile eşit)
WHEEL_LENGTH   = 0.4
WHEEL_WIDTH    = 0.15

# --- Güncel harita + graph dosya yolları (statik, yerel dosyadan) ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _first_existing(candidates):
    """Verilen aday yollardan ilk var olanı (mutlak) döndür, yoksa None."""
    for c in candidates:
        if c and os.path.exists(c):
            return os.path.abspath(c)
    return None


# Harita: map_server çıktısı my_map.pgm + my_map.yaml
MAP_YAML_PATH = _first_existing([
    '/maps/my_map.yaml',                                      # docker bind-mount
    os.path.join(SCRIPT_DIR, '..', 'maps', 'my_map.yaml'),    # yerel repo
    '/home/hilmi/talos-sim/scripts/talos26_ws/maps/my_map.yaml',
])

# Güncel graph: dual_lane_graph.yaml (düğüm + kendi edges anahtarı)
GRAPH_YAML_PATH = _first_existing([
    '/missions/dual_lane_graph.yaml',                              # docker bind-mount
    os.path.join(SCRIPT_DIR, '..', 'missions', 'dual_lane_graph.yaml'),  # yerel repo
    '/home/hilmi/talos-sim/scripts/talos26_ws/missions/dual_lane_graph.yaml',
])

# Waypoints - hedef tesliminden dinamik olarak alınır
waypoint_list = []  # [(x, y), ...] — sadece son alınan WP1 + WP2 (birikim yok)

# Statik graph (dosyadan yüklenir, çalışma boyunca sabit)
graph_nodes = []  # [(x, y), ...]
graph_edges = []  # [((x1, y1), (x2, y2)), ...]

# Statik harita (my_map.pgm, dosyadan yüklenir)
map_image = None   # numpy array (grayscale)
map_extent = None  # [xmin, xmax, ymin, ymax]

# Karar hafıza konileri + hedef silme çemberi (dinamik, ROS'tan)
hafiza_koniler = []   # [(x, y, confirmed_bool), ...] — odom-frame (karar obstacle_memory)
blok_cemberler = []   # [(cx, cy, r), ...] — hedef engel silme çemberleri
silinen_wps    = []   # [(x, y), ...] — çember içine düşen (silinen) waypoint'ler

# Karar (hedef-yoneticisi/karar-node'dan)
current_karar = "normal"       # "normal" | "slow" | "dur" | "acildurus" | "sag" | "sol"
karar_last_change_t = time.time()
karar_log_path = os.environ.get("KARAR_LOG_PATH", "/tmp/karar_transitions.log")
KARAR_COLORS = {
    "normal":    {"bg": "#1e1e1e", "text": "#7fdb7f", "label": "NORMAL"},
    "slow":      {"bg": "#3d2e1e", "text": "#ffb74d", "label": "YAVAS"},
    "dur":       {"bg": "#3d1e1e", "text": "#ff6e6e", "label": "DUR"},
    "acildurus": {"bg": "#5d0000", "text": "#ff1744", "label": "ACIL DURUS"},
    "sag":       {"bg": "#1e2e3d", "text": "#64b5f6", "label": "SAG"},
    "sol":       {"bg": "#1e2e3d", "text": "#64b5f6", "label": "SOL"},
}

# Veri Saklama
current_steer = 0.0
current_rpm = 0
current_gear = "N"
current_speed = 0.0
current_throttle = 0.0
current_brake = 0.0

# Konum ve Odometri
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
vehicle_path_x = deque(maxlen=2000)
vehicle_path_y = deque(maxlen=2000)

# Batarya ve sistem durumu
current_battery_soc = 100.0
current_battery_voltage = 48.0
current_battery_current = 0.0
current_battery_temp = 25
current_error_count = 0
current_error_level = 0
current_park_brake = False

start_time = time.time()
running = True
data_lock = threading.Lock()
bus = None

def load_map_image():
    """map_server my_map.pgm + my_map.yaml dosyasını statik arka plan olarak yükle."""
    global map_image, map_extent
    if not MAP_YAML_PATH:
        print("Harita bulunamadı (my_map.yaml).")
        return
    with open(MAP_YAML_PATH, 'r') as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    img_name = meta.get('image', 'my_map.pgm')
    # image yolu yaml'a göreceli olabilir → yaml klasörüne göre çöz
    if os.path.isabs(img_name):
        pgm_path = img_name
    else:
        pgm_path = os.path.join(os.path.dirname(MAP_YAML_PATH), os.path.basename(img_name))
    if not os.path.exists(pgm_path):
        print(f"Harita görüntüsü bulunamadı: {pgm_path}")
        return
    img = np.array(Image.open(pgm_path).convert('L'))
    h, w = img.shape
    map_image = img
    # map_server: origin = haritanın sol-alt köşesi (en küçük x, y)
    map_extent = [ox, ox + w * res, oy, oy + h * res]
    print(f"Harita yüklendi: {w}x{h}, extent={[round(v, 1) for v in map_extent]}")


def load_graph():
    """dual_lane_graph.yaml'dan güncel düğüm + kenarları statik olarak yükle."""
    global graph_nodes, graph_edges
    if not GRAPH_YAML_PATH:
        print("Graph bulunamadı (dual_lane_graph.yaml).")
        return
    with open(GRAPH_YAML_PATH, 'r') as f:
        data = yaml.safe_load(f)
    coords = {}
    for n in data.get('nodes', []):
        coords[n['id']] = (n['x'], n['y'])
    graph_nodes = list(coords.values())
    edges = []
    for u, v in data.get('edges', []):
        if u in coords and v in coords:
            edges.append((coords[u], coords[v]))
    graph_edges = edges
    print(f"Graph yüklendi: {len(graph_nodes)} düğüm, {len(graph_edges)} kenar")


load_map_image()
load_graph()


def compute_overview_bounds(edges, nodes, wps, cx, cy):
    """
    Güncel graph'ı + waypointleri + aracı kapsayan görünüm sınırlarını hesapla.
    Görünüm yol grafiğine odaklanır (harita arka planda kalır, gerekirse kırpılır);
    böylece 100x100 m'lik my_map tüm kareyi yutup graph'ı küçültmez.
    Dönüş: (xmin, xmax, ymin, ymax) ya da hiç veri yoksa None.
    """
    xs, ys = [], []

    # Yol grafiği (kenarlar + düğümler — gerçek sürülebilir yayılım)
    for (x1, y1), (x2, y2) in edges:
        xs.extend([x1, x2])
        ys.extend([y1, y2])
    for nx, ny in nodes:
        xs.append(nx)
        ys.append(ny)

    # Hedef waypointler — ekran dışında kalmasınlar
    for wx, wy in wps:
        xs.append(wx)
        ys.append(wy)

    # Araç her zaman görünür kalsın
    xs.append(cx)
    ys.append(cy)

    if not xs:
        return None

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    # Dejenere (tek nokta) durumda minimum bir pencere bırak
    if xmax - xmin < 1.0:
        xmin, xmax = xmin - MAP_WINDOW_SIZE, xmax + MAP_WINDOW_SIZE
    if ymax - ymin < 1.0:
        ymin, ymax = ymin - MAP_WINDOW_SIZE, ymax + MAP_WINDOW_SIZE

    return (xmin - VIEW_MARGIN, xmax + VIEW_MARGIN,
            ymin - VIEW_MARGIN, ymax + VIEW_MARGIN)


def compute_predicted_path(x, y, yaw, steer_deg, horizon=10.0, steps=60):
    """
    Bisiklet kinematik modeli ile tahmini dönüş yolunu hesapla.
    horizon: ileriye kaç metre bakılsın
    steps: örnekleme adımı
    Dönüş: (px_array, py_array)
    """
    steer_rad = np.radians(steer_deg)
    ds = horizon / steps  # her adımda kat edilen mesafe

    px = np.zeros(steps + 1)
    py = np.zeros(steps + 1)
    heading = yaw
    px[0], py[0] = x, y

    if abs(steer_rad) < 1e-4:
        # Düz git
        for i in range(steps):
            px[i + 1] = px[i] + ds * np.cos(heading)
            py[i + 1] = py[i] + ds * np.sin(heading)
    else:
        # Dönüş yarıçapı (ön tekerleğe göre dingil açıklığı)
        R = WHEELBASE / np.tan(steer_rad)
        dtheta = ds / R  # her adımda yaw değişimi
        for i in range(steps):
            heading_mid = heading + dtheta / 2  # Runge-Kutta benzeri ortalam
            px[i + 1] = px[i] + ds * np.cos(heading_mid)
            py[i + 1] = py[i] + ds * np.sin(heading_mid)
            heading += dtheta

    return px, py


def create_vehicle_artists(ax, x, y, yaw, steer_deg):
    """
    Araç boyutunda Tesla-tarzı poligon çiz.
    Gövde, kabin, 4 tekerlek (ön tekerlekler dönük), 2 far döndürür.
    """
    artists = []
    half_L = VEHICLE_LENGTH / 2
    half_W = VEHICLE_WIDTH / 2

    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])

    def transform(pts):
        return pts @ R.T + np.array([x, y])

    # Gövde (sivri burunlu, 7 köşe)
    taper = 0.08
    body_pts = np.array([
        [-half_L,          -half_W],
        [-half_L,           half_W],
        [ half_L - 0.3,    half_W],
        [ half_L,           half_W - taper],
        [ half_L + 0.15,   0.0],
        [ half_L,          -half_W + taper],
        [ half_L - 0.3,   -half_W],
    ])
    body = MplPolygon(transform(body_pts), closed=True,
                      facecolor='#37474F', edgecolor='#B0BEC5',
                      linewidth=1.5, zorder=5, alpha=0.92)
    ax.add_patch(body)
    artists.append(body)

    # Kabin
    cabin_pts = np.array([
        [-half_L + 0.4, -half_W + 0.15],
        [-half_L + 0.4,  half_W - 0.15],
        [ half_L - 0.6,  half_W - 0.15],
        [ half_L - 0.6, -half_W + 0.15],
    ])
    cabin = MplPolygon(transform(cabin_pts), closed=True,
                       facecolor='#455A64', edgecolor='none',
                       zorder=6, alpha=0.75)
    ax.add_patch(cabin)
    artists.append(cabin)

    # Tekerlekler
    rear_ax_x  = -WHEELBASE / 2
    front_ax_x =  WHEELBASE / 2
    w_y = half_W - 0.05

    steer_rad = np.radians(steer_deg)
    cos_s, sin_s = np.cos(steer_rad), np.sin(steer_rad)
    R_steer = np.array([[cos_s, -sin_s], [sin_s, cos_s]])

    wheel_defs = [
        (rear_ax_x,  -w_y, None),
        (rear_ax_x,   w_y, None),
        (front_ax_x, -w_y, R_steer),
        (front_ax_x,  w_y, R_steer),
    ]
    hw, hh = WHEEL_LENGTH / 2, WHEEL_WIDTH / 2
    base_wheel = np.array([[-hw, -hh], [-hw, hh], [hw, hh], [hw, -hh]])
    for wx, wy, R_sw in wheel_defs:
        wpts = base_wheel.copy()
        if R_sw is not None:
            wpts = wpts @ R_sw.T
        wpts += np.array([wx, wy])
        wpoly = MplPolygon(transform(wpts), closed=True,
                           facecolor='#212121', edgecolor='#616161',
                           linewidth=0.8, zorder=7)
        ax.add_patch(wpoly)
        artists.append(wpoly)

    # Farlar
    for sign in [-1, 1]:
        hl_pts = np.array([
            [half_L - 0.05, sign * (half_W - 0.30)],
            [half_L + 0.05, sign * (half_W - 0.30)],
            [half_L + 0.05, sign * (half_W - 0.10)],
            [half_L - 0.05, sign * (half_W - 0.10)],
        ])
        hl = MplPolygon(transform(hl_pts), closed=True,
                        facecolor='#FFEE58', edgecolor='none',
                        zorder=8, alpha=0.95)
        ax.add_patch(hl)
        artists.append(hl)

    return artists


# ROS Callback
def odom_callback(msg):
    global current_x, current_y, current_yaw
    with data_lock:
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Quaternion to Euler (Yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        current_yaw = yaw

        vehicle_path_x.append(current_x)
        vehicle_path_y.append(current_y)

def hedef_callback(msg):
    """Hedef tesliminden gelen waypoint (String: 'x1,y1|x2,y2').
    Sadece güncel WP1 + WP2'yi tut (birikim yok)."""
    global waypoint_list
    try:
        raw = msg.data.strip()
        # hedef-yoneticisi '|' ile ayırır; ';' geri-uyum
        sep = '|' if '|' in raw else ';'
        segments = raw.split(sep)
        new_wps = []
        for seg in segments:
            parts = seg.split(',')
            x, y = float(parts[0]), float(parts[1])
            new_wps.append((x, y))
        with data_lock:
            waypoint_list = new_wps  # önceki WP'leri at, sadece güncel olanları tut
    except (ValueError, IndexError):
        pass


def karar_callback(msg):
    """/karar (String) — karar-node'un yayını. Değişimi logla + arka plan rengini güncelle."""
    global current_karar, karar_last_change_t
    new = (msg.data or "").strip().lower()
    if not new:
        return
    with data_lock:
        if new != current_karar:
            now = time.time()
            dur = now - karar_last_change_t
            entry = (
                f"[{time.strftime('%H:%M:%S', time.localtime(now))}.{int((now%1)*1000):03d}] "
                f"KARAR: {current_karar:>10} → {new:<10}  "
                f"(önceki süre: {dur:6.1f}s)  "
                f"pos=({current_x:6.2f},{current_y:6.2f}) yaw={np.degrees(current_yaw):6.1f}°  "
                f"speed={current_speed:.2f} steer={current_steer:.1f} thr={current_throttle:.2f} brk={current_brake:.2f}"
            )
            print(entry, flush=True)
            try:
                with open(karar_log_path, "a") as f:
                    f.write(entry + "\n")
            except OSError:
                pass
            current_karar = new
            karar_last_change_t = now

def hafiza_koni_callback(msg):
    """/karar/hafiza_koni — karar'ın obstacle_memory'sindeki dubalar
    ('x,y,conf|...'; conf=1 konfirme). Odom-frame → doğrudan ax_map'e."""
    global hafiza_koniler
    out = []
    raw = (msg.data or "").strip()
    if raw:
        for seg in raw.split('|'):
            parts = seg.split(',')
            try:
                out.append((float(parts[0]), float(parts[1]), parts[2].strip() == '1'))
            except (ValueError, IndexError):
                pass
    with data_lock:
        hafiza_koniler = out


def blok_callback(msg):
    """/hedef/blok — hedefin silme çemberleri + silinen waypoint'ler
    ('cx,cy,r|...#wx,wy|...'). '#' ile iki kısım ayrılır."""
    global blok_cemberler, silinen_wps
    cember_str, _, wp_str = (msg.data or "").partition('#')
    cs, ws = [], []
    for seg in cember_str.split('|'):
        if not seg.strip():
            continue
        parts = seg.split(',')
        try:
            cs.append((float(parts[0]), float(parts[1]), float(parts[2])))
        except (ValueError, IndexError):
            pass
    for seg in wp_str.split('|'):
        if not seg.strip():
            continue
        parts = seg.split(',')
        try:
            ws.append((float(parts[0]), float(parts[1])))
        except (ValueError, IndexError):
            pass
    with data_lock:
        blok_cemberler = cs
        silinen_wps = ws


def can_listener():
    """Arka planda CAN dinleyen thread"""
    global current_steer, current_rpm, current_gear, current_speed, current_throttle, current_brake, running
    global current_battery_soc, current_battery_voltage, current_battery_current, current_battery_temp
    global current_error_count, current_error_level, current_park_brake
    global bus

    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
    except OSError:
        return

    while running:
        try:
            messages_processed = 0
            
            # Geçici değişkenler
            temp_speed = None
            temp_rpm = None
            temp_throttle = None
            temp_brake = None
            temp_gear = None
            temp_steer = None
            temp_battery = None
            temp_error = None
            temp_park = None

            while messages_processed < 50: 
                msg = bus.recv(timeout=0) 
                if msg is None:
                    break

                messages_processed += 1

                if msg.arbitration_id == CANMessageID.SPEED_RPM:
                    val = CANDecoder.decode_real_speed(msg.data)
                    rpm = CANDecoder.decode_rpm(msg.data)
                    temp_speed = val
                    temp_rpm = rpm

                elif msg.arbitration_id == CANMessageID.THROTTLE_BRAKE_GEAR:
                    temp_throttle = CANDecoder.decode_speed(msg.data) / 100.0
                    temp_brake = CANDecoder.decode_brake(msg.data)
                    g = CANDecoder.decode_gear(msg.data)
                    if g == 2: temp_gear = "D"
                    elif g == 3: temp_gear = "R"
                    elif g == 1: temp_gear = "N"
                    else: temp_gear = "P"

                elif msg.arbitration_id == CANMessageID.STEERING:
                    temp_steer = CANDecoder.decode_steering(msg.data)

                elif msg.arbitration_id == CANMessageID.BATTERY_STATUS:
                    temp_battery = CANDecoder.decode_battery_status(msg.data)

                elif msg.arbitration_id == CANMessageID.ERROR_CODES:
                    temp_error = CANDecoder.decode_error_codes(msg.data)

                elif msg.arbitration_id == CANMessageID.PARK_BRAKE_STATUS:
                    temp_park = CANDecoder.decode_park_brake(msg.data)

            if messages_processed > 0:
                with data_lock:
                    if temp_speed is not None: current_speed = temp_speed
                    if temp_rpm is not None: current_rpm = temp_rpm
                    if temp_throttle is not None: current_throttle = temp_throttle
                    if temp_brake is not None: current_brake = temp_brake
                    if temp_gear is not None: current_gear = temp_gear
                    if temp_steer is not None: current_steer = temp_steer
                    
                    if temp_battery is not None:
                        current_battery_soc = temp_battery['soc']
                        current_battery_voltage = temp_battery['voltage']
                        current_battery_current = temp_battery['current']
                        current_battery_temp = temp_battery['temperature']

                    if temp_error is not None:
                        current_error_count = temp_error['error_count']
                        current_error_level = temp_error['level']

                    if temp_park is not None:
                        current_park_brake = temp_park['state'] == 1

            time.sleep(0.02)

        except Exception:
            pass

# ROS Başlat
try:
    rospy.init_node('can_visualizer_gui', anonymous=True, disable_signals=True)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, odom_callback)
    rospy.Subscriber('/hedef', String, hedef_callback)
    rospy.Subscriber('/karar', String, karar_callback)
    rospy.Subscriber('/karar/hafiza_koni', String, hafiza_koni_callback)
    rospy.Subscriber('/hedef/blok', String, blok_callback)
except rospy.exceptions.ROSInitException:
    print("ROS başlatılamadı!")

# Thread Başlat
t = threading.Thread(target=can_listener)
t.daemon = True
t.start()

# --- Matplotlib Arayüzü ---
plt.style.use('dark_background') # Navigasyon modu için karanlık tema
fig = plt.figure(figsize=(12, 9))
fig.canvas.manager.set_window_title('TALOS Navigasyon')
gs = fig.add_gridspec(2, 3, height_ratios=[3, 1])

# 1. Navigasyon Haritası (Üst Kısım - Tam Genişlik)
ax_map = fig.add_subplot(gs[0, :])
ax_map.set_title('Navigasyon', fontsize=10, color='white')
ax_map.grid(True, linestyle=':', alpha=0.3, color='gray')
ax_map.set_aspect('equal')
ax_map.set_facecolor('#1e1e1e') # Koyu gri arka plan

# Harita arka planı (statik, my_map.pgm — bir kez çizilir)
if map_image is not None and map_extent is not None:
    # PGM: 254≈boş(açık), 0=dolu(koyu), 205=bilinmeyen(gri). origin='upper':
    # map_server'da ilk satır en yüksek y'ye denk gelir → extent ile hizalanır.
    ax_map.imshow(map_image, origin='upper', extent=map_extent,
                  cmap='gray', alpha=0.7, zorder=0, interpolation='nearest')

# Güncel graph (statik): kenarlar + düğümler
if graph_edges:
    gex, gey = [], []
    for (x1, y1), (x2, y2) in graph_edges:
        gex.extend([x1, x2, None])
        gey.extend([y1, y2, None])
    ax_map.plot(gex, gey, '-', color='#4FC3F7', linewidth=1.2, alpha=0.55,
                zorder=2, label='Yol grafiği')
if graph_nodes:
    gnx = [n[0] for n in graph_nodes]
    gny = [n[1] for n in graph_nodes]
    ax_map.plot(gnx, gny, 'o', color='#0288D1', markersize=3,
                alpha=0.6, zorder=3)

# Waypoint çizimleri (dinamik güncellenir)
line_waypoints, = ax_map.plot([], [], '*', color='#FFD600', markersize=18,
                              markeredgecolor='#000', markeredgewidth=1.0,
                              alpha=0.95, label='Hedefler', zorder=10)
line_wp_connector, = ax_map.plot([], [], '-', color='#FFD600', linewidth=2,
                                 alpha=0.7, zorder=9)

# Karar overlay metni — ekranın sol üst köşesinde büyük etiket
txt_karar = ax_map.text(0.02, 0.97, 'NORMAL', transform=ax_map.transAxes,
                        fontsize=20, fontweight='bold',
                        color=KARAR_COLORS['normal']['text'],
                        verticalalignment='top',
                        bbox=dict(boxstyle='round,pad=0.4',
                                  facecolor='#000', edgecolor='#444', alpha=0.7),
                        zorder=20)

# Araç yolu
line_path, = ax_map.plot([], [], 'c-', linewidth=2, alpha=0.6, label='İz', zorder=4)

# Araç poligonu (her frame'de yeniden oluşturulur)
vehicle_artists = []

# Tesla-tarzı tahmini dönüş yolu
line_predicted, = ax_map.plot([], [], color='#00E5FF', linewidth=2.5,
                               alpha=0.85, linestyle='-', zorder=9)

# --- Karar hafıza konileri + hedef silme çemberi (dinamik) ---
# Konfirme koni: dolu turuncu üçgen; konfirme-olmayan aday: içi boş üçgen.
line_koni_conf, = ax_map.plot([], [], '^', color='#FF7043', markersize=13,
                              markeredgecolor='#000', markeredgewidth=1.0,
                              alpha=0.95, label='Hafıza koni', zorder=12)
line_koni_unconf, = ax_map.plot([], [], '^', markerfacecolor='none',
                                markeredgecolor='#FFAB91', markeredgewidth=1.4,
                                markersize=11, alpha=0.7, zorder=12)
# Silme çemberi içine düşen (silinen) waypoint'ler: kırmızı çarpı
line_silinen, = ax_map.plot([], [], 'x', color='#EF5350', markersize=9,
                            markeredgewidth=2.0, alpha=0.9, label='Silinen WP', zorder=8)
# Silme çemberleri: her frame yeniden oluşturulan Circle patch'leri
blok_circle_artists = []

ax_map.legend(loc='upper right', fontsize=7, framealpha=0.4)

# 2. Direksiyon (Alt Sol)
ax_steer = fig.add_subplot(gs[1, 0], projection='polar')
ax_steer.set_facecolor('#1e1e1e')
ax_steer.set_theta_zero_location("N")
ax_steer.set_theta_direction(-1)
ax_steer.set_thetamin(-40)
ax_steer.set_thetamax(40)
ax_steer.set_rlim(0, 1)
ax_steer.set_yticklabels([])
ax_steer.set_xticklabels(['L', '', '0', '', 'R'], fontsize=8, color='white')
ax_steer.grid(True, color='gray', alpha=0.3)
line_steer, = ax_steer.plot([0, 0], [0, 0.9], color='red', linewidth=3)
ax_steer.set_title("Direksiyon", color='white', fontsize=9, pad=10)

# 3. Hız ve Vites (Alt Orta)
ax_info = fig.add_subplot(gs[1, 1])
ax_info.axis('off')
txt_gear = ax_info.text(0.5, 0.70, 'N', fontsize=40, ha='center', va='center', fontweight='bold', color='gray')
txt_speed = ax_info.text(0.5, 0.35, '0.0', fontsize=24, ha='center', va='center', color='cyan')
txt_unit = ax_info.text(0.5, 0.20, 'km/h', fontsize=10, ha='center', va='center', color='gray')
txt_rpm = ax_info.text(0.5, 0.05, '0 RPM', fontsize=9, ha='center', va='center', color='orange')

# 4. Batarya ve Durum (Alt Sağ)
ax_status = fig.add_subplot(gs[1, 2])
ax_status.axis('off')
txt_soc = ax_status.text(0.1, 0.8, 'BAT:', fontsize=10, color='gray')
val_soc = ax_status.text(0.6, 0.8, '100%', fontsize=10, fontweight='bold', color='lime')

txt_volt = ax_status.text(0.1, 0.6, 'VOLT:', fontsize=10, color='gray')
val_volt = ax_status.text(0.6, 0.6, '48.0V', fontsize=10, color='white')

txt_curr = ax_status.text(0.1, 0.4, 'AKIM:', fontsize=10, color='gray')
val_curr = ax_status.text(0.6, 0.4, '0.0A', fontsize=10, color='white')

txt_park = ax_status.text(0.5, 0.15, 'P', fontsize=16, ha='center', va='center',
                          fontweight='bold', color='gray',
                          bbox=dict(boxstyle='round', facecolor='#333333', edgecolor='gray', pad=0.3))

# --- Başlat Butonu ---
# Konum: [left, bottom, width, height]
ax_btn = fig.add_axes([0.4, 0.92, 0.2, 0.06])
btn_start = Button(ax_btn, 'ROTA BAŞLAT', color='#2E7D32', hovercolor='#4CAF50')
btn_start.label.set_color('white')
btn_start.label.set_fontweight('bold')

def on_start_clicked(event):
    """Başlat butonuna basılınca"""
    global bus
    if bus:
        try:
            # ID 0x500: Sistem Komutları (1 = Start)
            msg = can.Message(arbitration_id=0x500, data=[1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
            bus.send(msg)
            print(">>> Rota Başlatma Komutu Gönderildi! (ID: 0x500)")
            btn_start.label.set_text('BAŞLATILDI')
            btn_start.color = '#1B5E20'
        except can.CanError as e:
            print(f"Hata: {e}")

btn_start.on_clicked(on_start_clicked)

plt.tight_layout()
plt.subplots_adjust(top=0.9, hspace=0.3)

def update_plot(frame):
    global vehicle_artists, blok_circle_artists

    with data_lock:
        c_gear = current_gear
        c_speed = current_speed
        c_rpm = current_rpm
        c_steer = current_steer
        c_soc = current_battery_soc
        c_voltage = current_battery_voltage
        c_current = current_battery_current
        c_park = current_park_brake

        cx = current_x
        cy = current_y
        cyaw = current_yaw
        path_x = list(vehicle_path_x)
        path_y = list(vehicle_path_y)
        wps = list(waypoint_list)
        c_karar = current_karar
        koniler = list(hafiza_koniler)
        cemberler = list(blok_cemberler)
        sil_wps = list(silinen_wps)

    # --- Karar bazlı arka plan rengi + overlay ---
    karar_cfg = KARAR_COLORS.get(c_karar, KARAR_COLORS['normal'])
    ax_map.set_facecolor(karar_cfg['bg'])
    txt_karar.set_text(karar_cfg['label'])
    txt_karar.set_color(karar_cfg['text'])

    # --- Harita ve Navigasyon ---
    # Hedef waypoint'leri çiz (sarı yıldız) + WP1↔WP2 çizgisi
    # DEMO: path'i çok ileriden çizme — sadece YAKIN WP'ler (engele gelene kadar
    # sağ şeritte görünür, dodge ancak araç dubaya yaklaşınca/"görünce" çizilir).
    VIS_WP_AHEAD = 2
    if wps:
        vis = wps[:VIS_WP_AHEAD]
        xs = [w[0] for w in vis]
        ys = [w[1] for w in vis]
        line_waypoints.set_data(xs, ys)
        if len(wps) >= 2:
            # araç → WP1 → WP2 zincirini de çiz
            line_wp_connector.set_data([cx, xs[0], xs[1]], [cy, ys[0], ys[1]])
        else:
            line_wp_connector.set_data([cx, xs[0]], [cy, ys[0]])
    else:
        line_waypoints.set_data([], [])
        line_wp_connector.set_data([], [])

    # İz çiz
    line_path.set_data(path_x, path_y)
    
    # Araç poligonunu çiz (eski frame'dekileri kaldır)
    for artist in vehicle_artists:
        try:
            artist.remove()
        except ValueError:
            pass
    vehicle_artists = create_vehicle_artists(ax_map, cx, cy, cyaw, c_steer)

    # Tesla-tarzı tahmini dönüş yolunu çiz (ön akstan başlat)
    front_ax_x = cx + (WHEELBASE / 2) * np.cos(cyaw)
    front_ax_y = cy + (WHEELBASE / 2) * np.sin(cyaw)
    pred_x, pred_y = compute_predicted_path(front_ax_x, front_ax_y, cyaw, c_steer, horizon=12.0, steps=80)
    line_predicted.set_data(pred_x, pred_y)

    # --- Karar hafıza konileri + hedef silme çemberi + silinen WP ---
    line_koni_conf.set_data([k[0] for k in koniler if k[2]],
                            [k[1] for k in koniler if k[2]])
    line_koni_unconf.set_data([k[0] for k in koniler if not k[2]],
                              [k[1] for k in koniler if not k[2]])
    line_silinen.set_data([w[0] for w in sil_wps], [w[1] for w in sil_wps])
    # Silme çemberlerini yeniden çiz (sayıları değişken → patch'leri yenile)
    for c in blok_circle_artists:
        try:
            c.remove()
        except ValueError:
            pass
    blok_circle_artists = []
    for (ccx, ccy, ccr) in cemberler:
        circ = Circle((ccx, ccy), ccr, fill=False, linestyle='--',
                      edgecolor='#EF5350', linewidth=1.6, alpha=0.85, zorder=7)
        ax_map.add_patch(circ)
        blok_circle_artists.append(circ)

    # Güncel graph'ı + waypointleri + aracı sığdır (genel görünüm).
    # Harita arka planda kalır; görünüm yol grafiğine odaklanır.
    bounds = compute_overview_bounds(graph_edges, graph_nodes, wps, cx, cy)
    if bounds is not None:
        ax_map.set_xlim(bounds[0], bounds[1])
        ax_map.set_ylim(bounds[2], bounds[3])
    else:
        # Veri henüz gelmedi — geçici olarak araç çevresine bak
        ax_map.set_xlim(cx - MAP_WINDOW_SIZE, cx + MAP_WINDOW_SIZE)
        ax_map.set_ylim(cy - MAP_WINDOW_SIZE, cy + MAP_WINDOW_SIZE)

    # --- Göstergeler ---
    # Direksiyon
    rad = np.radians(c_steer)
    line_steer.set_data([0, rad], [0, 0.9])

    # Hız ve Vites
    txt_gear.set_text(c_gear)
    if c_gear == 'R': txt_gear.set_color('red')
    elif c_gear == 'D': txt_gear.set_color('lime')
    else: txt_gear.set_color('gray')

    txt_speed.set_text(f"{c_speed:.1f}")
    txt_rpm.set_text(f"{int(c_rpm)} RPM")
    
    # Batarya
    val_soc.set_text(f"{c_soc:.0f}%")
    if c_soc > 50: val_soc.set_color('lime')
    elif c_soc > 20: val_soc.set_color('orange')
    else: val_soc.set_color('red')

    val_volt.set_text(f"{c_voltage:.1f}V")
    val_curr.set_text(f"{c_current:.1f}A")

    # Park Freni
    if c_park:
        txt_park.set_text('P')
        txt_park.set_color('red')
        txt_park.set_bbox(dict(boxstyle='round', facecolor='yellow', edgecolor='red', pad=0.3))
    else:
        txt_park.set_text('P')
        txt_park.set_color('gray')
        txt_park.set_bbox(dict(boxstyle='round', facecolor='#333333', edgecolor='gray', pad=0.3))

    return (line_path, line_waypoints, line_wp_connector,
            line_koni_conf, line_koni_unconf, line_silinen,
            line_steer, txt_gear, txt_speed, txt_rpm, val_soc, val_volt,
            val_curr, txt_park, txt_karar)

print("Navigasyon paneli açılıyor...")
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

try:
    plt.show()
except KeyboardInterrupt:
    running = False
    plt.close(fig)
