#!/usr/bin/env python3

import can
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import numpy as np
from collections import deque
import time
import sys
import threading
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from can_waypoint_follower import DEFAULT_WAYPOINTS
# Import CANDecoder
try:
    from can_decoder import CANDecoder, CANMessageID
except ImportError:
    print("Hata: can_decoder.py bulunamadı.")
    sys.exit(1)

# Konfigürasyon
CAN_INTERFACE = 'vcan0'
MAP_WINDOW_SIZE = 25.0  # Metre (Harita görüş alanı yarıçapı)

# Waypoints (Varsayılan - can_waypoint_follower.py ile aynı)
# DEFAULT_WAYPOINTS = [
#     (9.8342, -34.313881),
#     (11.225352, -16.357474),
#     (11.225352, -7.227474),
#     (15.524211, -4.3727474),
#     (22.027806, -3.2479100),
#     (23.522607, -17.535281),
# ]

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
except rospy.exceptions.ROSInitException:
    print("ROS başlatılamadı!")

# Thread Başlat
t = threading.Thread(target=can_listener)
t.daemon = True
t.start()

# --- Matplotlib Arayüzü ---
plt.style.use('dark_background') # Navigasyon modu için karanlık tema
fig = plt.figure(figsize=(9, 6))
fig.canvas.manager.set_window_title('TALOS Navigasyon')
gs = fig.add_gridspec(2, 3, height_ratios=[2, 1])

# 1. Navigasyon Haritası (Üst Kısım - Tam Genişlik)
ax_map = fig.add_subplot(gs[0, :])
ax_map.set_title('Navigasyon', fontsize=10, color='white')
ax_map.grid(True, linestyle=':', alpha=0.3, color='gray')
ax_map.set_aspect('equal')
ax_map.set_facecolor('#1e1e1e') # Koyu gri arka plan

# Waypointleri çiz
wp_x = [w[0] for w in DEFAULT_WAYPOINTS]
wp_y = [w[1] for w in DEFAULT_WAYPOINTS]
ax_map.plot(wp_x, wp_y, 'ro--', markersize=5, alpha=0.7, label='Rota', zorder=1)

# Araç yolu
line_path, = ax_map.plot([], [], 'c-', linewidth=2, alpha=0.6, label='İz', zorder=2)

# Araç (Ok işareti ile yön)
# Başlangıçta boş, update'de güncellenecek
arrow_vehicle = ax_map.arrow(0, 0, 0, 0, head_width=1, head_length=1, fc='lime', ec='lime', zorder=3)

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
    global arrow_vehicle
    
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

    # --- Harita ve Navigasyon ---
    # İz çiz
    line_path.set_data(path_x, path_y)
    
    # Aracı Ok Olarak Çiz (Eski oku sil, yenisini çiz)
    if arrow_vehicle:
        arrow_vehicle.remove()
    
    # Ok uzunluğu ve yönü
    arrow_len = 2.0
    dx = arrow_len * np.cos(cyaw)
    dy = arrow_len * np.sin(cyaw)
    
    arrow_vehicle = ax_map.arrow(cx, cy, dx, dy, 
                               head_width=1.5, head_length=1.5, 
                               fc='lime', ec='white', zorder=3, width=0.3)

    # Haritayı araca ortala (Takip Modu)
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

    return (line_path, line_steer, txt_gear, txt_speed, txt_rpm, val_soc, val_volt, val_curr, txt_park)

print("Navigasyon paneli açılıyor...")
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

try:
    plt.show()
except KeyboardInterrupt:
    running = False
    plt.close(fig)
