#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TALOS Waypoint Follower + Karar Hız Limiti + Engelden Kaçınma

- Waypoint takibi ile yön kontrolü
- /karar topic ile hız limiti
- /engel ve /engel_distance ile engel tespiti
- Sol/sag sektor mesafeleri ile engelden kacinma (evasion)
- /steer_angle yayinlayarak lidar taramasini direksiyonla esler
"""

import rospy
import can
import math
import sys
import threading
import time
import os
import numpy as np
from datetime import datetime
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String
from tf.transformations import euler_from_quaternion

# =============================================================================
# LOG
# =============================================================================

class Logger:
    def __init__(self):
        log_dir = "/app/logs"
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(log_dir, f"control_{ts}.log")
        self.csv_file = os.path.join(log_dir, f"data_{ts}.csv")

        with open(self.csv_file, 'w') as f:
            f.write("time,x,y,yaw,speed,karar,steer,throttle,brake,wp\n")

        self.log("TALOS Control Started")

    def log(self, msg, level="INFO"):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] [{level}] {msg}"
        print(line)
        with open(self.log_file, 'a') as f:
            f.write(line + "\n")

    def log_data(self, d):
        with open(self.csv_file, 'a') as f:
            f.write(",".join(str(v) for v in d.values()) + "\n")

    def event(self, e, detail=""):
        self.log(f"EVENT: {e} | {detail}", "EVENT")


# =============================================================================
# KONFİGÜRASYON
# =============================================================================

CAN_INTERFACE = 'vcan0'

# Hız Limitleri (km/h)
LIMIT_NORMAL = 5.0
LIMIT_SLOW = 2.5

# Direksiyon
MAX_STEER_ANGLE = 30.0

# Waypoint
ARRIVAL_THRESHOLD = 1.5
SLOWDOWN_DISTANCE = 3.0

# Şerit Değiştirme
LANE_CHANGE_STEER = 20.0
LANE_CHANGE_DURATION = 2.0

# Vites
GEAR_NEUTRAL = 1
GEAR_FORWARD = 2

# DUR bekleme
DUR_WAIT_TIME = 3.0

# Engel Mesafe Esikleri (metre)
ENGEL_ACIL_DURUS = 1.0   # Acil fren (cok yakin, kacinilamaz)
ENGEL_FREN = 2.5         # Fren + kacinma manevra
ENGEL_KACINMA = 5.0      # Yavaslama + hafif kacinma

# Engel Kacinma Parametreleri
KACINMA_STEER_MAX = 20.0    # Max kacinma direksiyon acisi (derece)
KACINMA_STEER_YAKIN = 25.0  # Yakin engelde kacinma acisi
KACINMA_MIN_FARK = 0.5      # Sol/sag mesafe farki (m) - kacinma yonu icin


# =============================================================================
# KARAR DURUMLARI
# =============================================================================

class Karar:
    NORMAL = "normal"
    SLOW = "slow"
    DUR = "dur"
    ACIL_DURUS = "acildurus"
    SAG = "sag"
    SOL = "sol"


# =============================================================================
# PID CONTROLLER
# =============================================================================

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.dt = 0.02

    def compute(self, error):
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)
        deriv = (error - self.prev_err) / self.dt
        self.prev_err = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return np.clip(out, self.out_min, self.out_max)



# =============================================================================
# ANA KONTROLCÜ
# =============================================================================

class TalosController:
    def __init__(self):
        rospy.init_node('talos_controller', anonymous=True)

        self.logger = Logger()

        # CAN
        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
            self.logger.log(f"CAN OK: {CAN_INTERFACE}")
        except OSError as e:
            self.logger.log(f"CAN ERROR: {e}", "ERROR")
            sys.exit(1)

        # Hedef (dinamik, /hedef topic'inden gelir)
        self.current_target = None   # {"x": ..., "y": ..., "name": ...}
        self.target_reached = False

        # Araç durumu
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed_ms = 0.0

        # Karar durumu
        self.karar = Karar.NORMAL
        self.karar_time = time.time()

        # DUR bekleme
        self.dur_waiting = False
        self.dur_wait_start = 0.0

        # Engel durumu
        self.engel_var = False
        self.engel_mesafe = float('inf')
        self.engel_angle = 0.0        # Engel acisi (derece, sag:+, sol:-)
        self.engel_sol_mesafe = float('inf')   # Sol sektor min mesafe
        self.engel_sag_mesafe = float('inf')   # Sag sektor min mesafe

        # Şerit değiştirme
        self.lane_change_active = False
        self.lane_change_start = 0.0
        self.lane_change_dir = 0

        # PID
        self.speed_pid = PIDController(kp=3.0, ki=0.5, kd=0.2, out_min=0, out_max=100)
        self.steer_pid = PIDController(kp=40.0, ki=0.0, kd=5.0, out_min=-MAX_STEER_ANGLE, out_max=MAX_STEER_ANGLE)

        # Durum
        self.running = True
        self.started = False

        # ROS Subscribers
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self._odom_cb)
        rospy.Subscriber('/karar', String, self._karar_cb)
        rospy.Subscriber('/hedef', String, self._hedef_cb)
        rospy.Subscriber('/engel', Int32, self._engel_cb)
        rospy.Subscriber('/engel_distance', Float32, self._engel_mesafe_cb)
        rospy.Subscriber('/engel_angle', Float32, self._engel_aci_cb)
        rospy.Subscriber('/engel_sol_mesafe', Float32, self._engel_sol_cb)
        rospy.Subscriber('/engel_sag_mesafe', Float32, self._engel_sag_cb)

        # ROS Publishers
        self.gorev_durumu_pub = rospy.Publisher('/gorev_durumu', String, queue_size=1)
        self.steer_angle_pub = rospy.Publisher('/steer_angle', Float32, queue_size=10)

        # Threads
        threading.Thread(target=self._can_listener, daemon=True).start()
        threading.Thread(target=self._speed_publisher, daemon=True).start()

        self._init_vehicle()

    def _init_vehicle(self):
        self.logger.log("Waiting for odometry...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.x == 0.0 and self.y == 0.0 and rospy.Time.now() < timeout:
            self._send_cmd(0, 100, 0, GEAR_NEUTRAL)
            time.sleep(0.1)

        self.logger.log(f"Position: ({self.x:.1f}, {self.y:.1f})")
        self.logger.log("Hedef bekleniyor (/hedef topic)")
        self.logger.log("Ready! Waiting for CAN 0x500...")

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_ms = math.sqrt(vx**2 + vy**2)

    def _karar_cb(self, msg):
        """Karar topic callback - hız limiti"""
        new_karar = msg.data.lower().strip()

        if new_karar != self.karar:
            self.logger.event("KARAR", f"{self.karar} -> {new_karar}")
            self.karar = new_karar
            self.karar_time = time.time()

            # Şerit değiştirme
            if new_karar == Karar.SAG:
                self._start_lane_change(1)
            elif new_karar == Karar.SOL:
                self._start_lane_change(-1)

            # DUR bekleme
            if new_karar == Karar.DUR and not self.dur_waiting:
                self.dur_waiting = True
                self.dur_wait_start = time.time()
                self.logger.log(f"DUR: {DUR_WAIT_TIME}s bekleniyor...")

    def _engel_cb(self, msg):
        """Engel durumu callback - 0: yok, 1: var"""
        new_val = msg.data == 1
        if new_val != self.engel_var:
            self.engel_var = new_val
            if new_val:
                self.logger.event("ENGEL", f"Engel algilandi! Mesafe: {self.engel_mesafe:.1f}m")
            else:
                self.logger.event("ENGEL", "Engel kaldirildi")

    def _engel_mesafe_cb(self, msg):
        """Engel mesafesi callback (metre)"""
        self.engel_mesafe = msg.data

    def _engel_aci_cb(self, msg):
        """Engel acisi callback (derece, sag:+, sol:-)"""
        self.engel_angle = msg.data

    def _engel_sol_cb(self, msg):
        """Sol sektor minimum mesafe callback"""
        self.engel_sol_mesafe = msg.data

    def _engel_sag_cb(self, msg):
        """Sag sektor minimum mesafe callback"""
        self.engel_sag_mesafe = msg.data

    def _hedef_cb(self, msg):
        """Hedef topic callback - yeni waypoint (format: 'x,y')"""
        try:
            parts = msg.data.strip().split(',')
            x = float(parts[0])
            y = float(parts[1])
            self.current_target = {"x": x, "y": y, "name": f"wp({x:.1f},{y:.1f})"}
            self.target_reached = False
            self.steer_pid.reset()
            self.logger.event("HEDEF", f"Yeni hedef: ({x:.1f}, {y:.1f})")
        except (ValueError, IndexError) as e:
            self.logger.log(f"Hedef parse hatasi: {e}", "ERROR")

    # =========================================================================
    # CAN
    # =========================================================================

    def _can_listener(self):
        while self.running and not rospy.is_shutdown():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.arbitration_id == 0x500 and msg.data[0] == 1:
                    if not self.started:
                        self.started = True
                        self.logger.event("START", "CAN 0x500 received")
            except:
                pass

    def _speed_publisher(self):
        """Visualizer için hız bilgisi gönder (0x301)"""
        while self.running and not rospy.is_shutdown():
            try:
                speed_kmh = self.speed_ms * 3.6
                speed_raw = int(speed_kmh * 100)
                data = speed_raw.to_bytes(2, 'little') + bytes(6)
                self.bus.send(can.Message(arbitration_id=0x301, data=data, is_extended_id=False))
            except:
                pass
            time.sleep(0.1)

    def _send_cmd(self, throttle, brake, steer, gear):
        try:
            throttle_raw = int(np.clip(throttle, 0, 100) * 100)
            brake_raw = int(np.clip(brake, 0, 100))
            steer_raw = int((np.clip(steer, -30, 30) + 500) * 10)

            data_ctrl = throttle_raw.to_bytes(2, 'little') + bytes([gear, brake_raw]) + bytes(4)
            data_steer = steer_raw.to_bytes(2, 'little') + bytes(6)

            self.bus.send(can.Message(arbitration_id=0x100, data=data_ctrl, is_extended_id=False))
            self.bus.send(can.Message(arbitration_id=0x201, data=data_steer, is_extended_id=False))
        except:
            pass

    # =========================================================================
    # YARDIMCI
    # =========================================================================

    def _dist(self, x, y):
        return math.sqrt((x - self.x)**2 + (y - self.y)**2)

    def _heading_error(self, x, y):
        """Hedefe açı hatası (radyan)"""
        target = math.atan2(y - self.y, x - self.x)
        err = target - self.yaw
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    # =========================================================================
    # ŞERİT DEĞİŞTİRME
    # =========================================================================

    def _start_lane_change(self, direction):
        self.lane_change_active = True
        self.lane_change_start = time.time()
        self.lane_change_dir = direction
        self.logger.event("LANE_CHANGE", "SAG" if direction > 0 else "SOL")

    def _get_lane_change_steer(self):
        elapsed = time.time() - self.lane_change_start
        if elapsed >= LANE_CHANGE_DURATION:
            self.lane_change_active = False
            return 0.0
        progress = elapsed / LANE_CHANGE_DURATION
        smooth = math.sin(progress * math.pi)
        return self.lane_change_dir * LANE_CHANGE_STEER * smooth

    # =========================================================================
    # ENGELDEN KAÇINMA
    # =========================================================================

    def _engel_kacinma_hesapla(self):
        """
        Engelden kacinma yonunu ve steer acisini hesaplar.
        Dondurulen deger: kacinma steer acisi (derece)
            0     = kacinma yok veya kacinamaz
            pozitif = saga kacinma
            negatif = sola kacinma
        """
        engel_mesafe = self.engel_mesafe
        if engel_mesafe > ENGEL_KACINMA:
            return 0.0

        sol = self.engel_sol_mesafe
        sag = self.engel_sag_mesafe

        # Hangi tarafta daha cok alan var?
        fark = sag - sol  # pozitif = sagda daha cok yer

        # Yeterli fark yoksa (her iki taraf da benzer) - bos taraf bul
        if abs(fark) < KACINMA_MIN_FARK:
            # Her iki tarafta da engel var veya her iki tarafta da bos
            if sol > ENGEL_FREN and sag > ENGEL_FREN:
                # Her iki tarafta da yer var, waypoint yonune dogru kacinmayi tercih et
                return 0.0
            elif sol > ENGEL_FREN:
                fark = -1.0  # sola kacin
            elif sag > ENGEL_FREN:
                fark = 1.0   # saga kacin
            else:
                return 0.0  # Hicbir yere kacinamaz

        # Kacinma gucunu mesafeye gore ayarla
        if engel_mesafe < ENGEL_FREN:
            # Yakin - guclu kacinma
            steer_mag = KACINMA_STEER_YAKIN
        else:
            # Uzak - yumusak kacinma
            t = (ENGEL_KACINMA - engel_mesafe) / (ENGEL_KACINMA - ENGEL_FREN)
            steer_mag = KACINMA_STEER_MAX * t

        # Yonu belirle: fark pozitif = sagda yer var = saga kacin (pozitif steer)
        if fark > 0:
            return steer_mag
        else:
            return -steer_mag

    # =========================================================================
    # HIZ LİMİTİ
    # =========================================================================

    def _get_speed_limit(self):
        """Karar'a göre hız limiti"""
        if self.karar == Karar.ACIL_DURUS:
            return 0.0
        elif self.karar == Karar.DUR:
            return 0.0
        elif self.karar == Karar.SLOW:
            return LIMIT_SLOW
        elif self.karar in [Karar.SAG, Karar.SOL]:
            return LIMIT_SLOW
        else:
            return LIMIT_NORMAL

    # =========================================================================
    # ANA DÖNGÜ
    # =========================================================================

    def run(self):
        rate = rospy.Rate(50)

        # Başlat bekle
        while not rospy.is_shutdown() and not self.started:
            self._send_cmd(0, 100, 0, GEAR_NEUTRAL)
            time.sleep(0.1)

        self.logger.event("MISSION_START", f"({self.x:.1f}, {self.y:.1f})")
        rospy.loginfo("GÖREV BAŞLADI!")

        loop = 0
        while not rospy.is_shutdown() and self.running:
            loop += 1

            # Hedef kontrolü
            target = self.current_target
            if target is None:
                self._send_cmd(0, 100, 0, GEAR_NEUTRAL)
                rate.sleep()
                continue

            wx, wy = target["x"], target["y"]
            target_name = target["name"]
            dist = self._dist(wx, wy)
            heading_err = self._heading_error(wx, wy)

            # Hedefe ulaştık mı?
            if dist < ARRIVAL_THRESHOLD:
                self.logger.event("HEDEF_OK", f"{target_name} ({wx:.1f}, {wy:.1f})")
                rospy.loginfo(f"Hedefe ulasildi: {target_name}")
                self.gorev_durumu_pub.publish(String(data="VARILDI"))
                self.current_target = None
                self.target_reached = True
                self.steer_pid.reset()
                continue

            # === DİREKSİYON ===
            if self.lane_change_active:
                steer = self._get_lane_change_steer()
            else:
                steer = self.steer_pid.compute(heading_err)

            # Steer angle yayinla (engel_node icin dinamik tarama)
            self.steer_angle_pub.publish(Float32(data=steer))

            # === HIZ ===
            speed_kmh = self.speed_ms * 3.6
            speed_limit = self._get_speed_limit()

            throttle = 0.0
            brake = 0.0
            gear = GEAR_FORWARD

            # === ENGEL KACINMA (en yuksek oncelik) ===
            engel_mesafe = self.engel_mesafe
            engel_kacinma_steer = self._engel_kacinma_hesapla()

            if engel_mesafe < ENGEL_ACIL_DURUS:
                # Cok yakin - acil fren (kacinamaz)
                throttle = 0.0
                brake = 100.0
                gear = GEAR_NEUTRAL
                # Yine de kacinma yonune cevir
                if engel_kacinma_steer != 0:
                    steer = engel_kacinma_steer
                self._send_cmd(throttle, brake, steer, gear)
                if loop % 25 == 0:
                    self.logger.log(
                        f"ENGEL ACIL! Mesafe: {engel_mesafe:.2f}m | "
                        f"Aci: {self.engel_angle:.1f}° | "
                        f"Sol: {self.engel_sol_mesafe:.1f}m Sag: {self.engel_sag_mesafe:.1f}m",
                        "WARN")
                rate.sleep()
                continue

            elif engel_mesafe < ENGEL_FREN:
                # Yakin engel - fren + aktif kacinma
                if engel_kacinma_steer != 0:
                    # Kacinabilecek alan var - kacinma manevra
                    steer = engel_kacinma_steer
                    brake = 40.0
                    throttle = 10.0  # Hafif gaz (kacinirken durmamak icin)
                    if loop % 25 == 0:
                        yon = "SOL" if engel_kacinma_steer < 0 else "SAG"
                        self.logger.log(
                            f"ENGEL KACINMA {yon}! Mesafe: {engel_mesafe:.2f}m | "
                            f"Steer: {steer:+.1f}° | "
                            f"Sol: {self.engel_sol_mesafe:.1f}m Sag: {self.engel_sag_mesafe:.1f}m",
                            "WARN")
                else:
                    # Her iki tarafta da alan yok - fren
                    throttle = 0.0
                    brake = 80.0
                    if loop % 25 == 0:
                        self.logger.log(
                            f"ENGEL FREN! Mesafe: {engel_mesafe:.2f}m | Kacinacak yer yok",
                            "WARN")
                self._send_cmd(throttle, brake, steer, gear)
                rate.sleep()
                continue

            elif engel_mesafe < ENGEL_KACINMA:
                # Orta mesafe - yavaslama + hafif kacinma
                speed_limit = min(speed_limit, LIMIT_SLOW)
                if engel_kacinma_steer != 0:
                    # Hafif kacinma (waypoint steer ile blend)
                    blend = (ENGEL_KACINMA - engel_mesafe) / (ENGEL_KACINMA - ENGEL_FREN)
                    steer = steer * (1.0 - blend * 0.5) + engel_kacinma_steer * blend * 0.5

            # Acil duruş (karar)
            if self.karar == Karar.ACIL_DURUS:
                throttle = 0.0
                brake = 100.0
                gear = GEAR_NEUTRAL

            # DUR
            elif self.karar == Karar.DUR:
                throttle = 0.0
                # Hız varsa güçlü fren, durmuşsa sabit fren (kayma önleme)
                if speed_kmh > 0.5:
                    brake = 80.0
                else:
                    brake = 100.0  # Duruyorken tam fren tut
                    gear = GEAR_NEUTRAL

                if self.dur_waiting:
                    if time.time() - self.dur_wait_start >= DUR_WAIT_TIME:
                        self.dur_waiting = False
                        self.logger.log("DUR bekleme tamamlandı")

            # Normal / Slow / Sag / Sol - hız limiti uygula
            else:
                # Hedefe yaklaşırken yavaşla
                effective_limit = speed_limit
                if dist < SLOWDOWN_DISTANCE:
                    effective_limit *= max(0.3, dist / SLOWDOWN_DISTANCE)

                # Büyük açı hatasında yavaşla
                if abs(heading_err) > math.radians(20):
                    effective_limit *= max(0.4, 1.0 - abs(heading_err) / math.pi)

                # Hız kontrolü
                if speed_kmh < effective_limit - 0.3:
                    speed_err = effective_limit - speed_kmh
                    throttle = self.speed_pid.compute(speed_err)
                    throttle = max(0, min(throttle, 80))
                    brake = 0.0
                elif speed_kmh > effective_limit + 0.3:
                    overspeed = speed_kmh - effective_limit
                    brake = min(60, 20 + overspeed * 10)
                    throttle = 0.0
                else:
                    throttle = 15.0
                    brake = 0.0

                # Çok düşük hızda sabit fren (kayma önleme)
                if speed_kmh < 0.2 and effective_limit < 0.5:
                    throttle = 0.0
                    brake = 50.0

            # Komut gönder
            self._send_cmd(throttle, brake, steer, gear)

            # Log
            if loop % 25 == 0:
                self.logger.log_data({
                    't': f"{time.time():.2f}",
                    'x': f"{self.x:.2f}",
                    'y': f"{self.y:.2f}",
                    'yaw': f"{math.degrees(self.yaw):.1f}",
                    'spd': f"{speed_kmh:.1f}",
                    'karar': self.karar,
                    'str': f"{steer:.1f}",
                    'thr': f"{throttle:.1f}",
                    'brk': f"{brake:.1f}",
                    'wp': target_name
                })

                if self.engel_mesafe < 99:
                    engel_str = (f"E:{self.engel_mesafe:.1f}m "
                                 f"[S:{self.engel_sol_mesafe:.1f} "
                                 f"R:{self.engel_sag_mesafe:.1f}]")
                else:
                    engel_str = "E:--"
                rospy.loginfo(
                    f"[{self.karar.upper():8}] {target_name} | "
                    f"({self.x:.1f},{self.y:.1f})->({wx:.1f},{wy:.1f}) | "
                    f"{dist:.1f}m | {speed_kmh:.1f}km/h | St:{steer:+.1f}° | {engel_str}"
                )

            rate.sleep()

        self._send_cmd(0, 100, 0, GEAR_NEUTRAL)
        self.bus.shutdown()
        self.logger.log("Shutdown")


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("=" * 50)
    print("  TALOS Waypoint Follower + Karar + Engel Kacinma")
    print("  /hedef:           Dinamik hedef (hedef_yoneticisi)")
    print("  /karar:           Hiz limiti (normal, slow, dur, acildurus)")
    print("  /engel:           Engel durumu (0/1)")
    print("  /engel_distance:  Engel mesafesi (metre)")
    print("  /engel_sol_mesafe: Sol sektor mesafe")
    print("  /engel_sag_mesafe: Sag sektor mesafe")
    print("  -> /steer_angle:  Direksiyon (engel_node icin)")
    print("=" * 50)

    try:
        ctrl = TalosController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    main()
