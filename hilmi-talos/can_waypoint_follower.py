#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN Bus Waypoint Follower - PID Controller
Gazebo simülasyonunda aracı waypoint'lere götüren CAN tabanlı kontrol sistemi

CAN Mesajları:
    0x100: Gaz/Fren/Vites komutu gönder
    0x201: Direksiyon komutu gönder
    0x301: Gerçek hız oku (TalosStateToCAN'den)
"""

import rospy
import can
import math
import struct
import sys
import threading
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

# =============================================================================
# ARAÇ PARAMETRELERİ - TÜM AYARLAR BURADA
# =============================================================================

# --- Hız Ayarları ---
MAX_SPEED_KMH = 5.0                          # Maksimum hız (km/h)
MAX_SPEED_MS = MAX_SPEED_KMH / 3.6           # Maksimum hız (m/s) - otomatik hesaplanır
REVERSE_SPEED_RATIO = 0.6                    # Geri viteste hız oranı (0.6 = %60)
REVERSE_SPEED_KMH = MAX_SPEED_KMH * REVERSE_SPEED_RATIO  # Geri vites hızı - otomatik

# --- Direksiyon Ayarları ---
MAX_STEER_ANGLE = 30.0                       # Maksimum direksiyon açısı (derece)

# --- Waypoint Toleransları ---
ARRIVAL_THRESHOLD = 1.5                      # Waypoint'e ulaşma eşiği (metre)
SLOWDOWN_DISTANCE = 3.0                      # Yavaşlamaya başlama mesafesi (metre)
STOP_DISTANCE = 1.2                          # Tamamen durma mesafesi (metre)

# --- Geri Vites Ayarları ---
REVERSE_ANGLE_THRESHOLD = 120                # Bu açının üzerinde geri git (derece)
REVERSE_MAX_DISTANCE = 8.0                   # Bu mesafenin altında geri git (metre)
GEAR_CHANGE_STOP_TIME = 0.3                  # Vites değiştirirken bekleme (saniye)

# --- CAN Bus Ayarları ---
CAN_INTERFACE = 'vcan0'                      # CAN arayüzü

# --- Şerit Takip (Line Following) Ayarları ---
LINE_TOPIC = '/line'                         # Şerit açısı topic'i
LINE_ENABLED = True                          # Şerit takibi aktif mi?
LINE_WEIGHT = 0.15                           # Şerit düzeltme ağırlığı (0.0-0.5)
LINE_TIMEOUT = 0.5                           # Veri timeout süresi (saniye)
LINE_MAX_ANGLE = 25.0                        # Güvenilir maksimum açı (derece)
LINE_OFFSET = -5.0                           # Kamera kalibrasyonu offset (derece)

# --- Vites Sabitleri (değiştirmeyin) ---
GEAR_NEUTRAL = 1
GEAR_FORWARD = 2
GEAR_REVERSE = 3

# =============================================================================
# GELİŞMİŞ PID CONTROLLER
# =============================================================================

class PIDController:
    """
    Gelişmiş PID Kontrolcü

    Özellikler:
    - Anti-windup (integral clamping + back-calculation)
    - Derivative filtering (gürültü azaltma)
    - Derivative kick önleme (setpoint değişiminde)
    - Dinamik parametre ayarlama
    - Debug/logging desteği
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0,
                 integral_limit=5.0, derivative_filter=0.1, name="PID"):
        """
        Args:
            kp: Proportional kazanç
            ki: Integral kazanç
            kd: Derivative kazanç
            output_min: Minimum çıkış değeri
            output_max: Maksimum çıkış değeri
            integral_limit: Integral anti-windup limiti
            derivative_filter: Derivative low-pass filtre katsayısı (0-1, düşük=daha fazla filtreleme)
            name: Debug için kontrolcü adı
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.derivative_filter = derivative_filter
        self.name = name

        # İç durum
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_measurement = None
        self.dt = 0.02  # 50 Hz

        # Debug
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.last_output = 0.0

    def reset(self):
        """Kontrolcü durumunu sıfırla"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_measurement = None

    def set_gains(self, kp=None, ki=None, kd=None):
        """Kazançları dinamik olarak değiştir"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
            # Ki değiştiğinde integral'i sıfırla (opsiyonel)
        if kd is not None:
            self.kd = kd

    def compute(self, error, measurement=None):
        """
        PID çıkışını hesapla

        Args:
            error: Hata değeri (setpoint - measurement)
            measurement: Ölçüm değeri (derivative kick önleme için, opsiyonel)

        Returns:
            Kontrolcü çıkışı
        """
        # === PROPORTIONAL ===
        p_term = self.kp * error

        # === INTEGRAL (Anti-windup) ===
        self.integral += error * self.dt

        # Integral clamping
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        i_term = self.ki * self.integral

        # === DERIVATIVE ===
        # Derivative kick önleme: Setpoint değiştiğinde spike olmaması için
        # measurement üzerinden türev al (varsa)
        if measurement is not None and self.prev_measurement is not None:
            # Measurement-based derivative (daha pürüzsüz)
            raw_derivative = -(measurement - self.prev_measurement) / self.dt
        else:
            # Error-based derivative (klasik)
            raw_derivative = (error - self.prev_error) / self.dt

        # Low-pass filtre (gürültü azaltma)
        filtered_derivative = (self.derivative_filter * raw_derivative +
                               (1 - self.derivative_filter) * self.prev_derivative)

        d_term = self.kd * filtered_derivative

        # Durumları güncelle
        self.prev_error = error
        self.prev_derivative = filtered_derivative
        if measurement is not None:
            self.prev_measurement = measurement

        # === OUTPUT ===
        output = p_term + i_term + d_term

        # Saturation
        saturated_output = np.clip(output, self.output_min, self.output_max)

        # Anti-windup: Back-calculation
        # Eğer çıkış saturasyona girerse, integral'i geri hesapla
        if self.ki != 0 and output != saturated_output:
            # Saturation farkını integral'den çıkar
            self.integral -= (output - saturated_output) / self.ki * 0.5

        # Debug değerlerini sakla
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term
        self.last_output = saturated_output

        return saturated_output

    def get_debug_info(self):
        """Debug bilgisi döndür"""
        return {
            'name': self.name,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'p_term': self.last_p_term,
            'i_term': self.last_i_term,
            'd_term': self.last_d_term,
            'integral': self.integral,
            'output': self.last_output
        }


# =============================================================================
# PID PRESET'LERİ (Farklı senaryolar için hazır ayarlar)
# =============================================================================

class PIDPresets:
    """Farklı senaryolar için PID preset'leri"""

    # Hız kontrolü preset'leri
    SPEED_AGGRESSIVE = {'kp': 5.0, 'ki': 1.0, 'kd': 0.3}      # Hızlı tepki
    SPEED_NORMAL = {'kp': 3.0, 'ki': 0.5, 'kd': 0.2}          # Dengeli
    SPEED_SMOOTH = {'kp': 2.0, 'ki': 0.3, 'kd': 0.1}          # Yumuşak

    # Direksiyon kontrolü preset'leri
    STEER_AGGRESSIVE = {'kp': 50.0, 'ki': 0.5, 'kd': 8.0}     # Keskin dönüşler
    STEER_NORMAL = {'kp': 40.0, 'ki': 0.0, 'kd': 5.0}         # Dengeli
    STEER_SMOOTH = {'kp': 30.0, 'ki': 0.0, 'kd': 3.0}         # Yumuşak

    # Düşük hızda (park/manevra)
    SPEED_LOW_SPEED = {'kp': 4.0, 'ki': 0.8, 'kd': 0.2}
    STEER_LOW_SPEED = {'kp': 35.0, 'ki': 0.2, 'kd': 4.0}

    @staticmethod
    def get_speed_preset(mode='normal'):
        presets = {
            'aggressive': PIDPresets.SPEED_AGGRESSIVE,
            'normal': PIDPresets.SPEED_NORMAL,
            'smooth': PIDPresets.SPEED_SMOOTH,
            'low_speed': PIDPresets.SPEED_LOW_SPEED
        }
        return presets.get(mode, PIDPresets.SPEED_NORMAL)

    @staticmethod
    def get_steer_preset(mode='normal'):
        presets = {
            'aggressive': PIDPresets.STEER_AGGRESSIVE,
            'normal': PIDPresets.STEER_NORMAL,
            'smooth': PIDPresets.STEER_SMOOTH,
            'low_speed': PIDPresets.STEER_LOW_SPEED
        }
        return presets.get(mode, PIDPresets.STEER_NORMAL)


# =============================================================================
# ANA KONTROLCÜ
# =============================================================================

class CANWaypointFollower:
    """CAN Bus üzerinden waypoint takip eden kontrolcü"""

    def __init__(self, waypoints, pid_mode='normal'):
        """
        Args:
            waypoints: Liste [(x1, y1), (x2, y2), ...]
            pid_mode: PID preset modu ('aggressive', 'normal', 'smooth')
        """
        # ROS başlat
        rospy.init_node('can_waypoint_follower', anonymous=True)

        # CAN Bus bağlantısı
        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
            rospy.loginfo(f"CAN Bus bağlandı: {CAN_INTERFACE}")
        except OSError as e:
            rospy.logerr(f"CAN Bus bağlantı hatası: {e}")
            sys.exit(1)

        # Waypoint listesi
        self.waypoints = waypoints
        self.current_waypoint_idx = 0

        # Araç durumu
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0  # Açısal hız (dönüş hızı)
        self.speed_ms = 0.0
        self.speed_kmh = 0.0
        self.prev_yaw = 0.0

        # PID modu
        self.pid_mode = pid_mode
        speed_preset = PIDPresets.get_speed_preset(pid_mode)
        steer_preset = PIDPresets.get_steer_preset(pid_mode)

        # PID Kontrolcüler
        self.speed_pid = PIDController(
            kp=speed_preset['kp'],
            ki=speed_preset['ki'],
            kd=speed_preset['kd'],
            output_min=0.0,
            output_max=100.0,
            integral_limit=10.0,
            derivative_filter=0.3,
            name="Speed"
        )

        self.steer_pid = PIDController(
            kp=steer_preset['kp'],
            ki=steer_preset['ki'],
            kd=steer_preset['kd'],
            output_min=-MAX_STEER_ANGLE,
            output_max=MAX_STEER_ANGLE,
            integral_limit=2.0,
            derivative_filter=0.2,
            name="Steer"
        )

        # Adaptif PID ayarları
        self.adaptive_pid_enabled = True
        self.heading_error_threshold = math.radians(30)  # 30 derece

        # Geri vites durumu
        self.reverse_mode_enabled = True
        self.current_gear = GEAR_FORWARD
        self.gear_change_time = 0.0
        self.is_changing_gear = False

        # Durum
        self.is_running = True
        self.mission_complete = False
        self.mission_started = False

        # ROS Subscriber - Odometri
        self.odom_sub = rospy.Subscriber(
            '/base_pose_ground_truth',
            Odometry,
            self._odom_callback
        )

        # Şerit takip (Line Following)
        self.line_enabled = LINE_ENABLED
        self.line_angle = 0.0                    # Şeritten gelen açı (derece)
        self.line_last_time = 0.0                # Son veri zamanı
        self.line_valid = False                  # Veri geçerli mi?

        if self.line_enabled:
            self.line_sub = rospy.Subscriber(
                LINE_TOPIC,
                Float32,
                self._line_callback
            )
            rospy.loginfo(f"Şerit takibi aktif: {LINE_TOPIC}")

        # CAN okuyucu thread
        self.can_thread = threading.Thread(target=self._can_listener)
        self.can_thread.daemon = True
        self.can_thread.start()

        rospy.loginfo("=" * 60)
        rospy.loginfo("  CAN Waypoint Follower Başlatıldı")
        rospy.loginfo(f"  Maksimum Hız: {MAX_SPEED_KMH} km/h")
        rospy.loginfo(f"  Toplam Waypoint: {len(waypoints)}")
        rospy.loginfo(f"  PID Modu: {pid_mode}")
        rospy.loginfo(f"  Şerit Takip: {'Aktif (ağırlık: ' + str(LINE_WEIGHT) + ')' if self.line_enabled else 'Kapalı'}")
        rospy.loginfo("  [DURUM] Başlatma komutu bekleniyor (CAN ID 0x500)...")
        rospy.loginfo("=" * 60)

        # Başlangıç sekansı
        self._initialize_vehicle()

    def _initialize_vehicle(self):
        """Araç başlangıç - Vitesi doğrudan FORWARD'a al (keyboard_teleop gibi)"""
        rospy.loginfo("Araç başlatılıyor...")

        # Odom'un gelmesini bekle
        rospy.loginfo("Odometri bekleniyor...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.x == 0.0 and self.y == 0.0 and rospy.Time.now() < timeout:
            self._send_can_command(throttle_pct=0, brake_pct=0, steer_deg=0, gear=GEAR_FORWARD)
            time.sleep(0.1)

        rospy.loginfo(f"Başlangıç pozisyonu: ({self.x:.2f}, {self.y:.2f})")

        # Doğrudan FORWARD viteste başla
        for _ in range(25):  # 0.5 saniye boyunca FORWARD gönder
            self._send_can_command(throttle_pct=0, brake_pct=0, steer_deg=0, gear=GEAR_FORWARD)
            time.sleep(0.02)

        rospy.loginfo("Araç hazır! Vites: FORWARD")

    def _odom_callback(self, msg):
        """Odometri callback"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Yaw açısı
        q = msg.pose.pose.orientation
        _, _, new_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Yaw rate hesapla (dönüş hızı)
        yaw_diff = new_yaw - self.prev_yaw
        # Normalize
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        self.yaw_rate = yaw_diff / 0.02  # 50 Hz varsayımı

        self.prev_yaw = self.yaw
        self.yaw = new_yaw

        # Hız (odom'dan)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_ms = math.sqrt(vx**2 + vy**2)

    def _line_callback(self, msg):
        """Şerit açısı callback (/line topic)"""
        raw_angle = msg.data

        # Offset uygula (kamera kalibrasyonu)
        self.line_angle = raw_angle + LINE_OFFSET
        self.line_last_time = time.time()

        # Güvenilirlik kontrolü
        if abs(self.line_angle) <= LINE_MAX_ANGLE:
            self.line_valid = True
        else:
            self.line_valid = False

    def _is_line_data_fresh(self):
        """Şerit verisi güncel mi kontrol et"""
        if not self.line_enabled:
            return False
        elapsed = time.time() - self.line_last_time
        return elapsed < LINE_TIMEOUT and self.line_valid

    def _get_line_correction(self):
        """Şerit takibinden direksiyon düzeltmesi al (derece)"""
        if not self._is_line_data_fresh():
            return 0.0

        # Şerit açısını düzeltme olarak kullan
        # Negatif açı = sola kayma = sağa dön (pozitif düzeltme)
        correction = -self.line_angle * LINE_WEIGHT
        return correction

    def _adapt_pid_gains(self, heading_error, distance):
        """
        Duruma göre PID kazançlarını adapte et

        - Büyük açı hatası: Hızı düşür, direksiyon agresif
        - Küçük mesafe: Daha hassas kontrol
        - Yüksek hız: Daha yumuşak direksiyon
        """
        if not self.adaptive_pid_enabled:
            return

        abs_heading_error = abs(heading_error)
        current_speed = self.speed_ms * 3.6  # km/h

        # === HIZ PID ADAPTASYONU ===
        if abs_heading_error > self.heading_error_threshold:
            # Büyük açı hatası - yavaşla ve önce düzelt
            self.speed_pid.set_gains(kp=2.0, ki=0.3)
        elif distance < SLOWDOWN_DISTANCE:
            # Yaklaşıyoruz - hassas kontrol
            self.speed_pid.set_gains(kp=4.0, ki=0.6)
        else:
            # Normal mod
            preset = PIDPresets.get_speed_preset(self.pid_mode)
            self.speed_pid.set_gains(kp=preset['kp'], ki=preset['ki'])

        # === DİREKSİYON PID ADAPTASYONU ===
        if current_speed < 1.0:
            # Çok düşük hız - agresif direksiyon
            self.steer_pid.set_gains(kp=50.0, kd=8.0)
        elif current_speed > 4.0:
            # Yüksek hız - yumuşak direksiyon (kararlılık için)
            self.steer_pid.set_gains(kp=30.0, kd=4.0)
        else:
            # Normal mod
            preset = PIDPresets.get_steer_preset(self.pid_mode)
            self.steer_pid.set_gains(kp=preset['kp'], kd=preset['kd'])

    def _should_use_reverse(self, heading_error, distance):
        """
        Geri vites kullanılması gerekip gerekmediğini belirle

        Kriterler:
        1. Hedef arkada (açı > REVERSE_ANGLE_THRESHOLD)
        2. Mesafe makul (< REVERSE_MAX_DISTANCE)
        3. Geri vites modu aktif

        Returns:
            bool: Geri vites kullanılmalı mı
        """
        if not self.reverse_mode_enabled:
            return False

        abs_heading_error_deg = abs(math.degrees(heading_error))

        # Hedef arkada mı?
        target_is_behind = abs_heading_error_deg > REVERSE_ANGLE_THRESHOLD

        # Mesafe makul mü?
        distance_ok = distance < REVERSE_MAX_DISTANCE

        return target_is_behind and distance_ok

    def _change_gear(self, new_gear):
        """
        Vites değiştir (yumuşak geçiş için dur ve bekle)

        Args:
            new_gear: Yeni vites (GEAR_FORWARD veya GEAR_REVERSE)
        """
        if new_gear == self.current_gear:
            return

        # Önce dur
        self.is_changing_gear = True
        self.gear_change_time = time.time()

        # Fren uygula
        self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=self.current_gear)

        rospy.loginfo(f"Vites değiştiriliyor: {self._gear_name(self.current_gear)} -> {self._gear_name(new_gear)}")

        self.current_gear = new_gear

    def _gear_name(self, gear):
        """Vites adını döndür"""
        names = {GEAR_NEUTRAL: 'N', GEAR_FORWARD: 'D', GEAR_REVERSE: 'R'}
        return names.get(gear, '?')

    def _reverse_heading_error(self, target_x, target_y):
        """
        Geri giderken açı hatası hesapla
        (Araç arkası hedefe bakacak şekilde)

        Returns:
            Radyan cinsinden açı hatası
        """
        # Hedefin yönü
        target_yaw = math.atan2(target_y - self.y, target_x - self.x)

        # Araç arkasının yönü (180 derece ters)
        rear_yaw = self.yaw + math.pi

        # Hata
        error = target_yaw - rear_yaw

        # Normalize
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        return error

    def _can_listener(self):
        """CAN mesajlarını okuyan arka plan thread'i"""
        while self.is_running and not rospy.is_shutdown():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    if msg.arbitration_id == 0x301:
                        # Gerçek hız (talos_state_to_can'den)
                        self.speed_kmh = struct.unpack('<H', msg.data[0:2])[0] * 0.01
                    
                    elif msg.arbitration_id == 0x500:
                        # Sistem Komutları (Byte 0: 1=Start)
                        if msg.data[0] == 1 and not self.mission_started:
                            self.mission_started = True
                            rospy.loginfo(">>> CAN Başlatma komutu alındı (0x500) <<<")
                            
            except Exception:
                pass

    def _send_can_command(self, throttle_pct, brake_pct, steer_deg, gear=GEAR_FORWARD):
        """
        CAN bus üzerinden komut gönder

        Args:
            throttle_pct: Gaz yüzdesi (0-100)
            brake_pct: Fren yüzdesi (0-100)
            steer_deg: Direksiyon açısı (derece, + sol, - sağ)
            gear: Vites (GEAR_FORWARD, GEAR_REVERSE, GEAR_NEUTRAL)
        """
        try:
            # Kontrol mesajı (ID: 0x100)
            # Byte 0-1: Gaz (throttle * 100)
            # Byte 2: Vites
            # Byte 3: Fren
            throttle_raw = int(np.clip(throttle_pct, 0, 100) * 100)
            brake_raw = int(np.clip(brake_pct, 0, 100))

            data_ctrl = throttle_raw.to_bytes(2, 'little') + \
                        bytes([gear]) + \
                        brake_raw.to_bytes(1, 'little') + \
                        bytes(4)

            # Direksiyon mesajı (ID: 0x201)
            # Format: (açı + 500) * 10
            steer_clamped = np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
            steer_raw = int((steer_clamped + 500) * 10)

            data_steer = steer_raw.to_bytes(2, 'little') + bytes(6)

            # Mesajları gönder
            msg_ctrl = can.Message(arbitration_id=0x100, data=data_ctrl, is_extended_id=False)
            msg_steer = can.Message(arbitration_id=0x201, data=data_steer, is_extended_id=False)

            self.bus.send(msg_ctrl)
            self.bus.send(msg_steer)

        except can.CanError as e:
            rospy.logwarn(f"CAN gönderim hatası: {e}")

    def _distance_to(self, target_x, target_y):
        """Hedefe mesafe"""
        return math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

    def _heading_error(self, target_x, target_y):
        """Hedefe açı hatası (radyan, -pi ile pi arası)"""
        target_yaw = math.atan2(target_y - self.y, target_x - self.x)
        error = target_yaw - self.yaw

        # Normalize (-pi, pi)
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        return error

    def _get_current_waypoint(self):
        """Mevcut hedef waypoint"""
        if self.current_waypoint_idx < len(self.waypoints):
            return self.waypoints[self.current_waypoint_idx]
        return None

    def stop(self):
        """Aracı durdur"""
        self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0)

    def park(self):
        """Aracı park et - el freni çek, vitesi N'ye al"""
        rospy.loginfo("PARK - El freni çekiliyor...")
        # Önce dur
        for _ in range(50):  # 1 saniye fren
            self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
            time.sleep(0.02)

        # El freni komutu gönder (CAN ID: 0x102)
        try:
            # Park freni aktif (1)
            data = bytes([1]) + bytes(7)
            msg = can.Message(arbitration_id=0x102, data=data, is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo("El freni ÇEKILDI - Araç park edildi")
        except can.CanError as e:
            rospy.logwarn(f"El freni CAN hatası: {e}")

    def run(self):
        """Ana kontrol döngüsü"""
        rate = rospy.Rate(50)  # 50 Hz

        # Başlatma komutunu bekle
        while not rospy.is_shutdown() and self.is_running and not self.mission_started:
            self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
            time.sleep(0.1)

        rospy.loginfo("GÖREV BAŞLATILIYOR!")
        
        while not rospy.is_shutdown() and self.is_running:
            # Mevcut waypoint'i al
            wp = self._get_current_waypoint()

            if wp is None:
                # Görev tamamlandı
                if not self.mission_complete:
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("  GÖREV TAMAMLANDI!")
                    rospy.loginfo("  Araç park ediliyor...")
                    rospy.loginfo("=" * 60)
                    self.park()  # El freni çek ve park et
                    self.mission_complete = True
                self.stop()
                rate.sleep()
                continue

            target_x, target_y = wp

            # Mesafe hesapla
            distance = self._distance_to(target_x, target_y)

            # Waypoint'e ulaştık mı?
            if distance < ARRIVAL_THRESHOLD:
                rospy.loginfo(f"Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} TAMAMLANDI!")
                # Son waypoint değilse durmadan devam et
                is_last_waypoint = (self.current_waypoint_idx == len(self.waypoints) - 1)
                if not is_last_waypoint:
                    # Ara waypoint - durmadan geç
                    self.current_waypoint_idx += 1
                    self.steer_pid.reset()  # Sadece direksiyon PID'i sıfırla
                    # Geri vitesten çık
                    if self.current_gear == GEAR_REVERSE:
                        self._change_gear(GEAR_FORWARD)
                else:
                    # Son waypoint - kısa dur
                    for _ in range(10):  # 0.2 saniye dur
                        self._send_can_command(throttle_pct=0, brake_pct=60, steer_deg=0, gear=self.current_gear)
                        time.sleep(0.02)
                    self.current_waypoint_idx += 1
                    self.speed_pid.reset()
                    self.steer_pid.reset()
                continue

            # Vites değiştirme süreci devam ediyor mu?
            if self.is_changing_gear:
                elapsed = time.time() - self.gear_change_time
                if elapsed < GEAR_CHANGE_STOP_TIME:
                    # Hala bekliyoruz, fren uygula
                    self._send_can_command(throttle_pct=0, brake_pct=60, steer_deg=0, gear=self.current_gear)
                    rate.sleep()
                    continue
                else:
                    # Bekleme bitti, devam et
                    self.is_changing_gear = False
                    rospy.loginfo(f"Vites hazır: {self._gear_name(self.current_gear)}")

            # Açı hatasını hesapla (ileri yön için)
            heading_error = self._heading_error(target_x, target_y)

            # Geri vites gerekli mi?
            use_reverse = self._should_use_reverse(heading_error, distance)

            if use_reverse and self.current_gear != GEAR_REVERSE:
                self._change_gear(GEAR_REVERSE)
                continue
            elif not use_reverse and self.current_gear == GEAR_REVERSE:
                # Artık ileri gidebiliriz
                self._change_gear(GEAR_FORWARD)
                continue

            # Geri vitesteyken açı hatasını ters hesapla
            if self.current_gear == GEAR_REVERSE:
                heading_error = self._reverse_heading_error(target_x, target_y)

            # Adaptif PID ayarlarını güncelle
            self._adapt_pid_gains(heading_error, distance)

            # ========== HIZ KONTROLÜ ==========
            # Waypoint'e çok yakınsa tamamen dur (gaz kesme)
            if distance < STOP_DISTANCE:
                throttle_pct = 0
                brake_pct = 50  # Yumuşak fren
                current_speed_kmh = self.speed_ms * 3.6
                target_speed_kmh = 0.0
            else:
                # Geri viteste maksimum hız daha düşük
                if self.current_gear == GEAR_REVERSE:
                    max_speed = REVERSE_SPEED_KMH
                else:
                    max_speed = MAX_SPEED_KMH

                base_speed = max_speed

                # Yaklaşırken yavaşla
                if distance < SLOWDOWN_DISTANCE:
                    distance_factor = max(0.3, distance / SLOWDOWN_DISTANCE)
                    base_speed *= distance_factor

                # Büyük açı hatasında yavaşla (dönüş yaparken)
                abs_heading_error = abs(heading_error)
                if abs_heading_error > math.radians(20):
                    heading_factor = max(0.4, 1.0 - (abs_heading_error / math.pi))
                    base_speed *= heading_factor

                target_speed_kmh = base_speed

                # Hız hatası
                current_speed_kmh = self.speed_ms * 3.6
                speed_error = target_speed_kmh - current_speed_kmh

                # PID çıkışı
                throttle = self.speed_pid.compute(speed_error, measurement=current_speed_kmh)

                # Gaz/fren kararı
                if throttle > 0:
                    throttle_pct = throttle
                    brake_pct = 0
                    # Hız limiti aşıldıysa gazı kes
                    if current_speed_kmh >= MAX_SPEED_KMH:
                        throttle_pct = 0
                else:
                    throttle_pct = 0
                    brake_pct = min(60, abs(throttle) * 0.5)

            # ========== DİREKSİYON KONTROLÜ ==========
            steer_deg = self.steer_pid.compute(heading_error, measurement=self.yaw)

            # Şerit takip düzeltmesi ekle
            line_correction = self._get_line_correction()
            steer_deg += line_correction

            # Direksiyon limitlerini uygula
            steer_deg = np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

            # Geri viteste direksiyon ters etki eder
            if self.current_gear == GEAR_REVERSE:
                steer_deg = -steer_deg

            # Komutu gönder
            self._send_can_command(
                throttle_pct=throttle_pct,
                brake_pct=brake_pct,
                steer_deg=steer_deg,
                gear=self.current_gear
            )

            # Debug çıktısı
            gear_str = self._gear_name(self.current_gear)
            line_str = f"L:{self.line_angle:+.1f}°" if self._is_line_data_fresh() else "L:--"
            rospy.loginfo_throttle(0.5,
                f"[{gear_str}] WP {self.current_waypoint_idx + 1}/{len(self.waypoints)} | "
                f"Mesafe: {distance:.1f}m | "
                f"Hız: {current_speed_kmh:.1f}/{target_speed_kmh:.1f} km/h | "
                f"Dir: {steer_deg:+.1f}° | {line_str}"
            )

            rate.sleep()

        # Temizlik
        self.is_running = False
        self.stop()
        self.bus.shutdown()
        rospy.loginfo("CAN Waypoint Follower kapatıldı.")


# =============================================================================
# ÖRNEK WAYPOINTLER
# =============================================================================

# Senaryo2 haritası için örnek waypoint'ler
# Bu koordinatlar simülasyon ortamına göre ayarlanmalı
DEFAULT_WAYPOINTS = [
    (-4.7047, -34.308881),
    (-1.8232, -31.086682),
    (8.8342, -34.313881),
    (11.225352, -16.357474),
    (11.225352, -7.227474),
    (15.524211, -4.3727474),
    (22.027806, -3.2479100),
    (23.522607, -17.535281),

]


# =============================================================================
# ANA FONKSİYON
# =============================================================================

def main():
    """Ana fonksiyon"""
    global MAX_SPEED_KMH, REVERSE_SPEED_KMH, REVERSE_ANGLE_THRESHOLD
    import argparse

    parser = argparse.ArgumentParser(description='TALOS CAN Waypoint Follower')
    parser.add_argument('--waypoints', '-w', type=str, default=None,
                        help='Waypoint listesi: "x1,y1 x2,y2 ..."')
    parser.add_argument('--mode', '-m', type=str, default='normal',
                        choices=['aggressive', 'normal', 'smooth'],
                        help='PID modu (varsayılan: normal)')   
    parser.add_argument('--speed', '-s', type=float, default=None,
                        help=f'Maksimum hız km/h (varsayılan: {MAX_SPEED_KMH})')
    parser.add_argument('--no-adaptive', action='store_true',
                        help='Adaptif PID\'yi devre dışı bırak')
    parser.add_argument('--no-reverse', action='store_true',
                        help='Geri vites modunu devre dışı bırak')
    parser.add_argument('--reverse-angle', type=float, default=120.0,
                        help='Geri vites açı eşiği derece (varsayılan: 120)')
    parser.add_argument('--debug', '-d', action='store_true',
                        help='Debug modunu etkinleştir')

    # ROS argümanlarını filtrele
    args, _ = parser.parse_known_args()

    # Waypoint'leri al
    waypoints = DEFAULT_WAYPOINTS
    if args.waypoints:
        try:
            waypoints = []
            for point in args.waypoints.split():
                x, y = point.split(',')
                waypoints.append((float(x), float(y)))
            print(f"Komut satırından {len(waypoints)} waypoint yüklendi.")
        except Exception as e:
            print(f"Waypoint parse hatası: {e}")
            print("Varsayılan waypoint'ler kullanılıyor.")
            waypoints = DEFAULT_WAYPOINTS

    # Global değişkenleri güncelle (sadece komut satırından verilmişse)
    if args.speed is not None:
        MAX_SPEED_KMH = args.speed
    if args.reverse_angle != 120.0:
        REVERSE_ANGLE_THRESHOLD = args.reverse_angle

    # Geri vites hızını yeniden hesapla
    REVERSE_SPEED_KMH = MAX_SPEED_KMH * REVERSE_SPEED_RATIO

    print("\n" + "=" * 60)
    print("  TALOS CAN Waypoint Follower (Gelişmiş PID + Geri Vites)")
    print("=" * 60)
    print(f"  PID Modu: {args.mode}")
    print(f"  Adaptif PID: {'Kapalı' if args.no_adaptive else 'Açık'}")
    print(f"  Geri Vites: {'Kapalı' if args.no_reverse else 'Açık'}")
    print(f"  Geri Vites Açı Eşiği: {REVERSE_ANGLE_THRESHOLD}°")
    print(f"  Maksimum Hız: {MAX_SPEED_KMH} km/h (İleri), {REVERSE_SPEED_KMH} km/h (Geri)")
    print(f"  Debug: {'Açık' if args.debug else 'Kapalı'}")
    print("-" * 60)
    print("  PID Preset Değerleri:")
    speed_p = PIDPresets.get_speed_preset(args.mode)
    steer_p = PIDPresets.get_steer_preset(args.mode)
    print(f"    Hız: kp={speed_p['kp']}, ki={speed_p['ki']}, kd={speed_p['kd']}")
    print(f"    Dir: kp={steer_p['kp']}, ki={steer_p['ki']}, kd={steer_p['kd']}")
    print("-" * 60)
    print(f"  Waypoint'ler ({len(waypoints)} adet):")
    for i, (x, y) in enumerate(waypoints):
        print(f"    {i+1}. ({x:.1f}, {y:.1f})")
    print("=" * 60)
    print("  Bekleniyor (GUI'den Başlatın)... [Çıkış: Ctrl+C]")
    print("=" * 60)

    try:
        follower = CANWaypointFollower(waypoints, pid_mode=args.mode)
        follower.adaptive_pid_enabled = not args.no_adaptive
        follower.reverse_mode_enabled = not args.no_reverse
        follower.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu.")


if __name__ == '__main__':
    main()
