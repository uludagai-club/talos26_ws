#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN Bus Waypoint Follower - PID Controller
Gazebo simülasyonunda aracı waypoint'lere götüren CAN tabanlı kontrol sistemi

CAN Mesajları:
    0x100: Gaz/Fren/Vites komutu gönder
    0x201: Direksiyon komutu gönder
    0x301: Gerçek hız yayını (visualizer için)
"""

import rospy
import can
import math
import struct
import sys
import os
import threading
import time
import datetime
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion


# =============================================================================
# LOGGER
# =============================================================================

class Logger:
    """Log ve CSV kaydedici"""

    def __init__(self, log_dir='/app/logs/'):
        self.log_dir = log_dir
        try:
            os.makedirs(log_dir, exist_ok=True)
        except OSError:
            self.log_dir = '/tmp/talos_logs/'
            os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'control_{timestamp}.log')
        self.csv_file = os.path.join(self.log_dir, f'control_{timestamp}.csv')

        # CSV header
        with open(self.csv_file, 'w') as f:
            f.write('timestamp,x,y,yaw,speed_kmh,karar,target_x,target_y,throttle,brake,steer,gear\n')

    def log(self, message):
        """Log mesajı yaz"""
        ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = f"[{ts}] {message}"
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
        except OSError:
            pass
        rospy.loginfo(message)

    def csv(self, x, y, yaw, speed_kmh, karar, target_x, target_y, throttle, brake, steer, gear):
        """CSV satırı yaz"""
        ts = time.time()
        try:
            with open(self.csv_file, 'a') as f:
                f.write(f'{ts},{x:.3f},{y:.3f},{yaw:.3f},{speed_kmh:.2f},{karar},'
                        f'{target_x:.3f},{target_y:.3f},{throttle:.1f},{brake:.1f},{steer:.1f},{gear}\n')
        except OSError:
            pass


# =============================================================================
# KARAR SINIFLARI VE SABİTLERİ
# =============================================================================

class Karar:
    NORMAL = "normal"
    SLOW = "slow"
    DUR = "dur"
    ACIL_DURUS = "acildurus"
    SAG = "sag"
    SOL = "sol"

LIMIT_SLOW = 2.5            # km/h - yavaş mod hız limiti
DUR_WAIT_TIME = 3.0         # saniye - dur kararında bekleme süresi
LANE_CHANGE_STEER = 20.0    # derece - şerit değiştirme direksiyon açısı
LANE_CHANGE_DURATION = 2.0  # saniye - şerit değiştirme süresi


# =============================================================================
# ARAÇ PARAMETRELERİ - TÜM AYARLAR BURADA
# =============================================================================

# --- Hız Ayarları ---
MAX_SPEED_KMH = 5.0                          # Maksimum hız (km/h)
MAX_SPEED_MS = MAX_SPEED_KMH / 3.6           # Maksimum hız (m/s) - otomatik hesaplanır
# --- Direksiyon Ayarları ---
MAX_STEER_ANGLE = 30.0                       # Maksimum direksiyon açısı (derece)

# --- Waypoint Toleransları ---
ARRIVAL_THRESHOLD = 3.0                      # Waypoint'e ulaşma eşiği (metre)
SLOWDOWN_DISTANCE = 4.0                      # Yavaşlamaya başlama mesafesi (metre)
STOP_DISTANCE = 1.2                          # Tamamen durma mesafesi (metre)

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

    def __init__(self, pid_mode='normal'):
        """
        Args:
            pid_mode: PID preset modu ('aggressive', 'normal', 'smooth')
        """
        # ROS başlat
        rospy.init_node('can_waypoint_follower', anonymous=True)

        # Logger
        self.logger = Logger()

        # CAN Bus bağlantısı
        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
            self.logger.log(f"CAN Bus bağlandı: {CAN_INTERFACE}")
        except OSError as e:
            rospy.logerr(f"CAN Bus bağlantı hatası: {e}")
            sys.exit(1)

        # Dinamik hedef (/hedef topic'inden)
        self.dynamic_target = None  # (x, y) tuple veya None
        self.next_target = None     # Sonraki hedef (gecikmeyi onlemek icin)
        self.last_completed_target = None  # Son tamamlanan hedef (stale filtreleme)

        # Karar durumu
        self.karar = Karar.NORMAL
        self.lane_change_active = False
        self.lane_change_start = 0
        self.lane_change_dir = 0  # -1: sağ, +1: sol

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

        # /gorev_durumu publisher - waypoint'e varış bildirimi
        self.pub_gorev = rospy.Publisher('/gorev_durumu', String, queue_size=10)

        # /hedef subscriber - dinamik hedef teslimi
        self.hedef_sub = rospy.Subscriber('/hedef', String, self._hedef_callback)

        # /karar subscriber - karar entegrasyonu
        self.karar_sub = rospy.Subscriber('/karar', String, self._karar_callback)

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
            self.logger.log(f"Şerit takibi aktif: {LINE_TOPIC}")

        # CAN okuyucu thread
        self.can_thread = threading.Thread(target=self._can_listener)
        self.can_thread.daemon = True
        self.can_thread.start()

        # Hız yayıncı thread (0x301 CAN mesajı)
        self.speed_pub_thread = threading.Thread(target=self._speed_publisher)
        self.speed_pub_thread.daemon = True
        self.speed_pub_thread.start()

        self.logger.log("=" * 60)
        self.logger.log("  CAN Waypoint Follower Başlatıldı (Karar Entegrasyonlu)")
        self.logger.log(f"  Maksimum Hız: {MAX_SPEED_KMH} km/h")
        self.logger.log(f"  PID Modu: {pid_mode}")
        self.logger.log(f"  Şerit Takip: {'Aktif (ağırlık: ' + str(LINE_WEIGHT) + ')' if self.line_enabled else 'Kapalı'}")
        self.logger.log("  /hedef topic'i dinleniyor - hedef gelene kadar araç bekleyecek")
        self.logger.log("  /karar topic'i dinleniyor - karar entegrasyonu aktif")
        self.logger.log("  [DURUM] Başlatma komutu bekleniyor (CAN ID 0x500)...")
        self.logger.log("=" * 60)

        # Başlangıç sekansı
        self._initialize_vehicle()

    def _initialize_vehicle(self):
        """Araç başlangıç - Vitesi doğrudan FORWARD'a al (keyboard_teleop gibi)"""
        self.logger.log("Araç başlatılıyor...")

        # Odom'un gelmesini bekle
        self.logger.log("Odometri bekleniyor...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.x == 0.0 and self.y == 0.0 and rospy.Time.now() < timeout:
            self._send_can_command(throttle_pct=0, brake_pct=0, steer_deg=0, gear=GEAR_FORWARD)
            time.sleep(0.1)

        self.logger.log(f"Başlangıç pozisyonu: ({self.x:.2f}, {self.y:.2f})")

        # Doğrudan FORWARD viteste başla
        for _ in range(25):  # 0.5 saniye boyunca FORWARD gönder
            self._send_can_command(throttle_pct=0, brake_pct=0, steer_deg=0, gear=GEAR_FORWARD)
            time.sleep(0.02)

        self.logger.log("Araç hazır! Vites: FORWARD")

    # =========================================================================
    # CALLBACK'LER
    # =========================================================================

    def _hedef_callback(self, msg):
        """Hedef tesliminden gelen waypoint (String: 'x,y' veya 'x1,y1;x2,y2')"""
        try:
            raw = msg.data.strip()
            segments = raw.split(';')
            # Birinci hedef
            parts = segments[0].split(',')
            x, y = float(parts[0]), float(parts[1])

            # Stale hedef filtresi: az once tamamlanan hedefe geri donme
            if self.last_completed_target is not None:
                lx, ly = self.last_completed_target
                if abs(x - lx) < 0.5 and abs(y - ly) < 0.5:
                    return  # Bu hedef az once tamamlandi, yoksay

            self.dynamic_target = (x, y)
            # Ikinci hedef (varsa)
            if len(segments) > 1:
                parts2 = segments[1].split(',')
                x2, y2 = float(parts2[0]), float(parts2[1])
                self.next_target = (x2, y2)
            else:
                self.next_target = None
            self.logger.log(f"HEDEF ALINDI: ({x:.2f}, {y:.2f})"
                            + (f" sonraki: ({x2:.2f}, {y2:.2f})" if self.next_target else ""))
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Hedef parse hatası: {msg.data} - {e}")

    def _is_in_turn(self, threshold_deg=15):
        """Aracın aktif virajda olup olmadığını kontrol et"""
        if self.dynamic_target is None:
            return False
        heading_err = abs(self._heading_error(self.dynamic_target[0], self.dynamic_target[1]))
        return heading_err > math.radians(threshold_deg)

    def _karar_callback(self, msg):
        """Karar node'undan gelen durum (String: 'normal'/'slow'/'dur'/'acildurus'/'sag'/'sol')"""
        new_karar = msg.data.strip().lower()
        old_karar = self.karar

        if new_karar == Karar.DUR and old_karar != Karar.DUR:
            self.logger.log("KARAR: DUR")
        elif new_karar == Karar.ACIL_DURUS:
            self.logger.log("KARAR: ACIL DURUS!")
        elif new_karar == Karar.SAG and old_karar != Karar.SAG:
            if self._is_in_turn():
                self.logger.log(f"KARAR: SAG REDDEDILDI - virajda serit degistirme yasak")
                new_karar = Karar.NORMAL  # SAG'ı yoksay, NORMAL olarak devam et
            else:
                self._start_lane_change(-1)  # sağa şerit değiştir
                self.logger.log("KARAR: SAĞ - şerit değiştirme başladı")
        elif new_karar == Karar.SOL and old_karar != Karar.SOL:
            if self._is_in_turn():
                self.logger.log(f"KARAR: SOL REDDEDILDI - virajda serit degistirme yasak")
                new_karar = Karar.NORMAL  # SOL'u yoksay, NORMAL olarak devam et
            else:
                self._start_lane_change(1)  # sola şerit değiştir
                self.logger.log("KARAR: SOL - şerit değiştirme başladı")
        elif new_karar == Karar.SLOW and old_karar != Karar.SLOW:
            self.logger.log(f"KARAR: YAVAŞ - hız limiti {LIMIT_SLOW} km/h")
        elif new_karar == Karar.NORMAL and old_karar != Karar.NORMAL:
            self.lane_change_active = False  # NORMAL gelince şerit değiştirmeyi de sıfırla
            self.logger.log("KARAR: NORMAL")

        self.karar = new_karar

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

    # =========================================================================
    # KARAR YARDIMCI FONKSİYONLARI
    # =========================================================================

    def _get_speed_limit(self):
        """Karar durumuna göre hız limiti döndür (km/h)"""
        if self.karar == Karar.ACIL_DURUS:
            return 0.0
        elif self.karar == Karar.DUR:
            return 0.0
        elif self.karar in (Karar.SLOW, Karar.SAG, Karar.SOL):
            return LIMIT_SLOW
        else:
            return MAX_SPEED_KMH

    def _start_lane_change(self, direction):
        """Şerit değiştirme başlat (direction: -1=sağ, +1=sol)"""
        # Virajda şerit değiştirmeyi engelle
        if self._is_in_turn():
            self.logger.log(f"SERIT DEGISTIRME REDDEDILDI: Virajda")
            return
        # Yüksek yaw rate = aktif dönüş
        if abs(self.yaw_rate) > 0.3:
            self.logger.log(f"SERIT DEGISTIRME REDDEDILDI: Yuksek donus hizi (yaw_rate={math.degrees(self.yaw_rate):.1f}°/s)")
            return
        self.lane_change_active = True
        self.lane_change_start = time.time()
        self.lane_change_dir = direction

    def _get_lane_change_steer(self):
        """Şerit değiştirme aktifse direksiyon açısı override döndür, değilse None"""
        if not self.lane_change_active:
            return None

        elapsed = time.time() - self.lane_change_start
        if elapsed > LANE_CHANGE_DURATION:
            self.lane_change_active = False
            self.logger.log("Şerit değiştirme tamamlandı")
            return None

        # Virajda veya yüksek dönüş hızında şerit değiştirmeyi iptal et
        if self._is_in_turn(threshold_deg=20) or abs(self.yaw_rate) > 0.3:
            self.lane_change_active = False
            self.logger.log(f"SERIT DEGISTIRME IPTAL: Viraj/donus algilandi")
            return None

        return self.lane_change_dir * LANE_CHANGE_STEER

    def _speed_publisher(self):
        """Hız yayını thread'i - 0x301 CAN mesajı ile hızı yayınla (100ms aralık)"""
        while self.is_running and not rospy.is_shutdown():
            try:
                speed_raw = int(self.speed_ms * 3.6 * 100)  # km/h * 100
                data = struct.pack('<H', speed_raw) + bytes(6)
                msg = can.Message(arbitration_id=0x301, data=data, is_extended_id=False)
                self.bus.send(msg)
            except (can.CanError, struct.error):
                pass
            time.sleep(0.1)

    # =========================================================================
    # PID VE SÜRÜŞ YARDIMCI FONKSİYONLARI
    # =========================================================================

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
            # Büyük açı hatası - hızı biraz düşür ama durma (virajda hız lazım)
            self.speed_pid.set_gains(kp=3.0, ki=0.5)
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

    def _can_listener(self):
        """CAN mesajlarını okuyan arka plan thread'i"""
        while self.is_running and not rospy.is_shutdown():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    if msg.arbitration_id == 0x500:
                        # Sistem Komutları (Byte 0: 1=Start)
                        if msg.data[0] == 1 and not self.mission_started:
                            self.mission_started = True
                            self.logger.log(">>> CAN Başlatma komutu alındı (0x500) <<<")

            except Exception:
                pass

    def _send_can_command(self, throttle_pct, brake_pct, steer_deg, gear=GEAR_FORWARD):
        """
        CAN bus üzerinden komut gönder

        Args:
            throttle_pct: Gaz yüzdesi (0-100)
            brake_pct: Fren yüzdesi (0-100)
            steer_deg: Direksiyon açısı (derece, + sol, - sağ)
            gear: Vites (GEAR_FORWARD, GEAR_NEUTRAL)
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
        dx = target_x - self.x
        dy = target_y - self.y
        target_yaw = math.atan2(dy, dx)
        error = target_yaw - self.yaw

        # Normalize (-pi, pi)
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        return error

    def stop(self):
        """Aracı durdur"""
        self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0)

    def park(self):
        """Aracı park et - el freni çek, vitesi N'ye al"""
        self.logger.log("PARK - El freni çekiliyor...")
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
            self.logger.log("El freni ÇEKILDI - Araç park edildi")
        except can.CanError as e:
            rospy.logwarn(f"El freni CAN hatası: {e}")

    def run(self):
        """Ana kontrol döngüsü"""
        rate = rospy.Rate(50)  # 50 Hz

        # Başlatma komutunu bekle
        while not rospy.is_shutdown() and self.is_running and not self.mission_started:
            self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
            time.sleep(0.1)

        self.logger.log("GÖREV BAŞLATILIYOR! /hedef bekleniyor...")

        while not rospy.is_shutdown() and self.is_running:

            # ========== KARAR: ACIL DURUS ==========
            if self.karar == Karar.ACIL_DURUS:
                self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
                rospy.loginfo_throttle(1.0, "[ACIL DURUS] Tam fren, vites N")
                rate.sleep()
                continue

            # ========== KARAR: DUR ==========
            if self.karar == Karar.DUR:
                self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_FORWARD)
                rospy.loginfo_throttle(2.0, "[DUR] Bekleniyor...")
                rate.sleep()
                continue

            # ========== HEDEF KONTROLÜ ==========
            target = self.dynamic_target

            if target is None:
                # Hedef yok - dur ve bekle (hedefsiz ilerleme kazaya yol acar)
                self._send_can_command(
                    throttle_pct=0,
                    brake_pct=50,
                    steer_deg=0,
                    gear=GEAR_FORWARD
                )
                rospy.loginfo_throttle(2.0, f"[BEKLE] Hedef bekleniyor...")
                rate.sleep()
                continue

            target_x, target_y = target

            # Mesafe hesapla
            distance = self._distance_to(target_x, target_y)

            # Hedefe ulaştık mı?
            if distance < ARRIVAL_THRESHOLD:
                self.logger.log(f"HEDEF TAMAMLANDI: ({target_x:.2f}, {target_y:.2f})")
                self.last_completed_target = (target_x, target_y)
                self.pub_gorev.publish("varildi")
                # Sonraki hedef varsa, arkada olmadigini kontrol et
                if self.next_target:
                    nh_err = abs(self._heading_error(self.next_target[0], self.next_target[1]))
                    if nh_err < math.radians(90):
                        self.dynamic_target = self.next_target
                        self.next_target = None
                        self.logger.log(f"SONRAKI HEDEFE GECILDI: ({self.dynamic_target[0]:.2f}, {self.dynamic_target[1]:.2f})")
                    else:
                        self.logger.log(f"SONRAKI HEDEF ARKADA: heading_err={math.degrees(nh_err):.0f}° - bekleniyor")
                        self.pub_gorev.publish("varildi")
                        self.dynamic_target = None
                        self.next_target = None
                else:
                    self.dynamic_target = None
                rate.sleep()
                continue

            # Açı hatasını hesapla
            heading_error = self._heading_error(target_x, target_y)

            # U-dönüşü koruması: Hedef aracın arkasındaysa (>90°), atla
            if abs(heading_error) > math.radians(90):
                self.logger.log(f"HEDEF ATLANDI (arkada): ({target_x:.2f}, {target_y:.2f}) "
                                f"heading_err={math.degrees(heading_error):.0f}° mesafe={distance:.1f}m")
                self.last_completed_target = (target_x, target_y)
                self.pub_gorev.publish("varildi")
                # Sonraki hedef varsa, onu da kontrol et
                if self.next_target:
                    nh_err = abs(self._heading_error(self.next_target[0], self.next_target[1]))
                    if nh_err < math.radians(90):
                        self.dynamic_target = self.next_target
                        self.next_target = None
                        self.logger.log(f"SONRAKI HEDEFE GECILDI: ({self.dynamic_target[0]:.2f}, {self.dynamic_target[1]:.2f})")
                    else:
                        self.logger.log(f"SONRAKI HEDEF DE ARKADA: heading_err={math.degrees(nh_err):.0f}° - atlaniyor")
                        self.pub_gorev.publish("varildi")
                        self.dynamic_target = None
                        self.next_target = None
                else:
                    self.dynamic_target = None
                rate.sleep()
                continue

            # Adaptif PID ayarlarını güncelle
            self._adapt_pid_gains(heading_error, distance)

            # ========== HIZ KONTROLÜ ==========
            # Karar durumuna göre hız limiti
            speed_limit = self._get_speed_limit()

            max_speed = min(speed_limit, MAX_SPEED_KMH)
            base_speed = max_speed

            # Yaklaşırken yavaşla
            if distance < SLOWDOWN_DISTANCE:
                distance_factor = max(0.3, distance / SLOWDOWN_DISTANCE)
                base_speed *= distance_factor

            # Büyük açı hatasında yavaşla ama minimum hızı koru (virajda durma!)
            abs_heading_error = abs(heading_error)
            TURN_MIN_SPEED = 1.5  # km/h - virajda minimum hız (tekerleklerin kuvvet üretmesi için)
            if abs_heading_error > math.radians(20):
                # 20°→%60, 45°→%40, 90°→%25 hız
                heading_factor = max(0.25, 1.0 - (abs_heading_error / math.pi) * 1.5)
                base_speed *= heading_factor

            target_speed_kmh = max(base_speed, TURN_MIN_SPEED)

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
                if current_speed_kmh >= speed_limit:
                    throttle_pct = 0
            else:
                throttle_pct = 0
                brake_pct = min(60, abs(throttle) * 0.5)

            # ========== DİREKSİYON KONTROLÜ ==========
            steer_deg = self.steer_pid.compute(heading_error, measurement=self.yaw)

            # Şerit değiştirme override
            lane_steer = self._get_lane_change_steer()
            if lane_steer is not None:
                steer_deg = lane_steer
            else:
                # Şerit takip düzeltmesi ekle (sadece şerit değiştirme yoksa)
                line_correction = self._get_line_correction()
                steer_deg += line_correction

            # Direksiyon limitlerini uygula
            steer_deg = np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

            # Komutu gönder
            self._send_can_command(
                throttle_pct=throttle_pct,
                brake_pct=brake_pct,
                steer_deg=steer_deg,
                gear=GEAR_FORWARD
            )

            # CSV log
            self.logger.csv(
                self.x, self.y, self.yaw, current_speed_kmh,
                self.karar, target_x, target_y,
                throttle_pct, brake_pct, steer_deg, 'D'
            )

            # Debug çıktısı
            line_str = f"L:{self.line_angle:+.1f}" if self._is_line_data_fresh() else "L:--"
            karar_str = self.karar.upper() if self.karar != Karar.NORMAL else ""
            rospy.loginfo_throttle(0.5,
                f"Hedef ({target_x:.1f},{target_y:.1f}) | "
                f"Mesafe: {distance:.1f}m | "
                f"Hız: {current_speed_kmh:.1f}/{target_speed_kmh:.1f} km/h | "
                f"Dir: {steer_deg:+.1f} | {line_str}"
                + (f" | {karar_str}" if karar_str else "")
            )

            rate.sleep()

        # Temizlik
        self.is_running = False
        self.stop()
        self.bus.shutdown()
        self.logger.log("CAN Waypoint Follower kapatıldı.")


# =============================================================================
# ANA FONKSİYON
# =============================================================================

def main():
    """Ana fonksiyon"""
    global MAX_SPEED_KMH
    import argparse

    parser = argparse.ArgumentParser(description='TALOS CAN Waypoint Follower')
    parser.add_argument('--mode', '-m', type=str, default='normal',
                        choices=['aggressive', 'normal', 'smooth'],
                        help='PID modu (varsayılan: normal)')
    parser.add_argument('--speed', '-s', type=float, default=None,
                        help=f'Maksimum hız km/h (varsayılan: {MAX_SPEED_KMH})')
    parser.add_argument('--no-adaptive', action='store_true',
                        help='Adaptif PID\'yi devre dışı bırak')
    parser.add_argument('--debug', '-d', action='store_true',
                        help='Debug modunu etkinleştir')

    # ROS argümanlarını filtrele
    args, _ = parser.parse_known_args()

    # Global değişkenleri güncelle (sadece komut satırından verilmişse)
    if args.speed is not None:
        MAX_SPEED_KMH = args.speed

    print("\n" + "=" * 60)
    print("  TALOS CAN Waypoint Follower (Karar Entegrasyonlu)")
    print("=" * 60)
    print(f"  PID Modu: {args.mode}")
    print(f"  Adaptif PID: {'Kapalı' if args.no_adaptive else 'Açık'}")
    print(f"  Maksimum Hız: {MAX_SPEED_KMH} km/h")
    print(f"  Debug: {'Açık' if args.debug else 'Kapalı'}")
    print("-" * 60)
    print("  PID Preset Değerleri:")
    speed_p = PIDPresets.get_speed_preset(args.mode)
    steer_p = PIDPresets.get_steer_preset(args.mode)
    print(f"    Hız: kp={speed_p['kp']}, ki={speed_p['ki']}, kd={speed_p['kd']}")
    print(f"    Dir: kp={steer_p['kp']}, ki={steer_p['ki']}, kd={steer_p['kd']}")
    print("-" * 60)
    print("  Hedef kaynağı: /hedef topic (dinamik)")
    print("  Karar kaynağı: /karar topic")
    print("=" * 60)
    print("  Bekleniyor (GUI'den Başlatın)... [Çıkış: Ctrl+C]")
    print("=" * 60)

    try:
        follower = CANWaypointFollower(pid_mode=args.mode)
        follower.adaptive_pid_enabled = not args.no_adaptive
        follower.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu.")


if __name__ == '__main__':
    main()
