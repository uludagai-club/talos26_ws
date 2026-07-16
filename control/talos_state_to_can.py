#!/usr/bin/env python3

import rospy
import can
import struct
import math
import sys
import os

# ROS Mesaj Tipleri
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, BatteryState
from std_msgs.msg import Bool

# cart_sim mesaj tipi
try:
    from cart_sim.msg import cart_control
    CART_MSG_AVAILABLE = True
except ImportError:
    CART_MSG_AVAILABLE = False

# CAN Mesaj ID'leri
try:
    from can_decoder import CANMessageID, CANDecoder
except ImportError:
    # Fallback
    class CANMessageID:
        SPEED_RPM = 0x301
        IMU_ACCEL = 0x302
        BATTERY_STATUS = 0x303
        ERROR_CODES = 0x304
        PARK_BRAKE_STATUS = 0x305


class TalosStateToCAN:
    def __init__(self):
        rospy.init_node('talos_state_to_can', anonymous=True)

        # CAN Bağlantısı
        try:
            self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        except OSError:
            rospy.logerr("vcan0 bulunamadı!")
            sys.exit(1)

        # Araç Durumu
        self.actual_speed_kmh = 0.0
        self.steering_angle_deg = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        # Batarya (A13 — ekip kararı: zamana bağlı uydurma deşarj modeli
        # kaldırıldı). /battery_state'ten gerçek veri gelirse battery_callback
        # bunları günceller; hiç gelmezse SABİT nominal Bee1 değerlerinde kalır
        # (FB_OMUX_to_AUTONOMOUS.FB_BatteryVoltage/FB_BatterySOC semantiği).
        self.battery_soc = 95.0        # % - nominal (gerçek veri yoksa)
        self.battery_voltage = 72.0    # V - nominal Bee1 (gerçek veri yoksa)
        self.battery_current = 0.0     # A - nominal (gerçek veri yoksa)
        self.battery_temperature = 25  # °C - nominal (gerçek veri yoksa)

        # Park Freni
        self.park_brake_active = False

        # Hata Durumu
        self.error_count = 0
        self.error_level = 0  # 0=Yok, 1=Uyarı, 2=Hata, 3=Kritik
        self.main_error_code = 0
        self.sub_error_code = 0
        self.system_status = 0x01  # Bit 0: Sistem çalışıyor

        # Subscriber'lar
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # /battery_state abone ol — geldiyse battery_callback gerçek değerleri
        # günceller; hiç mesaj gelmezse __init__'te set edilen SABİT nominal
        # değerler (72V/%95) kalır. (A13: eski try/except gizli bug'ıydı —
        # rospy.Subscriber() bir yayıncı olmasa bile ASLA istisna atmaz, yani
        # self.battery_sim_enabled = False HER ZAMAN set ediliyordu; o değişken
        # ve zamana bağlı uydurma deşarj modeliyle birlikte kaldırıldı.)
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        rospy.loginfo("  Batarya: gerçek /battery_state verisi veya sabit nominal (72V/%95)")

        # El freni durumunu /cart topic'inden al
        if CART_MSG_AVAILABLE:
            rospy.Subscriber('/cart', cart_control, self.cart_callback)
            rospy.loginfo("  El freni durumu /cart topic'inden alınıyor")

        # Periyodik yayıncı (20Hz)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.send_can_messages)

        rospy.loginfo("=" * 60)
        rospy.loginfo("  TALOS State -> CAN Köprüsü (Genişletilmiş)")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"  [ID 0x{CANMessageID.SPEED_RPM:03X}] Hız ve RPM")
        rospy.loginfo(f"  [ID 0x{CANMessageID.IMU_ACCEL:03X}] IMU (İvme)")
        rospy.loginfo(f"  [ID 0x{CANMessageID.BATTERY_STATUS:03X}] Batarya Durumu")
        rospy.loginfo(f"  [ID 0x{CANMessageID.ERROR_CODES:03X}] Hata Kodları")
        rospy.loginfo(f"  [ID 0x{CANMessageID.PARK_BRAKE_STATUS:03X}] Park Freni")
        rospy.loginfo("=" * 60)

    def odom_callback(self, msg):
        # Lineer hız vektörünün büyüklüğü
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v_ms = math.sqrt(vx**2 + vy**2)

        self.actual_speed_kmh = v_ms * 3.6
        # (A13: hıza bağlı uydurma akım-çekme modeli kaldırıldı — battery_current
        # artık ya gerçek /battery_state'ten (battery_callback) ya da SABİT
        # nominal 0.0 A'de kalır.)

    def imu_callback(self, msg):
        self.accel_x = msg.linear_acceleration.x
        self.accel_y = msg.linear_acceleration.y
        self.accel_z = msg.linear_acceleration.z

    def battery_callback(self, msg):
        """Gerçek batarya verisi (varsa)"""
        self.battery_soc = msg.percentage * 100.0
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        # Sıcaklık varsa
        if hasattr(msg, 'temperature') and msg.temperature != 0:
            self.battery_temperature = int(msg.temperature)

    def cart_callback(self, msg):
        """Gazebo'dan araç kontrol durumunu al"""
        # El freni durumu (0.5'ten büyükse aktif).
        # getattr: sim v0.3'un cart_control.msg'inde 'handbrake' alani YOK. Duz erisim
        # her karede AttributeError atardi — rospy callback istisnasini yutar, yani node
        # olmez ama 20 Hz log spam'i akar. Alan yoksa "park freni serbest" varsayiyoruz
        # (can_bridge de o surumde 0x305 geri-bildirimini zaten yaymiyor).
        self.park_brake_active = getattr(msg, 'handbrake', 0.0) > 0.5

    def send_can_messages(self, event):
        # --- MESAJ 1: Araç Durumu (ID: 0x301) ---
        # Byte 0-1: Gerçek Hız (km/h * 100) — decode_real_speed (can_decoder.py,
        #           ×0.01) ile eşleşir (B4)
        # Byte 2-3: Motor Devri — Bee1 VehicleRPM TEKER devridir (A13, dokümana
        #           göre düzeltme; teker yarıçapı 0.2575 m, golf.urdf)

        speed_raw = max(0, min(65535, int(abs(self.actual_speed_kmh) * 100)))

        # Teker devri: rpm = v_ms / (2*pi*teker_yaricapi) * 60
        v_ms = abs(self.actual_speed_kmh) / 3.6
        WHEEL_RADIUS_M = 0.2575  # m - golf.urdf
        rpm = int(v_ms / (2 * math.pi * WHEEL_RADIUS_M) * 60)
        if rpm > 6000:
            rpm = 6000

        data_301 = speed_raw.to_bytes(2, 'little') + \
                   rpm.to_bytes(2, 'little') + \
                   bytes(4)

        msg1 = can.Message(arbitration_id=CANMessageID.SPEED_RPM, data=data_301, is_extended_id=False)

        # --- MESAJ 2: IMU / İvme (ID: 0x302) ---
        # Bee1: IMU Xsens ROS sürücüsünden gelir; sim /imu buna denk (A13)
        # Byte 0-1: X İvmesi (m/s^2 * 100) + Offset
        # Byte 2-3: Y İvmesi
        # Byte 4-5: Z İvmesi

        acc_x_raw = int((self.accel_x + 20) * 100)
        acc_y_raw = int((self.accel_y + 20) * 100)
        acc_z_raw = int((self.accel_z + 20) * 100)

        data_302 = struct.pack('<HHH', acc_x_raw, acc_y_raw, acc_z_raw) + bytes(2)

        msg2 = can.Message(arbitration_id=CANMessageID.IMU_ACCEL, data=data_302, is_extended_id=False)

        # --- MESAJ 3: Batarya Durumu (ID: 0x303) ---
        # Byte 0-1: SoC (% * 10)
        # Byte 2-3: Voltaj (V * 10)
        # Byte 4-5: Akım (A * 10, signed)
        # Byte 6: Sıcaklık (°C + 40)

        soc_raw = int(self.battery_soc * 10)
        voltage_raw = int(self.battery_voltage * 10)
        current_raw = int(self.battery_current * 10)
        temp_raw = self.battery_temperature + 40

        data_303 = struct.pack('<HHhB', soc_raw, voltage_raw, current_raw, temp_raw) + bytes(1)

        msg3 = can.Message(arbitration_id=CANMessageID.BATTERY_STATUS, data=data_303, is_extended_id=False)

        # --- MESAJ 4: Hata Kodları (ID: 0x304) ---
        # Byte 0: Hata sayısı
        # Byte 1: Seviye
        # Byte 2-3: Ana kod
        # Byte 4-5: Alt kod
        # Byte 6: Sistem durumu

        data_304 = bytes([self.error_count, self.error_level])
        data_304 += struct.pack('<HH', self.main_error_code, self.sub_error_code)
        data_304 += bytes([self.system_status, 0])

        msg4 = can.Message(arbitration_id=CANMessageID.ERROR_CODES, data=data_304, is_extended_id=False)

        # --- MESAJ 5: Park Freni (ID: 0x305) ---
        park_state = 1 if self.park_brake_active else 0
        data_305 = bytes([park_state, park_state]) + bytes(6)

        msg5 = can.Message(arbitration_id=CANMessageID.PARK_BRAKE_STATUS, data=data_305, is_extended_id=False)

        try:
            self.bus.send(msg1)
            self.bus.send(msg2)
            self.bus.send(msg3)
            self.bus.send(msg4)
            self.bus.send(msg5)
        except can.CanError:
            pass

if __name__ == '__main__':
    try:
        node = TalosStateToCAN()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
