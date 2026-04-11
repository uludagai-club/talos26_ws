#!/usr/bin/env python3

import rospy
import can
import sys
import os
import math

# TALOS workspace'ini ekle
sys.path.insert(0, os.path.expanduser('~/talos-sim/devel/lib/python3/dist-packages'))

from cart_sim.msg import cart_control
from std_msgs.msg import Header
from can_decoder import CANDecoder

class CANtoTalosCart:
    def __init__(self):
        rospy.init_node('can_to_talos_cart', anonymous=True)
        
        self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        self.cart_pub = rospy.Publisher('/cart', cart_control, queue_size=10)
        
        # Durum Değişkenleri
        self.current_throttle_cmd = 0.0
        self.current_brake_cmd = 0.0 # Yeni: Fren komutu
        self.current_steering = 0.0
        self.current_gear = cart_control.FORWARD
        self.current_handbrake = 0.0  # El freni
        
        # Gerçek Hız (Geri Besleme)
        self.actual_speed_kmh = 0.0
        
        # Rampalama için son çıktı değeri
        self.last_throttle_output = 0.0
        
        # Parametreler
        self.max_steering_angle = 30.0
        
        # --- AYARLAR ---
        self.POWER_LIMIT = 0.1
        self.THROTTLE_RAMP_UP = 0.02
        self.THROTTLE_RAMP_DOWN = 0.05
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("      CAN-to-TALOS-Cart (DOĞAL SÜRÜŞ MODU)")
        rospy.loginfo("=" * 70)
        
    def normalize_steering(self, angle_deg):
        steer = angle_deg / self.max_steering_angle
        return min(1.0, max(-1.0, steer))
    
    def run(self):
        rate = rospy.Rate(50)
        seq = 0
        
        while not rospy.is_shutdown():
            # Tüm mesajları oku
            while True:
                message = self.bus.recv(timeout=0)
                if message is None:
                    break
                
                msg_id = message.arbitration_id
                
                # 1. KOMUT: Gaz Pedalı, Vites, Fren (ID 0x100)
                if msg_id == 0x100:
                    raw_throttle = CANDecoder.decode_speed(message.data)
                    self.current_throttle_cmd = raw_throttle / 100.0
                    self.current_gear = CANDecoder.decode_gear(message.data)
                    self.current_brake_cmd = CANDecoder.decode_brake(message.data) # Fren komutunu oku
                
                # 2. KOMUT: Direksiyon (ID 0x201)
                elif msg_id == 0x201:
                    self.current_steering = CANDecoder.decode_steering(message.data)
                    
                # 3. KOMUT: Park Freni (ID 0x102)
                elif msg_id == 0x102:
                    self.current_handbrake = float(message.data[0])  # 0=serbest, 1=aktif
                    if self.current_handbrake > 0.5:
                        rospy.loginfo("El freni AKTIF")
                    else:
                        rospy.loginfo("El freni SERBEST")

                # 4. GERİ BESLEME: Gerçek Hız (ID 0x301) - TalosStateToCAN'den gelir
                elif msg_id == 0x301:
                    self.actual_speed_kmh = CANDecoder.decode_real_speed(message.data)
            
            # --- KONTROL MANTIĞI ---
            cart_msg = cart_control()
            cart_msg.header = Header()
            cart_msg.header.seq = seq
            cart_msg.header.stamp = rospy.Time.now()
            cart_msg.header.frame_id = "can_bridge"
            
            # 1. Hedef Pedal (Kullanıcının bastığı)
            target_pedal = max(0.0, min(1.0, self.current_throttle_cmd))
            
            # 2. Rampa (Yumuşak Geçiş)
            if target_pedal > self.last_throttle_output:
                self.last_throttle_output += self.THROTTLE_RAMP_UP
                if self.last_throttle_output > target_pedal:
                    self.last_throttle_output = target_pedal
            else:
                self.last_throttle_output -= self.THROTTLE_RAMP_DOWN
                if self.last_throttle_output < target_pedal:
                    self.last_throttle_output = target_pedal
            
            # 3. Güç Limiti Uygula
            final_throttle = self.last_throttle_output * self.POWER_LIMIT
            
            # 4. Fren Uygula (CAN'den gelen frene göre)
            final_brake = self.current_brake_cmd
            
            # Gaza basılıyorsa freni sıfırla, frene basılıyorsa gazı sıfırla
            if final_throttle > 0.01:
                final_brake = 0.0
            elif final_brake > 0.01:
                final_throttle = 0.0

            cart_msg.throttle = final_throttle
            cart_msg.brake = final_brake
            cart_msg.steer = self.normalize_steering(self.current_steering)
            cart_msg.handbrake = self.current_handbrake
            cart_msg.shift_gears = self.current_gear

            # El freni aktifse gazı kapat
            if self.current_handbrake > 0.5:
                cart_msg.throttle = 0.0
            
            self.cart_pub.publish(cart_msg)
            
            # Debug
            if seq % 50 == 0:
                rospy.loginfo(
                    f"[BRIDGE] P:{target_pedal*100:.0f}%->M:{final_throttle*100:.1f}% | "
                    f"F:{final_brake*100:.0f}% | V:{self.current_gear} | S:{self.current_steering:.1f}°"
                )
            
            seq += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = CANtoTalosCart()
        bridge.run()
    except Exception as e:
        rospy.logerr(f"Hata: {e}")