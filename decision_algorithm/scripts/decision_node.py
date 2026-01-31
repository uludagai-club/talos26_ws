#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class VehicleState:
    """Araç durumları"""
    IDLE = "idle"
    MOVING_FORWARD = "forward"
    SLOWING_DOWN = "slowing"
    STOPPED = "stopped"
    TURNING_LEFT = "turning_left"
    TURNING_RIGHT = "turning_right"
    EMERGENCY_STOP = "emergency"

class DecisionAlgorithm:
    """Ana karar algoritması sınıfı"""
    
    def __init__(self):
        """Node'u başlat"""
        rospy.init_node('decision_algorithm', anonymous=False)
        rospy.loginfo("="*60)
        rospy.loginfo("TEKNOFEST Karar Algoritması Başlatıldı")
        rospy.loginfo("="*60)
        
        # Parametreler
        self.load_parameters()
        
        # Subscriber'lar (Görüntü İşleme'den veri al)
        rospy.Subscriber('/trafik_levha', String, self.traffic_sign_callback, queue_size=10)
        rospy.Subscriber('/yaya_gecidi', String, self.crosswalk_callback, queue_size=10)
        
        # Publisher (CAN Bus'a komut gönder)
        self.canbus_pub = rospy.Publisher('/control_canbus', String, queue_size=10)
        
        # Durum değişkenleri
        self.current_state = VehicleState.IDLE
        self.current_sign = None
        self.sign_distance = float('inf')
        self.crosswalk_detected = False
        self.crosswalk_distance = float('inf')
        self.crosswalk_wait_start = None
        self.stop_sign_wait_start = None
        self.turn_start_time = None
        
        # Karar döngüsü (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.decision_loop)
        
        rospy.loginfo("✓ Hazır!")
        rospy.loginfo("="*60)
    
    def load_parameters(self):
        """Parametreleri yükle"""
        self.distance_thresholds = {
            'crosswalk_slow_down': 10.0,
            'crosswalk_stop': 3.0,
            'traffic_sign_slow_down': 8.0,
            'traffic_sign_stop': 2.5,
            'traffic_sign_action': 5.0,
            'emergency_stop': 2.0,
        }
        
        self.timing = {
            'crosswalk_wait_time': 5.0,
            'stop_sign_wait_time': 3.0,
            'turn_duration': 2.0,
        }
        
        self.confidence_thresholds = {
            'min_sign_confidence': 0.7,
            'min_crosswalk_confidence': 0.6,
        }
        
        rospy.loginfo(f"Yaya geçidi: {self.distance_thresholds['crosswalk_slow_down']}m yavaşla, {self.distance_thresholds['crosswalk_stop']}m dur")
        rospy.loginfo(f"STOP levhası: {self.distance_thresholds['traffic_sign_slow_down']}m yavaşla, {self.distance_thresholds['traffic_sign_stop']}m dur")
    
    def traffic_sign_callback(self, msg):
        """Trafik levhası mesajı geldiğinde çalışır
        Format: "sign_type,distance,confidence"
        Örnek: "stop,8.5,0.95"
        """
        try:
            data = msg.data.split(',')
            if len(data) >= 3:
                sign_type = data[0].strip()
                distance = float(data[1])
                confidence = float(data[2])
                
                if confidence >= self.confidence_thresholds['min_sign_confidence']:
                    self.current_sign = sign_type
                    self.sign_distance = distance
                    rospy.loginfo(f"📋 Levha: {sign_type} | {distance:.2f}m | {confidence:.2f}")
            elif len(data) == 1 and data[0].strip().lower() == "none":
                self.current_sign = None
                self.sign_distance = float('inf')
        except Exception as e:
            rospy.logerr(f"❌ Levha parse hatası: {e}")
    
    def crosswalk_callback(self, msg):
        """Yaya geçidi mesajı geldiğinde çalışır
        Format: "detected,distance,confidence"
        Örnek: "true,5.2,0.92"
        """
        try:
            data = msg.data.split(',')
            if len(data) >= 3:
                detected = data[0].strip().lower() == "true"
                distance = float(data[1])
                confidence = float(data[2])
                
                if detected and confidence >= self.confidence_thresholds['min_crosswalk_confidence']:
                    self.crosswalk_detected = True
                    self.crosswalk_distance = distance
                    rospy.loginfo(f"🚶 Yaya geçidi: {distance:.2f}m | {confidence:.2f}")
                else:
                    self.crosswalk_detected = False
                    self.crosswalk_distance = float('inf')
        except Exception as e:
            rospy.logerr(f"❌ Yaya geçidi parse hatası: {e}")
    
    def decision_loop(self, event):
        """Ana karar döngüsü"""
        decision = self.make_decision()
        if decision:
            self.send_to_canbus(decision)
    
    def make_decision(self):
        """Karar ver
        Öncelik: 1.Acil Durum > 2.Yaya Geçidi > 3.Trafik Levhaları > 4.Normal Sürüş
        """
        # ÖNCELİK 1: ACİL DURUM
        if self.crosswalk_detected and self.crosswalk_distance < self.distance_thresholds['emergency_stop']:
            if self.current_state != VehicleState.EMERGENCY_STOP:
                rospy.logwarn("🚨 ACİL DURUM!")
                self.current_state = VehicleState.EMERGENCY_STOP
            return "emergency_stop"
        
        # ÖNCELİK 2: YAYA GEÇİDİ
        if self.crosswalk_detected:
            return self.handle_crosswalk()
        
        # ÖNCELİK 3: TRAFİK LEVHALARI
        if self.current_sign and self.sign_distance < self.distance_thresholds['traffic_sign_action']:
            return self.handle_traffic_sign()
        
        # ÖNCELİK 4: NORMAL SÜRÜŞ
        if self.current_state != VehicleState.MOVING_FORWARD:
            self.current_state = VehicleState.MOVING_FORWARD
        return "forward,normal"
    
    def handle_crosswalk(self):
        """Yaya geçidi davranışı"""
        distance = self.crosswalk_distance
        
        # > 10m: Normal sürüş
        if distance > self.distance_thresholds['crosswalk_slow_down']:
            return "forward,normal"
        
        # 10m - 3m: Yavaşla
        elif distance > self.distance_thresholds['crosswalk_stop']:
            if self.current_state != VehicleState.SLOWING_DOWN:
                rospy.loginfo(f"⚠️  Yaya geçidine yaklaşılıyor ({distance:.2f}m)")
                self.current_state = VehicleState.SLOWING_DOWN
            return "forward,slow"
        
        # < 3m: Dur ve bekle
        else:
            if self.current_state != VehicleState.STOPPED:
                rospy.logwarn(f"🛑 Yaya geçidinde duruldu ({distance:.2f}m)")
                self.current_state = VehicleState.STOPPED
                self.crosswalk_wait_start = rospy.Time.now()
                return "stop"
            else:
                wait_time = (rospy.Time.now() - self.crosswalk_wait_start).to_sec()
                if wait_time < self.timing['crosswalk_wait_time']:
                    return "stop"
                else:
                    rospy.loginfo("✓ Bekleme doldu - Devam")
                    self.crosswalk_wait_start = None
                    self.current_state = VehicleState.MOVING_FORWARD
                    return "forward,slow"
    
    def handle_traffic_sign(self):
        """Trafik levhası davranışı"""
        sign = self.current_sign
        distance = self.sign_distance
        
        # STOP LEVHASI
        if sign == "stop":
            # > 8m: Normal
            if distance > self.distance_thresholds['traffic_sign_slow_down']:
                return "forward,normal"
            # 8m - 2.5m: Yavaşla
            elif distance > self.distance_thresholds['traffic_sign_stop']:
                if self.current_state != VehicleState.SLOWING_DOWN:
                    rospy.loginfo(f"⚠️  STOP'a yaklaşılıyor ({distance:.2f}m)")
                    self.current_state = VehicleState.SLOWING_DOWN
                return "forward,slow"
            # < 2.5m: Dur ve bekle
            else:
                if self.current_state != VehicleState.STOPPED:
                    rospy.logwarn(f"🛑 STOP'ta duruldu ({distance:.2f}m)")
                    self.current_state = VehicleState.STOPPED
                    self.stop_sign_wait_start = rospy.Time.now()
                    return "stop"
                else:
                    wait_time = (rospy.Time.now() - self.stop_sign_wait_start).to_sec()
                    if wait_time < self.timing['stop_sign_wait_time']:
                        return "stop"
                    else:
                        rospy.loginfo("✓ STOP bekleme doldu")
                        self.stop_sign_wait_start = None
                        self.current_state = VehicleState.MOVING_FORWARD
                        self.current_sign = None
                        return "forward,normal"
        
        # GO SİNYALİ
        elif sign == "go":
            rospy.loginfo(f"➡️  GO sinyali")
            self.current_state = VehicleState.MOVING_FORWARD
            return "forward,normal"
        
        # SOLA DÖN
        elif sign == "turn_left":
            if self.current_state != VehicleState.TURNING_LEFT:
                rospy.loginfo(f"↰ SOLA DÖN")
                self.current_state = VehicleState.TURNING_LEFT
                self.turn_start_time = rospy.Time.now()
            
            if self.turn_start_time:
                turn_duration = (rospy.Time.now() - self.turn_start_time).to_sec()
                if turn_duration < self.timing['turn_duration']:
                    return "turn_left"
                else:
                    rospy.loginfo("✓ Sola dönüş tamamlandı")
                    self.turn_start_time = None
                    self.current_state = VehicleState.MOVING_FORWARD
                    self.current_sign = None
                    return "forward,normal"
            return "turn_left"
        
        # SAĞA DÖN
        elif sign == "turn_right":
            if self.current_state != VehicleState.TURNING_RIGHT:
                rospy.loginfo(f"↱ SAĞA DÖN")
                self.current_state = VehicleState.TURNING_RIGHT
                self.turn_start_time = rospy.Time.now()
            
            if self.turn_start_time:
                turn_duration = (rospy.Time.now() - self.turn_start_time).to_sec()
                if turn_duration < self.timing['turn_duration']:
                    return "turn_right"
                else:
                    rospy.loginfo("✓ Sağa dönüş tamamlandı")
                    self.turn_start_time = None
                    self.current_state = VehicleState.MOVING_FORWARD
                    self.current_sign = None
                    return "forward,normal"
            return "turn_right"
        
        # BİLİNMEYEN
        else:
            rospy.logwarn(f"⚠️  Bilinmeyen levha: {sign}")
            return "forward,slow"
    
    def send_to_canbus(self, command):
        """CAN Bus'a komut gönder"""
        msg = String()
        msg.data = command
        self.canbus_pub.publish(msg)
    
    def run(self):
        """Çalıştır"""
        rospy.loginfo("🚗 Karar Algoritması çalışıyor...")
        rospy.spin()

if __name__ == '__main__':
    try:
        decision = DecisionAlgorithm()
        decision.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Kapatıldı")
