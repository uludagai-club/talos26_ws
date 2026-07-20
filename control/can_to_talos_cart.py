#!/usr/bin/env python3

import rospy
import can
import sys
import os
import math

# TALOS workspace'ini ekle
sys.path.insert(0, os.path.expanduser('~/talos-sim/devel/lib/python3/dist-packages'))

try:
    from cart_sim.msg import cart_control
except ImportError as _e:
    sys.stderr.write(
        "\n[can_bridge] KRITIK: cart_sim.msg import edilemedi (%s).\n"
        "  ~/talos-sim derlenmemis veya bayat. Cozum:\n"
        "      cd ~/talos-sim && catkin_make\n"
        "      cd ~/talos-sim/scripts/talos26_ws && docker compose restart can-bridge\n"
        "  Bu kopru olmadan /cart yayinlanmaz; arac HIC hareket etmez.\n\n" % _e)
    sys.exit(1)

from std_msgs.msg import Header
from can_decoder import CANDecoder

# Sim v0.3'un cart_control.msg'inde 'handbrake' alani YOK; sonraki surumlerde var.
# Alan yoksa ona yazmak AttributeError -> kopru olur -> arac hic kimildamaz.
# Bu yuzden bir kez yoklayip bayraga aliyoruz (asagida :handbrake yazimi bunu kontrol eder).
HANDBRAKE_ALANI_VAR = hasattr(cart_control(), 'handbrake')

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER — hepsi burada
#   (canlı: config/canli_params.yaml 'can_bridge:' — restart'sız uygulanır)
#   Öncelik: kod varsayılanı < TALOS_* env (başlangıç) < canli_params.yaml (canlı)
# ════════════════════════════════════════════════════════════════════════
# Bee1 maks bisiklet açısı (ackermann.py max_bicycle_angle); urdf max_steer=0.5053 rad ile 1:1
MAX_STEERING_ANGLE_DEG = 28.95
# Gaz gücü ölçekleri. urdf motor torku artık gerçekçi (tam gaz ≈2.5 m/s², Bee1);
# yapay 0.1 tavanı kalktı, env/canlı-param ile hâlâ kısılabilir.
POWER_LIMIT        = float(os.environ.get('TALOS_POWER_LIMIT', '1.0'))       # manuel mod (0-1)
POWER_LIMIT_AUTO   = float(os.environ.get('TALOS_POWER_LIMIT_AUTO', '1.0'))  # otonom devirde (0x500=1) (0-1)
THROTTLE_RAMP_UP   = float(os.environ.get('TALOS_RAMP_UP', '0.02'))          # tick başına gaz artışı
THROTTLE_RAMP_DOWN = float(os.environ.get('TALOS_RAMP_DOWN', '0.05'))        # tick başına gaz azalışı
CAN_SEND_RATE_HZ   = 50   # (RESTART) rospy.Rate başlangıçta kurulur

try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle(
        'can_bridge', globals(),
        sinirlar={
            'POWER_LIMIT':            (0.0, 1.0),
            'POWER_LIMIT_AUTO':       (0.0, 1.0),
            'THROTTLE_RAMP_UP':       (0.001, 0.5),
            'THROTTLE_RAMP_DOWN':     (0.001, 0.5),
            'MAX_STEERING_ANGLE_DEG': (5.0, 45.0),
        })
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[can_bridge] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class CANtoTalosCart:
    def __init__(self):
        rospy.init_node('can_to_talos_cart', anonymous=True)
        
        self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        self.cart_pub = rospy.Publisher('/cart', cart_control, queue_size=10)

        if not HANDBRAKE_ALANI_VAR:
            rospy.logwarn(
                "[can_bridge] sim v0.3 tespit edildi: cart_control.msg'de 'handbrake' alani yok "
                "-> CAN 0x305 park-fren GERI-BILDIRIMI KAPALI. Surus etkilenmez "
                "(gaz kesme kilidi 0x102'den calismaya devam eder). Geri-bildirim gerekiyorsa "
                "cart_control.msg'ye 'float64 handbrake' ekleyip catkin_make calistirin.")

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

        # Parametreler üst blokta (AYARLANABİLİR PARAMETRELER — env + canlı override)
        # Direksiyon setinden buton 3 (0x500) ile değişir: False=manuel, True=otonom.
        self.autonomous_mode = False
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("      CAN-to-TALOS-Cart (DOĞAL SÜRÜŞ MODU)")
        rospy.loginfo("=" * 70)
        
    def normalize_steering(self, angle_deg):
        steer = angle_deg / MAX_STEERING_ANGLE_DEG
        return min(1.0, max(-1.0, steer))

    def run(self):
        rate = rospy.Rate(CAN_SEND_RATE_HZ)
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

                # 5. MOD: Otonom devir (ID 0x500) - direksiyon setinden buton 3
                # 1=otonom devraldı -> gaz ölçeğini POWER_LIMIT_AUTO'ya kıs
                # 0=manuel devraldı  -> POWER_LIMIT'e (manuel gaz) dön
                elif msg_id == 0x500:
                    if len(message.data) < 1:
                        rospy.logwarn("[BRIDGE] 0x500: bozuk CAN frame (DLC=0) - yok sayildi")
                    else:
                        new_mode = (message.data[0] == 1)
                        if new_mode != self.autonomous_mode:
                            self.autonomous_mode = new_mode
                            rospy.loginfo(
                                f"[BRIDGE] Mod: {'OTONOM' if new_mode else 'MANUEL'} -> "
                                f"guc limiti {POWER_LIMIT_AUTO if new_mode else POWER_LIMIT}")
            
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
                self.last_throttle_output += THROTTLE_RAMP_UP
                if self.last_throttle_output > target_pedal:
                    self.last_throttle_output = target_pedal
            else:
                self.last_throttle_output -= THROTTLE_RAMP_DOWN
                if self.last_throttle_output < target_pedal:
                    self.last_throttle_output = target_pedal

            # 3. Güç Limiti Uygula (otonom modda güvenli/otonom-ayarlı limit)
            effective_limit = POWER_LIMIT_AUTO if self.autonomous_mode else POWER_LIMIT
            final_throttle = self.last_throttle_output * effective_limit
            
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
            if HANDBRAKE_ALANI_VAR:
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
    except rospy.ROSInterruptException:
        pass          # temiz kapanis (Ctrl+C / docker stop) — restart tetiklenmemeli
    except Exception as e:
        # sys.exit(1) SART: exit 0 ile bitersek docker'in `restart: on-failure`'i
        # tetiklenmez ve baslat.sh gozcusu RestartCount=0 gordugu icin uyarmaz —
        # kopru "temiz cikmis" gibi sessizce olur, arac sebepsiz durur.
        rospy.logerr(f"[can_bridge] KRITIK ({type(e).__name__}): {e} — kopru duruyor, /cart kesiliyor.")
        sys.exit(1)