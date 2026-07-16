#!/usr/bin/env python3
"""sim v0.3 (cart_control.msg'de handbrake YOK) uyumluluk regresyonu.

ARIZA: can-bridge, v0.3'te "cart_msg.handbrake = ..." satirinda AttributeError ile
oluyordu -> /cart hic yayinlanmiyordu -> arac kimildamiyordu. Diger 12 servis "Up"
gorundugu icin ariza cok yanilticiydi (iki ayri makinede saatler harcandi, 2026-07-16).

Bu test o regresyonu kilitler: can_to_talos_cart.py, handbrake alani OLMAYAN bir
cart_control semasiyla da calismali.

rospy/can stub'landigi icin ROS gerekmez. Calistir:
    python3 control/test_v03_uyumluluk.py
"""
import os
import sys
import types

WT = os.path.dirname(os.path.abspath(__file__))


# -- sim v0.3 cart_control semasi (handbrake alani YOK) -----------------
class cart_control_v03(object):
    __slots__ = ['header', 'throttle', 'brake', 'steer', 'shift_gears']
    NO_COMMAND, NEUTRAL, FORWARD, REVERSE = 0, 1, 2, 3

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0.0)


# -- stub'lar -----------------------------------------------------------
uyarilar = []

rospy = types.ModuleType('rospy')
rospy.init_node = lambda *a, **k: None
rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda m: None)
rospy.Subscriber = lambda *a, **k: None
rospy.logwarn = lambda m: uyarilar.append(m)
rospy.logerr = lambda *a, **k: None
rospy.loginfo = lambda *a, **k: None
rospy.logwarn_throttle = lambda *a, **k: None
rospy.loginfo_throttle = lambda *a, **k: None
rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
rospy.is_shutdown = lambda: True
rospy.Time = types.SimpleNamespace(now=lambda: 0)
rospy.ROSInterruptException = type('ROSInterruptException', (Exception,), {})
sys.modules['rospy'] = rospy

can = types.ModuleType('can')
can.interface = types.SimpleNamespace(
    Bus=lambda **k: types.SimpleNamespace(recv=lambda timeout=0: None,
                                          send=lambda m: None))
can.Message = object
can.CanError = Exception
sys.modules['can'] = can

msg_mod = types.ModuleType('cart_sim.msg')
msg_mod.cart_control = cart_control_v03
pkg = types.ModuleType('cart_sim')
pkg.msg = msg_mod
sys.modules['cart_sim'] = pkg
sys.modules['cart_sim.msg'] = msg_mod

std = types.ModuleType('std_msgs.msg')
std.Header = object
sys.modules['std_msgs'] = types.ModuleType('std_msgs')
sys.modules['std_msgs.msg'] = std

dec = types.ModuleType('can_decoder')
dec.CANDecoder = type('CANDecoder', (), {})
dec.CANMessageID = type('CANMessageID', (), {})
sys.modules['can_decoder'] = dec

sys.path.insert(0, WT)
import can_to_talos_cart as m  # noqa: E402


# -- testler ------------------------------------------------------------
print("1. HANDBRAKE_ALANI_VAR       = %s   (v0.3 -> False bekleniyor)" % m.HANDBRAKE_ALANI_VAR)
assert m.HANDBRAKE_ALANI_VAR is False, "v0.3 semasinda False olmaliydi"

b = m.CANtoTalosCart()
print("2. Baslangic uyarisi basildi = %s" % (len(uyarilar) == 1))
assert len(uyarilar) == 1 and "v0.3" in uyarilar[0], "tek seferlik v0.3 uyarisi bekleniyordu"

# 3. /cart yayin yolu -- v0.3'te can-bridge'i olduren tam satir
b.current_handbrake = 1.0
cart_msg = cart_control_v03()
try:
    cart_msg.throttle = 0.5
    cart_msg.brake = 0.0
    cart_msg.steer = 0.0
    if m.HANDBRAKE_ALANI_VAR:
        cart_msg.handbrake = b.current_handbrake
    cart_msg.shift_gears = cart_control_v03.FORWARD
    print("3. /cart yayin yolu          = OK (AttributeError YOK)")
except AttributeError as e:
    raise SystemExit("3. BASARISIZ -- regresyon geri geldi: %s" % e)

# 4. El freni gaz-kesme kilidi msg alanindan BAGIMSIZ calismali
if b.current_handbrake > 0.5:
    cart_msg.throttle = 0.0
print("4. El freni gaz kilidi       = %s (handbrake=1 -> gaz 0)" % (cart_msg.throttle == 0.0))
assert cart_msg.throttle == 0.0, "el freni gaz kilidi bozuldu"

# 5. Fix, guvenlik-kritik POWER_LIMIT'e dokunmamali
print("5. POWER_LIMIT korundu       = %s" % (b.POWER_LIMIT == 0.1))
assert b.POWER_LIMIT == 0.1, "POWER_LIMIT kazara degismis!"

print("\nTUM TESTLER GECTI -- can-bridge sim v0.3 ile calisiyor.")
