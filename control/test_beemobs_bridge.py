#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_beemobs_bridge.py — beemobs_bridge.py saf mantık regresyonu (ROS'suz).

rospy/can/mesaj paketleri STUB'lanır (test_estop_deadlock.py deseni). Köprünün saf
eşleme fonksiyonları (rospy'a bağlanmadan) test edilir. Çalıştır:
    python3 control/test_beemobs_bridge.py
"""
import os
import sys
import types

# --- ROS / CAN / mesaj tepe-import'larını stub'la (import zamanı rospy'a bağlanmaz) ---
for _name in ("rospy", "can"):
    sys.modules.setdefault(_name, types.ModuleType(_name))
# _durum_ilerlet gibi saf-mantık yolları log da basar; log'ları no-op yap.
for _fn in ("loginfo", "logwarn", "logerr", "logdebug"):
    setattr(sys.modules["rospy"], _fn, lambda *a, **k: None)


def _stub_module(name, attrs):
    m = types.ModuleType(name)
    for a in attrs:
        setattr(m, a, object)
    sys.modules[name] = m


_stub_module("nav_msgs", []); _stub_module("nav_msgs.msg", ["Odometry"])
_stub_module("std_msgs", []); _stub_module("std_msgs.msg", ["Bool", "Float32"])

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import beemobs_bridge as B  # noqa: E402

_fail = 0


def chk(cond, msg):
    global _fail
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def yeni(**params):
    """Varsayılan parametrelerle bir köprü (rospy'suz)."""
    return B.BeemobsBridge(params or None)


# ============================================================================
print("== 1) Vites çevirisi (A2): cart {1:N,2:İLERİ,3:GERİ} -> RC {0,1,2} ==")
g = B.BeemobsBridge.cart_vites_to_rc_gear
chk(g(1) == 0, "cart N(1) -> RC 0 (N)")
chk(g(2) == 1, "cart FORWARD(2) -> RC 1 (D)")
chk(g(3) == 2, "cart REVERSE(3) -> RC 2 (R)")
chk(g(0) == 0, "cart NO_COMMAND(0) -> RC 0 (N, güvenli)")
chk(g(99) == 0, "bilinmeyen -> RC 0 (N, güvenli)")

# ============================================================================
print("\n== 2) Gaz eşleme: pct>0 -> PRESS=0/POS=70; pct=0 -> PRESS=1 ==")
b = yeni()
press, pos = b.gaz_hesapla(50.0, 0.0)
chk(press == 0 and pos == 70, "gaz var, fren yok -> PRESS=0 (güç ver), POSITION=70 (sabit)")
press, pos = b.gaz_hesapla(0.0, 0.0)
chk(press == 1, "gaz yok -> PRESS=1 (serbest/güç yok)")

# gaz_olcekli: throttle_pct ile band 50-250 arasında ölçekle
b2 = yeni(gaz_olcekli=True)
press, pos = b2.gaz_hesapla(100.0, 0.0)
chk(press == 0 and pos == 250, "gaz_olcekli=True, %100 -> POSITION=250 (band üstü)")
press, pos = b2.gaz_hesapla(0.0, 0.0)
chk(press == 1, "gaz_olcekli iken de gaz yok -> PRESS=1")

# ============================================================================
print("\n== 3) A7 KARŞILIKLI DIŞLAMA: gaz+fren asla aynı anda aktif ==")
b = yeni()
press, pos = b.gaz_hesapla(80.0, 40.0)   # gaz VE fren komutu birlikte
per = b.fren_hesapla(40.0)
chk(press == 1, "fren>0 iken PRESS=1 (gaz ASLA basılmaz)")
chk(per == 40, "fren>0 iken PER=fren yüzdesi (40)")
# gaz aktifken fren PER=0
press_g, _ = b.gaz_hesapla(80.0, 0.0)
per_g = b.fren_hesapla(0.0)
chk(press_g == 0 and per_g == 0, "gaz aktif & fren yok -> PRESS=0 ve PER=0")
# hiçbir durumda (PRESS=0 iken PER>0) olmamalı
kotu = False
for thr in (0.0, 10.0, 100.0):
    for brk in (0.0, 5.0, 100.0):
        pr, _ = b.gaz_hesapla(thr, brk)
        pe = b.fren_hesapla(brk)
        if pr == 0 and pe > 0:
            kotu = True
chk(not kotu, "hiçbir kombinasyonda 'güç ver (PRESS=0) + fren (PER>0)' oluşmaz")

# ============================================================================
print("\n== 4) Direksiyon PWM (A3): sol/sağ bant, deadband, ters, clamp ==")
b = yeni()  # kp=12, dur_pwm=0, deadband=0.02, fb_isaret=1, fb_olcek=1
pwm_sol = b.direksiyon_pwm(30.0, 0.0)     # büyük SOL hata
chk(1 <= pwm_sol <= 127, f"hedef +30° -> SOL bant 1-127 (pwm={pwm_sol})")
chk(pwm_sol == 127, "büyük SOL hata -> effort clamp 1.0 -> pwm=127 (en hızlı sol)")
pwm_sag = b.direksiyon_pwm(-30.0, 0.0)    # büyük SAĞ hata
chk(128 <= pwm_sag <= 255, f"hedef -30° -> SAĞ bant 128-255 (pwm={pwm_sag})")
chk(pwm_sag == 255, "büyük SAĞ hata -> pwm=255 (en hızlı sağ)")
pwm_dur = b.direksiyon_pwm(0.0, 0.0)      # hata 0
chk(pwm_dur == 0, "hata≈0 (deadband) -> dur_pwm=0")
pwm_fb = b.direksiyon_pwm(10.0, 10.0)     # hedef=fb -> hata 0
chk(pwm_fb == 0, "FeedbackSteeringAngle hedefe eşit -> dur (kapalı-döngü oturdu)")
# küçük SOL hata -> hâlâ sol bantta, 1-127
pwm_kucuk = b.direksiyon_pwm(1.0, 0.0)
chk(1 <= pwm_kucuk <= 127, f"küçük SOL hata -> sol bant (pwm={pwm_kucuk})")
# ters bayrağı yön çevirir
bt = yeni(steer_pwm_ters=True)
pwm_ters = bt.direksiyon_pwm(30.0, 0.0)   # normalde SOL, ters ile SAĞ
chk(128 <= pwm_ters <= 255, f"ters=True -> SOL komut SAĞ banta döner (pwm={pwm_ters})")
# fb işareti/ölçeği hatayı etkiler
bf = yeni(steer_fb_isaret=1.0, steer_fb_olcek=1.0)
chk(bf.direksiyon_pwm(0.0, 20.0) >= 128, "fb=+20 (araç solda) -> SAĞ düzeltme (pwm>=128)")

# ============================================================================
print("\n== 5) Watchdog (A9): >0.5 fren %60 ; >2.0 EMERGENCY ; komut gelince sıfır ==")
b = yeni()
md, per, emg = b.watchdog_durum(0.1)
chk(md is False and emg is False, "0.1 s -> müdahale yok, EMERGENCY yok")
md, per, emg = b.watchdog_durum(0.6)
chk(md is True and per == 60 and emg is False, ">0.5 s -> fren PER=60, EMERGENCY yok")
md, per, emg = b.watchdog_durum(2.5)
chk(md is True and per == 60 and emg is True, ">2.0 s -> fren PER=60, EMERGENCY=1")
# hesapla_cikis üzerinden uçtan uca (DRIVE durumunda)
b.durum = B.BeemobsBridge.DRIVE
b.son_cmd_t = 100.0
c = b.hesapla_cikis(100.6)
chk(c["mod"] == "WATCHDOG" and c["press"] == 1 and c["brake_per"] == 60 and c["emergency"] == 0,
    "hesapla_cikis: 0.6 s bayat -> WATCHDOG (PRESS=1, PER=60, EMERGENCY=0)")
c = b.hesapla_cikis(102.5)
chk(c["emergency"] == 1, "hesapla_cikis: 2.5 s bayat -> EMERGENCY=1")
# komut gelince sıfırlanır
b.can_komut_geldi(103.0)
c = b.hesapla_cikis(103.05)
chk(c["mod"] == "DRIVE", "CAN komutu gelince watchdog sıfırlanır -> DRIVE")

# İncele regresyonu (2026-07-04): HİÇ CAN komutu gelmeden kontak açılıp DRIVE'a
# geçilirse watchdog ENABLE-giriş anından silahlanmalı (eskiden _gecen_sn=0.0
# dönüyordu -> el freni inik + fren %0 ile frensiz kalınıyordu).
b2 = yeni()
b2.durum = B.BeemobsBridge.IGNITION_WAIT
b2.fb_ignition = 1
b2._durum_ilerlet(200.0)                      # -> ENABLE (_enable_giris_t=200.0)
b2._durum_ilerlet(200.1); b2._durum_ilerlet(200.2); b2._durum_ilerlet(200.3)  # -> DRIVE
chk(b2.durum == B.BeemobsBridge.DRIVE and b2.son_cmd_t is None,
    "hiç komut yok ama DRIVE'a geçildi (senaryo kuruldu)")
c = b2.hesapla_cikis(200.4)
chk(c["mod"] == "DRIVE", "ilk 0.5 s: watchdog henüz tetiklenmez (yanlış alarm yok)")
c = b2.hesapla_cikis(200.9)
chk(c["mod"] == "WATCHDOG" and c["press"] == 1 and c["brake_per"] == 60,
    "komutsuz 0.9 s -> WATCHDOG (frensiz DRIVE'da kalınmaz)")
c = b2.hesapla_cikis(202.5)
chk(c["emergency"] == 1, "komutsuz 2.5 s -> EMERGENCY=1")
# referans hiç yoksa fail-safe: bayat say
b3 = yeni()
b3.durum = B.BeemobsBridge.DRIVE            # (normalde olmaz: sekans atlanmış)
chk(b3._gecen_sn(0.0) == float("inf"), "referanssız _gecen_sn -> inf (fail-safe)")
chk(b3.hesapla_cikis(0.0)["mod"] == "WATCHDOG", "referanssız DRIVE -> WATCHDOG")

# ============================================================================
print("\n== 6) E-stop (A8): FB_EMERGENCY=1 -> güvenli; 0 -> sekans yeniden ==")
b = yeni()
b.durum = B.BeemobsBridge.DRIVE
b.fb_emergency = 1
chk(b.estop_aktif() is True, "FB_EMERGENCY=1 -> estop_aktif")
c = b.hesapla_cikis(0.0)
chk(c["mod"] == "ESTOP" and c["press"] == 1 and c["brake_per"] == 60 and c["pwm"] == 0,
    "e-stop çıkışları güvenli (PRESS=1, fren %60, direksiyon dur)")
aktif = b.estop_kontrol(0.0)
chk(aktif is True and b.durum == B.BeemobsBridge.ENABLE, "estop latch -> durum ENABLE'a düşer")
# temizle
b.fb_emergency = 0
aktif = b.estop_kontrol(1.0)
chk(aktif is False and b._estop_latch is False and b.durum == B.BeemobsBridge.ENABLE,
    "FB_EMERGENCY=0 -> latch bırakılır, sekans ENABLE'dan yeniden kurulur")
# FB_VehicleStatus acil değeri (parametreli) de e-stop tetikler
bv = yeni(estop_vehiclestatus=3)
bv.fb_vehiclestatus = 3
chk(bv.estop_aktif() is True, "FB_VehicleStatus == acil değeri -> estop_aktif")

# ============================================================================
print("\n== 7) _stamp(): Header'lı şemada damgalar, Header'sızda dokunmaz ==")
b = yeni()
NOW = 12345


class MsgWithHeader:
    __slots__ = ("header", "x")

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.x = 0


class MsgNoHeader:
    __slots__ = ("a", "b")

    def __init__(self):
        self.a = 0
        self.b = 0


m1 = MsgWithHeader()
b._stamp(m1, NOW)
chk(m1.header.stamp == NOW, "Header'lı mesaj -> header.stamp yazıldı")
m2 = MsgNoHeader()
b._stamp(m2, NOW)  # patlamamalı
chk(not hasattr(m2, "header"), "Header'sız mesaj -> dokunulmadı (Header eklenmedi/patlamadı)")

# ============================================================================
print("\n== 8) Kapanış sekansı: ters sıra + RC_Ignition=0 + PRESS=1 + fren %60 ==")
b = yeni()
plan = b.kapanis_plani()
adlar = [a["ad"] for a in plan]
chk(adlar == ["fren", "direksiyon", "kontak"],
    "kapanış planı enable'ın TERSİ sırada (fren -> direksiyon -> kontak)")
chk(plan[0]["brake_per"] == 60, "ilk adım: fren %60")
son = plan[-1]
chk(son["ad"] == "kontak" and son["ignition"] == 0 and son["press"] == 1,
    "son adım: kontak KES (RC_Ignition=0, PRESS=1)")

# ============================================================================
print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
    sys.exit(0)
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    sys.exit(1)
