#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
beemobs_bridge.py — CAN (vcan0)  <->  Beemobs Bee1 /beemobs/* araç arayüzü köprüsü
=================================================================================
DİJİTAL İKİZ önceliği: stack (control.py) HİÇ DEĞİŞMEDEN vcan0'a CAN yazar; bu köprü
CAN komutlarını Bee1 aracının resmî /beemobs/* topic ŞEMASINA çevirir. Böylece AYNI
stack hem GERÇEK araçta hem de sim'de (beemobs_gazebo_emulatoru.py ile) koşar.

VERİ AKIŞI
----------
  control.py ── 0x100 (gaz/fren/vites), 0x102 (el freni), 0x201 (direksiyon) ──► vcan0
      │
      ▼  (bu köprü CAN'i okur -> /beemobs/* komutları yayınlar)
  /beemobs/rc_unittoOmux              (kontak, vites, e-stop)
  /beemobs/RC_THRT_DATA               (gaz: PRESS + POSITION)
  /beemobs/AUTONOMOUS_BrakePedalControl (ayak freni: EN + PER %)
  /beemobs/AUTONOMOUS_SteeringMot_Control (ham PWM sol/sağ hız)
  /beemobs/AUTONOMOUS_HB_MotorControl  (el freni motoru)
  /beemobs/AutonomousHeardBit          (heartbeat, ManuelSelect=1)
      │
      ▼  [GERÇEK ARAÇ  VEYA  beemobs_gazebo_emulatoru.py]
  /beemobs/FB_VehicleSpeed             (gerçek hız — geri besleme)
  /beemobs/FeedbackSteeringAngle       (tekerlek açısı — geri besleme)
  /beemobs/FB_OMUX_to_AUTONOMOUS       (FB_IGNITION / FB_EMERGENCY / durum)
      │
      ▼  (köprü FB'yi okur -> 0x301 CAN + /beemobs_odom yayınlar)
  vcan0 0x301 (hız/RPM)  +  /beemobs_odom (nav_msgs/Odometry)

TASARIM KARARLARI (ekip): band 50-250, sabit gaz 70; direksiyon dur≈0 (127-128 nötr
YANLIŞ, araç başında kalibre); cmd_rate 10 Hz (mesaj önceliklerinin karışmaması için
DÜŞÜK ve sorunsuz); rc_unittoOmux SADECE alan değişiminde + 1 Hz tazeleme (bus'u boğma).

ŞEMA NOTU: /beemobs mesajlarında Header YOK. Yine de savunmacı _stamp() deseni
kullanılır (Header'lı bir şemaya geçilirse otomatik damgalar).

TEST EDİLEBİLİRLİK: rospy/CAN'e SADECE main() -> setup_ros() içinde bağlanılır;
__init__ saf mantık + parametredir. Eşleme fonksiyonları saf (rospy'suz) test edilir
(control/test_beemobs_bridge.py).
"""

import math
import os
import struct
import sys

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

# can_decoder.py bu köprü ile aynı klasörden mount edilir (docker-compose.beemobs.yml).
# DEĞİŞTİRİLMEZ; sadece import edilir.
from can_decoder import CANDecoder

# Direksiyon limiti TEK kaynaktan (ackermann.py, Bee1 teker limitlerinden ≈28.95°).
# İncele düzeltmesi (2026-07-04): eski elle yazılmış 30.0 varsayılanı kaynaktan sapmıştı.
try:
    import ackermann as _ackermann
    _MAX_TEKER_ACI_VARSAYILAN = float(_ackermann.max_bicycle_angle())
except Exception:
    _MAX_TEKER_ACI_VARSAYILAN = 28.95


# ---------------------------------------------------------------------------
# beemobs mesaj sınıfları — smart_can_msgs (gerçek araç) veya cart_sim (sim devel)
# ---------------------------------------------------------------------------
_BEEMOBS_MESAJLARI = (
    "rc_unittoOmux", "RC_THRT_DATA", "AUTONOMOUS_BrakePedalControl",
    "AUTONOMOUS_SteeringMot_Control", "AUTONOMOUS_HB_MotorControl",
    "AutonomousHeardBit", "FB_VehicleSpeed", "FeedbackSteeringAngle",
    "FB_OMUX_to_AUTONOMOUS",
)


def _devel_yollarini_ekle():
    """beemobs (.msg) python sınıflarını sys.path'e ekler: derlenmiş smart_can_msgs
    (araç/CAN imajı) ve bind-mount cart_sim devel (sim). Gerçek araçta workspace
    source'lanmışsa paket zaten PYTHONPATH'tedir."""
    for _p in ("/can_ws/devel/lib/python3/dist-packages",
               "/talos-devel/lib/python3/dist-packages",
               os.path.expanduser("~/talos-sim/devel/lib/python3/dist-packages")):
        if os.path.isdir(_p) and _p not in sys.path:
            sys.path.insert(0, _p)


def yukle_beemobs_mesajlari(paket_tercih="smart_can_msgs"):
    """beemobs mesaj sınıflarını md5/şema-uyumlu paketten yükler: önce tercih (araçta
    'smart_can_msgs'), bulunmazsa 'cart_sim'. sınıf-adı -> sınıf sözlüğü döner."""
    import importlib
    _devel_yollarini_ekle()
    son_hata = None
    paketler = [paket_tercih] + (["cart_sim"] if paket_tercih != "cart_sim" else [])
    for pkg in paketler:
        try:
            mod = importlib.import_module(pkg + ".msg")
            tipler = {ad: getattr(mod, ad) for ad in _BEEMOBS_MESAJLARI}  # yoksa AttributeError
            rospy.loginfo("[BEEMOBS] mesajlar '%s' paketinden yuklendi.", pkg)
            return tipler
        except (ImportError, AttributeError) as e:
            son_hata = e
    raise ImportError(
        "beemobs mesajlari yuklenemedi (denenen: %s). Arac: 'smart_can_msgs', sim: "
        "cart_sim devel agacinda beemobs .msg'leri DERLENMIS olmali. Son hata: %s"
        % (paketler, son_hata))


# ---------------------------------------------------------------------------
# Köprü
# ---------------------------------------------------------------------------
class BeemobsBridge:
    """CAN komutlarını /beemobs/* komutlarına çeviren, FB'den 0x301+odom üreten köprü.

    __init__ SAF (rospy/CAN'e dokunmaz). setup_ros() ROS'a bağlanır; run() döngüdür.
    Eşleme metotları saf -> rospy'suz test edilir.
    """

    # Durum makinesi (A5)
    INIT = "INIT"
    IGNITION_WAIT = "IGNITION_WAIT"
    ENABLE = "ENABLE"
    DRIVE = "DRIVE"

    def __init__(self, params=None):
        p = params or {}

        # --- Parametreler (rosparam ~, env fallback -> main() doldurur) ---
        self.can_interface = p.get("can_interface", os.environ.get("CAN_INTERFACE", "vcan0"))
        self.cmd_rate_hz = float(p.get("cmd_rate_hz", 10.0))       # A14: düşük+sorunsuz
        self.omux_rate_hz = float(p.get("omux_rate_hz", 1.0))      # değişimde anında + 1 Hz tazeleme
        self.heartbeat_rate_hz = float(p.get("heartbeat_rate_hz", 1.0))

        # Gaz (EKİP: band 50-250, standart sabit 70; ölçekleme varsayılan KAPALI)
        self.gaz_pozisyon = int(p.get("gaz_pozisyon", 70))
        self.gaz_olcekli = bool(p.get("gaz_olcekli", False))
        self.gaz_band_min = int(p.get("gaz_band_min", 50))
        self.gaz_band_max = int(p.get("gaz_band_max", 250))

        # Direksiyon (KD: dur≈0; "127-128 nötr" YANLIŞ)
        self.direksiyon_dur_pwm = int(p.get("direksiyon_dur_pwm", 0))
        self.steer_pwm_kp = float(p.get("steer_pwm_kp", 12.0))     # slalom varsayılanı
        self.steer_pwm_ters = bool(p.get("steer_pwm_ters", False))
        self.steer_fb_isaret = float(p.get("steer_fb_isaret", 1.0))
        self.steer_fb_olcek = float(p.get("steer_fb_olcek", 1.0))
        self.steer_deadband = float(p.get("steer_deadband", 0.02))  # |effort|<=bu -> dur

        # Güvenlik / geri besleme
        self.watchdog_sn = float(p.get("watchdog_sn", 0.5))
        self.watchdog_acil_sn = float(p.get("watchdog_acil_sn", 2.0))
        self.fren_acc = int(p.get("fren_acc", 10000))
        self.watchdog_fren_per = int(p.get("watchdog_fren_per", 60))   # A9: komut kesilince %60
        self.estop_fren_per = int(p.get("estop_fren_per", 60))         # A8: e-stop fren %60
        self.kapanis_fren_per = int(p.get("kapanis_fren_per", 60))     # kapanışta fren %60
        self.odom_topic = p.get("odom_topic", "/beemobs_odom")
        self.hb_pwm = int(p.get("hb_pwm", 100))
        # FB_VehicleStatus'un acil değeri bilinmiyor -> negatif=devre dışı (RİSK, sahada kalibre).
        self.estop_vehiclestatus = int(p.get("estop_vehiclestatus", -1))
        self.max_teker_aci = float(p.get("max_teker_aci", _MAX_TEKER_ACI_VARSAYILAN))

        # --- Durum ---
        self.durum = self.INIT
        self._enable_gecikme = int(p.get("enable_gecikme_tik", 3))  # ENABLE'da kaç tik sonra DRIVE
        self._enable_sayac = 0

        # CAN'den okunan komutlar
        self.throttle_pct = 0.0     # 0-100 (control.py 0x100 encode ölçeği: <H/100)
        self.brake_pct = 0.0        # 0-100
        self.gear = 0               # cart vitesi: 1=N,2=FORWARD,3=REVERSE (0x100 byte 2)
        self.handbrake = 0.0        # 0=serbest, 1=aktif (0x102)
        self.hedef_direksiyon = 0.0  # derece (0x201)

        # Geri besleme (FB) durumu
        self.fb_steer_deg = 0.0
        self.fb_speed_kmh = 0.0
        self.fb_ignition = 0
        self.fb_emergency = 0
        self.fb_vehiclestatus = 0

        # Zamanlama / latch
        self.son_cmd_t = None       # son CAN komutunun zamanı (watchdog)
        self._enable_giris_t = None  # ENABLE'a giriş anı — hiç komut gelmezse watchdog referansı
        self._estop_latch = False
        self._omux_son = None       # rc_unittoOmux son yayınlanan alanları (A14: değişimde yayınla)
        self._omux_son_t = None
        self._hb_son_t = None
        self._son_fb_yayin_t = None  # FB->odom/0x301 yayın hız sınırı (cmd_rate)

        # ROS nesneleri (setup_ros'ta doldurulur)
        self.m = None               # mesaj sınıfı sözlüğü
        self.bus = None
        self._pubs = {}

    # =====================================================================
    # SAF EŞLEME MANTIĞI (rospy'suz test edilir)
    # =====================================================================
    @staticmethod
    def cart_vites_to_rc_gear(cart_gear):
        """A2: cart vitesi {1:N, 2:İLERİ, 3:GERİ} -> RC_SelectionGear {0:N, 1:D, 2:R}.
        Bilinmeyen -> 0 (N, güvenli)."""
        return {1: 0, 2: 1, 3: 2}.get(int(cart_gear), 0)

    def gaz_hesapla(self, throttle_pct, brake_pct):
        """A7 KARŞILIKLI DIŞLAMA (Bee1 'önce gücü kes sonra frenle'):
          - fren varsa VEYA gaz yoksa  -> PRESS=1 (güç yok), POSITION etkisiz.
          - fren yok & gaz varsa        -> PRESS=0 (güç ver), POSITION=gaz_pozisyon (ölçekli ise band).
        (press, position) döndürür. press: 0=güç ver, 1=serbest."""
        guc_ver = (brake_pct <= 0.0 and throttle_pct > 0.0)
        if guc_ver:
            press = 0
            if self.gaz_olcekli:
                oran = max(0.0, min(1.0, throttle_pct / 100.0))
                pos = int(round(self.gaz_band_min + oran * (self.gaz_band_max - self.gaz_band_min)))
            else:
                pos = self.gaz_pozisyon
        else:
            press = 1
            pos = self.gaz_pozisyon  # PRESS=1 iken etkisiz; yine de tutarlı değer gönder
        return press, max(0, min(255, int(pos)))

    @staticmethod
    def fren_hesapla(brake_pct):
        """AUTONOMOUS_BrakePedalMotor_PER (0-100). Fren yoksa 0 (A7: gaz aktifken PER=0)."""
        if brake_pct > 0.0:
            return max(0, min(100, int(round(brake_pct))))
        return 0

    def direksiyon_pwm(self, hedef_aci, fb_aci):
        """A3 kapalı-döngü P: hedef tekerlek açısını FeedbackSteeringAngle ile tutar.
          hata = hedef - (fb * isaret / olcek);  effort = clamp(kp*hata/127, -1, 1)
          effort> deadband -> SOL  band 1-127   = int(round(127*effort))
          effort<-deadband -> SAĞ  band 128-255 = int(round(128+127*(-effort)))
          |effort|<=deadband -> direksiyon_dur_pwm
        ters bayrağı yön haritasını çevirir (sahada kalibre)."""
        hedef_aci = max(-self.max_teker_aci, min(self.max_teker_aci, hedef_aci))
        fb = self.steer_fb_isaret * fb_aci / self.steer_fb_olcek
        hata = hedef_aci - fb                      # + = daha SOLA gerek
        effort = max(-1.0, min(1.0, (self.steer_pwm_kp * hata) / 127.0))
        if self.steer_pwm_ters:
            effort = -effort
        if effort > self.steer_deadband:           # SOL
            return max(1, min(127, int(round(127.0 * effort))))
        if effort < -self.steer_deadband:          # SAĞ
            return max(128, min(255, int(round(128.0 + 127.0 * (-effort)))))
        return self.direksiyon_dur_pwm             # dur/merkez

    @staticmethod
    def hb_hesapla(handbrake):
        """A12 el freni: handbrake>0.5 -> MotState=0 (ÇEK), değilse 1 (İNDİR)."""
        return 0 if handbrake > 0.5 else 1

    def watchdog_durum(self, gecen_sn):
        """A9: komutlar araçta KALICI olduğundan son CAN komutu üzerinden geçen süreye göre:
          >watchdog_acil_sn (2.0) -> (müdahale, PER=60, EMERGENCY=1)
          >watchdog_sn      (0.5) -> (müdahale, PER=60, EMERGENCY=0)
          aksi                     -> (müdahale yok, 0, EMERGENCY=0)
        (mudahale, per, emergency) döndürür."""
        if gecen_sn > self.watchdog_acil_sn:
            return True, self.watchdog_fren_per, True
        if gecen_sn > self.watchdog_sn:
            return True, self.watchdog_fren_per, False
        return False, 0, False

    def estop_aktif(self):
        """A8 e-stop girdisi: FB_EMERGENCY==1 veya FB_VehicleStatus acil değeri."""
        if self.fb_emergency == 1:
            return True
        if self.estop_vehiclestatus >= 0 and self.fb_vehiclestatus == self.estop_vehiclestatus:
            return True
        return False

    def _gecen_sn(self, now):
        """now - son referans (sn). rospy.Time (Duration.to_sec) ve düz float ile çalışır.
        İncele düzeltmesi (2026-07-04): hiç CAN komutu gelmediyse referans ENABLE'a giriş
        anıdır — kontak açık ama control.py hiç yazmıyorsa watchdog yine silahlanır
        (aksi hâlde DRIVE'da el freni inik + fren %0 ile frensiz kalınıyordu).
        Referans hiç yoksa fail-safe: bayat say (inf)."""
        ref = self.son_cmd_t if self.son_cmd_t is not None else self._enable_giris_t
        if ref is None:
            return float("inf")
        d = now - ref
        return d.to_sec() if hasattr(d, "to_sec") else float(d)

    def guvenli_cikis(self, ignition=1, emergency=0, fren_per=None):
        """Güvenli çıkış paketi: gaz kes (PRESS=1), fren bas, direksiyon dur, vites N."""
        return {
            "mod": "GUVENLI",
            "press": 1,
            "position": self.gaz_pozisyon,
            "brake_en": 1,
            "brake_per": self.estop_fren_per if fren_per is None else fren_per,
            "pwm": self.direksiyon_dur_pwm,
            "rc_gear": 0,
            "hb_motstate": 0 if self.fb_speed_kmh < 1.0 else 1,  # (neredeyse) durunca çek
            "emergency": emergency,
            "ignition": ignition,
        }

    def hesapla_cikis(self, now):
        """Bir komut tikinin TÜM çıkış alanlarını (ROS'suz) hesaplar. Güvenlik katmanları
        önceliği: E-STOP > durum(bekle) > watchdog > normal DRIVE."""
        # 1) E-STOP: en yüksek öncelik
        if self.estop_aktif():
            c = self.guvenli_cikis(ignition=1, emergency=0, fren_per=self.estop_fren_per)
            c["mod"] = "ESTOP"
            return c

        # 2) Kontak bekleme / enable öncesi: güvenli tut (fren çekili, gaz yok)
        if self.durum in (self.INIT, self.IGNITION_WAIT):
            c = self.guvenli_cikis(
                ignition=(0 if self.durum == self.INIT else 1),
                emergency=0, fren_per=self.watchdog_fren_per)
            c["mod"] = "BEKLE"
            c["hb_motstate"] = 0  # kontak gelene dek el freni ÇEK (eğimde kaymaz)
            return c

        # 3) Watchdog: son CAN komutu bayatladı
        mudahale, wd_per, wd_emg = self.watchdog_durum(self._gecen_sn(now))
        if mudahale:
            c = self.guvenli_cikis(ignition=1, emergency=(1 if wd_emg else 0), fren_per=wd_per)
            c["mod"] = "WATCHDOG"
            c["rc_gear"] = self.cart_vites_to_rc_gear(self.gear)
            return c

        # 4) Normal DRIVE (ENABLE de aynı komutları üretir; enable EN alanlarıyla sağlanır)
        press, pos = self.gaz_hesapla(self.throttle_pct, self.brake_pct)
        return {
            "mod": "DRIVE",
            "press": press,
            "position": pos,
            "brake_en": 1,
            "brake_per": self.fren_hesapla(self.brake_pct),
            "pwm": self.direksiyon_pwm(self.hedef_direksiyon, self.fb_steer_deg),
            "rc_gear": self.cart_vites_to_rc_gear(self.gear),
            "hb_motstate": self.hb_hesapla(self.handbrake),
            "emergency": 0,
            "ignition": 1,
        }

    def estop_kontrol(self, now):
        """A8 latch: FB_EMERGENCY -> durum ENABLE'a düşer (sekans yeniden kurulur).
        Temizlenince latch bırakılır; durum ENABLE kalır -> ENABLE->DRIVE tekrar işler."""
        aktif = self.estop_aktif()
        if aktif and not self._estop_latch:
            self._estop_latch = True
            self.durum = self.ENABLE
            self._enable_sayac = 0
        elif not aktif and self._estop_latch:
            self._estop_latch = False
            self.durum = self.ENABLE      # sekansı yeniden kur
            self._enable_sayac = 0
        return aktif

    def can_komut_geldi(self, now):
        """CAN komutu (0x100/0x102/0x201) gelince watchdog zamanlayıcısını tazele."""
        self.son_cmd_t = now

    def kapanis_plani(self):
        """rospy.on_shutdown: enable sekansının TERS sırası + RC_Ignition=0 + PRESS=1 + fren %60.
        Sıralı adım listesi döndürür (test: son adım kontak-kapa, sıra enable'ın tersi)."""
        return [
            {"ad": "fren", "brake_en": 1, "brake_per": self.kapanis_fren_per, "press": 1},
            {"ad": "direksiyon", "steer_en": 0, "pwm": self.direksiyon_dur_pwm},
            {"ad": "kontak", "ignition": 0, "press": 1, "hb_motstate": 0},
        ]

    def _stamp(self, msg, now):
        """Savunmacı Header damgası: /beemobs mesajlarında Header YOK -> dokunma; Header'lı
        bir şemaya geçilirse otomatik damgala."""
        if "header" in getattr(msg, "__slots__", ()):
            try:
                msg.header.stamp = now
            except Exception:
                pass
        return msg

    # =====================================================================
    # ROS BAĞLANTISI (SADECE main()'den sonra)
    # =====================================================================
    def setup_ros(self, paket_tercih="smart_can_msgs"):
        import can
        self.m = yukle_beemobs_mesajlari(paket_tercih)

        # Komut publisher'ları (queue_size=1: en yeni komut geçerli)
        self._pubs["omux"] = rospy.Publisher(
            "/beemobs/rc_unittoOmux", self.m["rc_unittoOmux"], queue_size=1)
        self._pubs["thrt"] = rospy.Publisher(
            "/beemobs/RC_THRT_DATA", self.m["RC_THRT_DATA"], queue_size=1)
        self._pubs["brake"] = rospy.Publisher(
            "/beemobs/AUTONOMOUS_BrakePedalControl",
            self.m["AUTONOMOUS_BrakePedalControl"], queue_size=1)
        self._pubs["steer"] = rospy.Publisher(
            "/beemobs/AUTONOMOUS_SteeringMot_Control",
            self.m["AUTONOMOUS_SteeringMot_Control"], queue_size=1)
        self._pubs["hb"] = rospy.Publisher(
            "/beemobs/AUTONOMOUS_HB_MotorControl",
            self.m["AUTONOMOUS_HB_MotorControl"], queue_size=1)
        self._pubs["heart"] = rospy.Publisher(
            "/beemobs/AutonomousHeardBit", self.m["AutonomousHeardBit"], queue_size=1)

        # Geri besleme / debug publisher'ları
        self._pubs["odom"] = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self._pubs["estop"] = rospy.Publisher("/beemobs_estop", Bool, queue_size=1, latch=True)
        self._pubs["steer_dbg"] = rospy.Publisher("/beemobs_direksiyon_deg", Float32, queue_size=1)

        # FB subscriber'ları
        rospy.Subscriber("/beemobs/FB_VehicleSpeed", self.m["FB_VehicleSpeed"],
                         self._speed_cb, queue_size=1)
        rospy.Subscriber("/beemobs/FeedbackSteeringAngle", self.m["FeedbackSteeringAngle"],
                         self._steer_cb, queue_size=1)
        rospy.Subscriber("/beemobs/FB_OMUX_to_AUTONOMOUS", self.m["FB_OMUX_to_AUTONOMOUS"],
                         self._omux_cb, queue_size=1)

        # CAN (komut girişi)
        self.bus = can.interface.Bus(channel=self.can_interface, interface="socketcan")

        rospy.on_shutdown(self.kapanis_sekansi)

        rospy.loginfo("=" * 70)
        rospy.loginfo("  BEEMOBS BRIDGE  (CAN %s <-> /beemobs/*)", self.can_interface)
        rospy.loginfo("  cmd_rate=%.0f Hz | gaz_pozisyon=%d (band %d-%d, olcekli=%s)",
                      self.cmd_rate_hz, self.gaz_pozisyon, self.gaz_band_min,
                      self.gaz_band_max, self.gaz_olcekli)
        rospy.loginfo("  direksiyon: dur_pwm=%d kp=%.1f ters=%s | watchdog %.1f/%.1f s",
                      self.direksiyon_dur_pwm, self.steer_pwm_kp, self.steer_pwm_ters,
                      self.watchdog_sn, self.watchdog_acil_sn)
        rospy.loginfo("=" * 70)

    # ---- FB callback'leri (0x301 + odom + debug üretir) ----
    def _speed_cb(self, msg):
        # Fix B: FB_ReelVehicleSpeed_KMh uint8 (kaba); m/s'yi /3.6 ile TÜRET (Ms alanından
        # değil -> düşük hızda daha iyi çözünürlük). Sonra odom + 0x301 yayınla.
        # İncele düzeltmesi (2026-07-04): yayın cmd_rate ile sınırlandı — araç FB'yi
        # yüksek frekansta basarsa 0x301/odom trafiği bus disiplinini (A14) delmesin.
        self.fb_speed_kmh = float(getattr(msg, "FB_ReelVehicleSpeed_KMh", 0))
        now = rospy.Time.now()
        if self._son_fb_yayin_t is not None:
            d = now - self._son_fb_yayin_t
            gecen = d.to_sec() if hasattr(d, "to_sec") else float(d)
            if gecen < (1.0 / self.cmd_rate_hz):
                return  # durum güncellendi, yayın sıradaki pencerede
        self._son_fb_yayin_t = now

        hiz_ms = max(0.0, self.fb_speed_kmh / 3.6)
        odom = Odometry()
        self._stamp(odom, now)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = hiz_ms
        self._pubs["odom"].publish(odom)

        self._301_yaz(self.fb_speed_kmh)

    def _steer_cb(self, msg):
        # FeedbackSteeringAngle int8 -> dahili P-döngüsü FB'si + debug Float32.
        self.fb_steer_deg = float(getattr(msg, "FeedBackSteeringAngle", 0))
        self._pubs["steer_dbg"].publish(Float32(data=self.fb_steer_deg))

    def _omux_cb(self, msg):
        self.fb_ignition = int(getattr(msg, "FB_IGNITION", 0))
        self.fb_emergency = int(getattr(msg, "FB_EMERGENCY", 0))
        self.fb_vehiclestatus = int(getattr(msg, "FB_VehicleStatus", 0))

    def _301_yaz(self, speed_kmh):
        """0x301 CAN encode — talos_state_to_can ile AYNI format (hız km/h*100, RPM)."""
        import can
        speed_raw = max(0, min(65535, int(round(abs(speed_kmh) * 100))))
        rpm = min(6000, 800 + int(abs(speed_kmh) * 150))
        data = speed_raw.to_bytes(2, "little") + rpm.to_bytes(2, "little") + bytes(4)
        try:
            self.bus.send(can.Message(arbitration_id=0x301, data=data, is_extended_id=False))
        except Exception:
            pass

    # ---- CAN komut girişi ----
    def _can_oku(self, now):
        """vcan0'dan bekleyen tüm frame'leri oku; komut state'ini tazele + watchdog resetle."""
        komut_geldi = False
        while True:
            message = self.bus.recv(timeout=0)
            if message is None:
                break
            mid = message.arbitration_id
            if mid == 0x100:
                # control.py encode: byte0-1 = throttle_pct*100 (<H), byte2 = vites, byte3 = fren %
                if len(message.data) >= 4:
                    self.throttle_pct = struct.unpack("<H", message.data[0:2])[0] / 100.0
                    self.gear = CANDecoder.decode_gear(message.data)
                    self.brake_pct = CANDecoder.decode_brake(message.data) * 100.0  # 0-1 -> 0-100
                    komut_geldi = True
            elif mid == 0x201:
                self.hedef_direksiyon = CANDecoder.decode_steering(message.data)
                komut_geldi = True
            elif mid == 0x102:
                self.handbrake = float(message.data[0]) if len(message.data) else 0.0
                komut_geldi = True
        if komut_geldi:
            self.can_komut_geldi(now)

    # ---- komut yayını ----
    def _yaz_komutlar(self, cikis, now):
        # rc_unittoOmux: SADECE alan değişiminde + omux_rate tazelemesi (A14: bus'u boğma)
        omux_alan = (int(cikis["ignition"]), int(cikis["rc_gear"]), int(cikis["emergency"]))
        yenile = (self._omux_son != omux_alan)
        if not yenile and self._omux_son_t is not None:
            gecen = (now - self._omux_son_t)
            gecen = gecen.to_sec() if hasattr(gecen, "to_sec") else float(gecen)
            yenile = gecen >= (1.0 / self.omux_rate_hz)
        if self._omux_son is None:
            yenile = True
        if yenile:
            om = self.m["rc_unittoOmux"]()
            self._stamp(om, now)
            om.RC_Ignition = int(cikis["ignition"])
            om.RC_SelectionGear = int(cikis["rc_gear"])
            om.AUTONOMOUS_EMERGENCY = int(cikis["emergency"])
            om.RC_ReverseLight = 1 if int(cikis["rc_gear"]) == 2 else 0
            self._pubs["omux"].publish(om)
            self._omux_son = omux_alan
            self._omux_son_t = now

        # Gaz
        th = self.m["RC_THRT_DATA"]()
        self._stamp(th, now)
        th.RC_THRT_PEDAL_PRESS = int(cikis["press"])
        th.RC_THRT_PEDAL_POSITION = int(cikis["position"])
        self._pubs["thrt"].publish(th)

        # Fren
        br = self.m["AUTONOMOUS_BrakePedalControl"]()
        self._stamp(br, now)
        br.AUTONOMOUS_BrakePedalMotor_EN = int(cikis["brake_en"])
        br.AUTONOMOUS_BrakeMotor_Voltage = 1
        br.AUTONOMOUS_BrakePedalMotor_ACC = int(self.fren_acc)
        br.AUTONOMOUS_BrakePedalMotor_PER = int(cikis["brake_per"])
        self._pubs["brake"].publish(br)

        # Direksiyon
        st = self.m["AUTONOMOUS_SteeringMot_Control"]()
        self._stamp(st, now)
        st.AUTONOMOUS_SteeringMot_EN = 1
        st.AUTONOMOUS_SteeringMot_PWM = int(cikis["pwm"])
        self._pubs["steer"].publish(st)

        # El freni
        hb = self.m["AUTONOMOUS_HB_MotorControl"]()
        self._stamp(hb, now)
        hb.AUTONOMOUS_HB_MotEN = 1
        hb.AUTONOMOUS_HB_MotState = int(cikis["hb_motstate"])
        hb.AUTONOMOUS_HB_Motor_PWM = int(self.hb_pwm)
        self._pubs["hb"].publish(hb)

        # E-stop bildirimi (latched)
        self._pubs["estop"].publish(Bool(data=(cikis["mod"] == "ESTOP")))

    def _heartbeat(self, now):
        """AutonomousHeardBit (ManuelSelect=1), heartbeat_rate (1 Hz)."""
        if self._hb_son_t is not None:
            gecen = (now - self._hb_son_t)
            gecen = gecen.to_sec() if hasattr(gecen, "to_sec") else float(gecen)
            if gecen < (1.0 / self.heartbeat_rate_hz):
                return
        hb = self.m["AutonomousHeardBit"]()
        self._stamp(hb, now)
        hb.AutonomousManuelSelect = 1
        self._pubs["heart"].publish(hb)
        self._hb_son_t = now

    def _durum_ilerlet(self, now):
        """Durum makinesi: INIT -> IGNITION_WAIT -> ENABLE -> DRIVE."""
        if self.durum == self.INIT:
            self.durum = self.IGNITION_WAIT
            rospy.loginfo("[BEEMOBS] INIT -> IGNITION_WAIT (RC_Ignition=1 yayinlaniyor)")
        elif self.durum == self.IGNITION_WAIT:
            if self.fb_ignition == 1:
                self.durum = self.ENABLE
                self._enable_sayac = 0
                self._enable_giris_t = now  # watchdog referansı (komut hiç gelmezse)
                rospy.loginfo("[BEEMOBS] FB_IGNITION=1 -> ENABLE (steering/brake EN)")
        elif self.durum == self.ENABLE:
            self._enable_sayac += 1
            if self._enable_sayac >= self._enable_gecikme:
                self.durum = self.DRIVE
                rospy.loginfo("[BEEMOBS] ENABLE -> DRIVE")

    def kapanis_sekansi(self):
        """rospy.on_shutdown: kapanış planını (ters sıra) araca yaz."""
        try:
            now = rospy.Time.now()
        except Exception:
            now = None
        for adim in self.kapanis_plani():
            for _ in range(3):
                try:
                    om = self.m["rc_unittoOmux"]()
                    self._stamp(om, now)
                    om.RC_Ignition = int(adim.get("ignition", 1))
                    om.RC_SelectionGear = 0
                    om.AUTONOMOUS_EMERGENCY = 0
                    self._pubs["omux"].publish(om)

                    th = self.m["RC_THRT_DATA"]()
                    self._stamp(th, now)
                    th.RC_THRT_PEDAL_PRESS = int(adim.get("press", 1))
                    th.RC_THRT_PEDAL_POSITION = self.gaz_pozisyon
                    self._pubs["thrt"].publish(th)

                    br = self.m["AUTONOMOUS_BrakePedalControl"]()
                    self._stamp(br, now)
                    br.AUTONOMOUS_BrakePedalMotor_EN = int(adim.get("brake_en", 1))
                    br.AUTONOMOUS_BrakeMotor_Voltage = 1
                    br.AUTONOMOUS_BrakePedalMotor_ACC = int(self.fren_acc)
                    br.AUTONOMOUS_BrakePedalMotor_PER = int(adim.get("brake_per", self.kapanis_fren_per))
                    self._pubs["brake"].publish(br)

                    st = self.m["AUTONOMOUS_SteeringMot_Control"]()
                    self._stamp(st, now)
                    st.AUTONOMOUS_SteeringMot_EN = int(adim.get("steer_en", 1))
                    st.AUTONOMOUS_SteeringMot_PWM = int(adim.get("pwm", self.direksiyon_dur_pwm))
                    self._pubs["steer"].publish(st)

                    rospy.sleep(0.03)
                except Exception as e:
                    rospy.logerr("[BEEMOBS] kapanis yayini basarisiz: %s", e)
                    break
        rospy.loginfo("[BEEMOBS] kapanis sekansi tamam (RC_Ignition=0, fren %%%d).",
                      self.kapanis_fren_per)

    # =====================================================================
    # ANA DÖNGÜ
    # =====================================================================
    def run(self):
        rate = rospy.Rate(self.cmd_rate_hz)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self._can_oku(now)
                self.estop_kontrol(now)
                self._durum_ilerlet(now)
                cikis = self.hesapla_cikis(now)
                self._yaz_komutlar(cikis, now)
                self._heartbeat(now)
                rospy.loginfo_throttle(
                    2.0, "[BEEMOBS] %s | gaz %.0f%% fren %.0f%% vites %d dir %.1f -> pwm %d "
                    "| FB hiz %.1f km/h aci %.1f",
                    cikis["mod"], self.throttle_pct, self.brake_pct, self.gear,
                    self.hedef_direksiyon, cikis["pwm"], self.fb_speed_kmh, self.fb_steer_deg)
            except Exception as e:
                rospy.logerr("[BEEMOBS] dongu hatasi: %s", e)
            rate.sleep()


def _param(ad, varsayilan):
    """rosparam ~ad, yoksa varsayılan (env fallback CAN_INTERFACE için main'de ayrı)."""
    try:
        return rospy.get_param("~" + ad, varsayilan)
    except Exception:
        return varsayilan


def main():
    rospy.init_node("beemobs_bridge")
    params = {
        "can_interface": os.environ.get("CAN_INTERFACE", "vcan0"),
        "cmd_rate_hz": _param("cmd_rate_hz", 10.0),
        "omux_rate_hz": _param("omux_rate_hz", 1.0),
        "heartbeat_rate_hz": _param("heartbeat_rate_hz", 1.0),
        "gaz_pozisyon": _param("gaz_pozisyon", 70),
        "gaz_olcekli": _param("gaz_olcekli", False),
        "direksiyon_dur_pwm": _param("direksiyon_dur_pwm", 0),
        "steer_pwm_kp": _param("steer_pwm_kp", 12.0),
        "steer_pwm_ters": _param("steer_pwm_ters", False),
        "steer_fb_isaret": _param("steer_fb_isaret", 1.0),
        "steer_fb_olcek": _param("steer_fb_olcek", 1.0),
        "steer_deadband": _param("steer_deadband", 0.02),
        "watchdog_sn": _param("watchdog_sn", 0.5),
        "watchdog_acil_sn": _param("watchdog_acil_sn", 2.0),
        "fren_acc": _param("fren_acc", 10000),
        "odom_topic": _param("odom_topic", "/beemobs_odom"),
        "hb_pwm": _param("hb_pwm", 100),
        "estop_vehiclestatus": _param("estop_vehiclestatus", -1),
    }
    paket_tercih = _param("msg_paketi", os.environ.get("BEEMOBS_MSG_PAKETI", "smart_can_msgs"))
    bridge = BeemobsBridge(params)
    bridge.setup_ros(paket_tercih=paket_tercih)
    bridge.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
