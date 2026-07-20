#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN Bus Waypoint Follower - PID Controller
Gazebo simülasyonunda aracı waypoint'lere götüren CAN tabanlı kontrol sistemi

CAN Mesajları:
    0x100: Gaz/Fren/Vites komutu gönder
    0x201: Direksiyon komutu gönder
    0x301: Gerçek hız yayını - ARTIK BURADAN GÖNDERİLMEZ (C9): tek yazar
           state-bridge (talos_state_to_can.py); control.py yalnız okur.
"""

import rospy
import can
import json
import math
import sys
import os
import threading
import time
import numpy as np
from collections import deque
try:
    import ackermann
except ImportError:
    ackermann = None
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion
# /karar_decision (cart_sim/Decision): karar'ın reason alanı — DUR-kaçış filtresi
# (P0 №1, inceleme 2026-07-16) bunu okur. devel mount yoksa import düşer ve
# yalnız /karar String ile çalışılır → reason hiç dolmaz → engel-DUR kaçışı
# tetiklenmez (fail-safe: özellik sessizce kapanır, davranış bozulmaz).
try:
    from cart_sim.msg import Decision as DecisionMsg
except ImportError:
    DecisionMsg = None


# =============================================================================
# GOREV GEOJSON YÜKLEYİCİ (FIX 2 — hedef_yoneticisi.py ile aynı format)
# =============================================================================
def _yukle_gorev_geojson_for_control(yol='/missions/gorev.geojson'):
    """GeoJSON'dan gorev koordinatlarını (x, y) tuple listesi olarak yükle.
    datum + start hariç tüm gorev_* ve park_giris noktalarını döndürür.
    Dosya yoksa boş liste döner (mevcut davranışı korur)."""
    if not os.path.exists(yol):
        return []
    try:
        with open(yol) as f:
            data = json.load(f)
        out = []
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            name = props.get('name', '')
            if name in ('datum', 'start'):
                continue
            if 'local_x' not in props or 'local_y' not in props:
                continue
            out.append((float(props['local_x']), float(props['local_y'])))
        return out
    except (OSError, ValueError, KeyError):
        return []


# =============================================================================
# ENGEL GEOMETRİSİ (saf, ROS'suz → standalone test edilebilir)
# =============================================================================
# `/obstacles/poses` (geometry_msgs/PoseArray) konumları araç GÖVDE çerçevesinde,
# REP-103: x ileri, y sol (talos_obstacle_detector velodyne frame'inde hesaplar;
# karar/ros_bridge.py ile aynı konvansiyon). H-B e-stop güvenlik ağı: dead-ahead
# yakındaki engeli seçip, mevcut direksiyonla (Ackermann yayı) GERÇEKTEN çarpacaksa
# tam fren bas (sadece mesafe değil → keskin dönüşte aradan geçişe izin verir).

def select_blocking_obstacle(points, fwd_min, fwd_max, corridor_m):
    """Gövde-çerçevesi engel noktaları arasından yolu bloklayan EN YAKIN olanı seç.

    points: [(fwd, lat), ...]  (fwd ileri +, lat sol +; metre)
    Aday = önümüzde (fwd_min < fwd < fwd_max) ve koridor içinde (|lat| < corridor_m).
    Döner: (fwd, lat) en yakın aday, ya da None.
    """
    best = None
    for fwd, lat in points:
        if fwd <= fwd_min or fwd >= fwd_max:
            continue
        if abs(lat) >= corridor_m:
            continue
        if best is None or fwd < best[0]:
            best = (fwd, lat)
    return best


def select_arc_blocking_obstacle(points, fwd_min, fwd_max, corridor_m,
                                 steer_deg, wheelbase_m, half_width_m,
                                 sensor_to_ra_m, nose_m):
    """Koridor adayları arasından mevcut direksiyonun 2B SÜPÜRME BANDININ
    içine aldığı en yakın engeli seç (yoksa None).

    İncele düzeltmesi (2026-07-04, CONFIRMED): eskiden yalnız en-yakın aday yay
    testine sokuluyordu — yakın-ama-yanal (yayın zaten geçtiği) bir koni, biraz
    uzaktaki tam-önde koniyi gölgeleyip e-stop'u susturabiliyordu (ör. A=(1.0,1.3)
    seçilir ve 'geçer', B=(2.0,0.1) hiç denetlenmez). Artık TÜM adaylar yay
    testinden geçirilir; yalnızca hepsi geçilirse yol açık sayılır."""
    best = None
    for fwd, lat in points:
        if fwd <= fwd_min or fwd >= fwd_max:
            continue
        if abs(lat) >= corridor_m:
            continue
        if ackermann_path_clears(fwd, lat, steer_deg, wheelbase_m,
                                 half_width_m, sensor_to_ra_m, nose_m):
            continue
        if best is None or fwd < best[0]:
            best = (fwd, lat)
    return best


def ackermann_path_clears(fwd, lat, steer_deg, wheelbase_m, half_width_m,
                          sensor_to_ra_m, nose_m):
    """Araç MEVCUT direksiyonla giderken 2B SÜPÜRME BANDI engeli (fwd, lat)
    içine alıyor mu? (kullanıcı 2026-07-04: bisiklet yayını araç GENİŞLİĞİYLE
    iki boyutlu yap; koni bandın dışındaysa dur emri verme)

    (fwd, lat) SENSÖR (lidar) çerçevesindedir; bisiklet modeli ARKA AKS
    referanslıdır → ICR sensörün sensor_to_ra_m arkasında: (−sensor_to_ra_m, ±R).
    (Eski kod ICR'yi (0, ±R) alıyordu = yayı lidar-arka aks mesafesi kadar,
    ~1.76 m, İLERİ kaydırma hatası; ayrıca genişlik yerine tek 0.9 m 'pay' vardı.)

    Düz (~0°): |lat| ≥ half_width_m → geçer (bant sensör ekseniyle paralel).
    Dönüşte: R = L/tan|δ| (δ>0 sol → ICR +y). Gövdenin süpürdüğü HALKA:
      iç kenar  r_ic  = R − half_width (arka iç yan)
      dış kenar r_dis = hypot(R + half_width, nose_m) (ÖN DIŞ KÖŞE dışa taşar)
    Engelin ICR'ye uzaklığı d_c bu halkanın DIŞINDAysa (d_c ≤ r_ic veya
    d_c ≥ r_dis) araç engele değmeden geçer.
    True = geçer (DURMA gerekmez); False = bant içinde → engel."""
    delta = math.radians(steer_deg)
    if abs(delta) < math.radians(1.0):
        return abs(lat) >= half_width_m        # düz gidiş: yanal ayrım yeterli mi
    R = wheelbase_m / math.tan(abs(delta))
    cy = R if delta > 0.0 else -R              # sol dönüş (δ>0) → dönüş merkezi +y
    d_c = math.hypot(fwd + sensor_to_ra_m, lat - cy)
    r_ic = R - half_width_m
    r_dis = math.hypot(R + half_width_m, nose_m)
    return d_c <= r_ic or d_c >= r_dis


def govde_disi_filtre(points_ra, arka_m, burun_m, yari_gen_m, pay_m=0.05):
    """Aracın KENDİ GÖVDE kutusu içindeki noktaları ele (arka-aks çerçevesi).

    CANLI BULGU (run 092553Z, 2026-07-15): /obstacles/poses araç-üstü noktalar
    da içeriyor (arka-aks +0.55m, lat ~0 — lidar'ın 1.2m ARKASI, yani gövde).
    E-stop OBSTACLE_FWD_MIN=0.3 (yalnız önde) ile bağışıktı; güvenli-dönüş
    kapısı yan/arka noktalara muhtaç olduğundan o filtreyi kullanamaz → gövde
    kutusu dışlaması şart, yoksa alan HİÇ temizlenmez (DÜZ TUT → timeout →
    İPTAL → pursuit engele sürer; bag replay kanıtlı). Gerçek bir koni gövde
    kutusunun içindeyse zaten temas etmiştir — kapının kurtaracağı durum değil."""
    return [(f, l) for f, l in points_ra
            if not ((-arka_m - pay_m) <= f <= (burun_m + pay_m)
                    and abs(l) <= yari_gen_m + pay_m)]


def tam_kilit_donus_riskli(points_ra, swing_deg, donus_yonu, max_steer_deg,
                           wheelbase_m, half_width_m, nose_m, arka_m, tail_m):
    """GÜVENLİ DÖNÜŞ ALANI: araç ŞU AN tam-kilitle "eski şeride dönüş"e başlasa
    süpüreceği alanın içinde engel var mı? (kullanıcı 2026-07-15: koni bu riskli
    alandan çıkmadan şeride dönme.)

    Dönüş yolu iki parçadan oluşur (ARKA AKS çerçevesi, x ileri / y sol):
      1) YAY: tam-kilit (R = L/tan(max_steer)) donus_yonu yönünde, heading
         swing_deg kadar dönene dek (S-manevra başlangıç yönüne hizalanma).
         Gövde süpürme HALKASI [R−half_width, hypot(R+half_width, nose)]; açı
         sektörü [−atan(arka/R), swing + atan(nose/R)] (arka/ön taşma payları).
      2) KUYRUK: yay bitiminden itibaren tail_m uzunluğunda düz koridor
         (|yanal| ≤ half_width) — dönüş tamamlanınca eski şeritte üstünden
         geçilecek bölge. Kuyruk olmadan, manevra başında hâlâ ÖNDE duran koni
         "yay değmiyor" diye alan-dışı sayılır ve araç koninin üstüne dönerdi.
         NOT: ön-iç köşe kapsaması kısmen kuyruğun yay bitişiyle hizalı ve
         half_width genişliğinde olmasına dayanır — kuyruksuz (tail_m=0) veya
         farklı genişlikli kullanım (ör. gelecekteki park manevrası) bu örtük
         telafiyi kaybeder; on_pay/arka_pay bu yüzden r_ic ile (muhafazakâr)
         hesaplanır (algoritma incelemesi 2026-07-15).

    points_ra : ARKA AKS çerçevesinde (fwd, lat) engel noktaları
    swing_deg : şu anki heading swing'i (başlangıç yönünden sapma, ≥0 beklenir)
    donus_yonu: dönüş yönü işareti (+1 sol, −1 sağ) — S-manevrada −_sman_dir
    Döner: alan İÇİNDEKİ araca en yakın nokta (fwd, lat) ya da None (temiz)."""
    phi = math.radians(max(0.0, swing_deg))
    R = wheelbase_m / math.tan(math.radians(max_steer_deg))
    s = 1.0 if donus_yonu > 0 else -1.0
    icr_y = s * R
    r_ic = R - half_width_m
    # Dış yarıçap = ICR'den en uzak GÖVDE KÖŞESİ (ön/arka hangisi uzunsa) +
    # FP-epsilon (köşe tam sınır yarıçapında sabit kalır; d_c<=r_dis testi
    # 1e-15'lik aritmetik farkla titremesin — algoritma incelemesi 2026-07-15).
    r_dis = math.hypot(R + half_width_m, max(nose_m, arka_m)) + 1e-9
    # Açısal taşma payları İÇ yarıçapla (r_ic) hesaplanır: ICR tarafındaki köşe
    # aynı boyuna ötelemeye daha BÜYÜK açıyla taşar (atan2(x, r) r küçüldükçe
    # büyür). Eski atan2(x, R) merkez-hat varsayımı arka-iç köşe şeridini hiçbir
    # swing'de yakalamıyordu (algoritma incelemesi 2026-07-15, sayısal kanıtlı).
    on_pay = math.atan2(nose_m, r_ic)    # burnun açısal taşması (yay sektörü ilerisi)
    arka_pay = math.atan2(arka_m, r_ic)  # arka tamponun açısal taşması (sektör gerisi)
    # Yay bitişindeki arka-aks konumu + oradaki heading (kuyruk ekseni)
    ex = R * math.sin(phi)
    ey = s * R * (1.0 - math.cos(phi))
    ux = math.cos(s * phi)
    uy = math.sin(s * phi)

    best = None
    for fwd, lat in points_ra:
        riskli = False
        # (1) YAY halkası + açı sektörü. Nokta açısı, başlangıç konum vektöründen
        #     (araç→ICR ekseni) gidiş yönünde ölçülür: θ = s·açı(v0→v).
        vx, vy = fwd, lat - icr_y
        d_c = math.hypot(vx, vy)
        if r_ic <= d_c <= r_dis:
            v0x, v0y = 0.0, -icr_y
            theta = s * math.atan2(v0x * vy - v0y * vx, v0x * vx + v0y * vy)
            if -arka_pay <= theta <= phi + on_pay:
                riskli = True
        # (2) KUYRUK düz koridoru (yay bitiminden ileri)
        if not riskli:
            wx, wy = fwd - ex, lat - ey
            along = wx * ux + wy * uy
            yanal = ux * wy - uy * wx
            if 0.0 <= along <= tail_m and abs(yanal) <= half_width_m:
                riskli = True
        if riskli:
            d_arac = math.hypot(fwd, lat)
            if best is None or d_arac < math.hypot(best[0], best[1]):
                best = (fwd, lat)
    return best


# =============================================================================
# LOGGER
# =============================================================================

class Logger:
    """Log ve CSV kaydedici.

    ZAMAN DAMGASI SÖZLEŞMESİ (diğer loglarla birebir aynı — karar/trace.csv,
    hedef/pose.csv, *events.jsonl): her satır üç zaman taşır →
        t_unix : time.time() epoch (round 3) — makine sıralama
        t_iso  : UTC ISO 8601 + 'Z' (gmtime; TZ'den BAĞIMSIZ) — insan/eşleştirme
        ros_t  : rospy.Time.now() (sim-time; use_sim_time) — rosbag/candump hizası
    Eski tek-sütun ham epoch 'timestamp' (insan-okunmaz + ros_t yok) kaldırıldı;
    artık CSV'yi karar/hedef logları ve rosbag ile saniyesine kadar eşleyebiliriz.
    .log öneki de gmtime UTC'dir (eski datetime.now() yereldi → TZ değişirse
    diğer UTC loglardan kayardı; tarih/Z/ros_t da yoktu)."""

    def __init__(self, log_dir='/app/logs/'):
        self.log_dir = log_dir
        try:
            os.makedirs(log_dir, exist_ok=True)
        except OSError:
            self.log_dir = '/tmp/talos_logs/'
            os.makedirs(self.log_dir, exist_ok=True)

        # Dosya adı UTC (gmtime) — RUN_ID ve t_iso ile aynı saat dilimi; konteyner
        # TZ'i değişse bile karar/hedef RUN_ID'leriyle aynı damgayı taşır.
        timestamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime())
        self.log_file = os.path.join(self.log_dir, f'control_{timestamp}.log')
        self.csv_file = os.path.join(self.log_dir, f'control_{timestamp}.csv')

        # CSV header — t_unix,t_iso,ros_t öneki karar/trace.csv & hedef/pose.csv ile aynı
        with open(self.csv_file, 'w') as f:
            f.write('t_unix,t_iso,ros_t,x,y,yaw,speed_kmh,karar,'
                    'target_x,target_y,throttle,brake,steer,gear\n')
        # CSV'yi açık tut: per-tick open/close (50 Hz → 50 aç-kapa/sn) yerine
        # kalıcı handle + periyodik flush (rapor H5 perf). Ctrl+C'de en fazla
        # ~0.5 s satır kaybı (analiz için kabul edilebilir).
        try:
            self._csv_fh = open(self.csv_file, 'a')
        except OSError:
            self._csv_fh = None
        self._csv_since_flush = 0

    @staticmethod
    def _iso(t):
        """UTC ISO 8601 + Z (hedef/karar logger ile birebir). gmtime → TZ-bağımsız."""
        return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(t))

    @staticmethod
    def _ros_t():
        """sim-time (use_sim_time) saniye → rosbag/candump/karar ile hizalı.
        Node init olmadan patlamasın diye korumalı. /clock henüz akmıyorken
        rospy.Time.now() İSTİSNA atmaz, Time(0)=0.0 döner → yanıltıcı 'ros_t=0.000'
        yerine None ver (ROS bulgusu, §incele)."""
        try:
            t = rospy.Time.now().to_sec()
            return round(t, 3) if t > 0.0 else None
        except Exception:
            return None

    def log(self, message):
        """Log mesajı yaz — öneki [t_iso ros=<ros_t>] (UTC, diğer loglarla hizalı)."""
        t = time.time()
        ros = self._ros_t()
        ros_str = f"{ros:.3f}" if ros is not None else "--"
        line = f"[{self._iso(t)} ros={ros_str}] {message}"
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
        except OSError:
            pass
        rospy.loginfo(message)

    def csv(self, x, y, yaw, speed_kmh, karar, target_x, target_y, throttle, brake, steer, gear):
        """CSV satırı yaz — t_unix,t_iso,ros_t üçlü zaman damgası ile."""
        if self._csv_fh is None:
            return
        t = time.time()
        ros = self._ros_t()
        ros_str = f"{ros:.3f}" if ros is not None else ""
        try:
            self._csv_fh.write(f'{t:.3f},{self._iso(t)},{ros_str},'
                               f'{x:.3f},{y:.3f},{yaw:.3f},{speed_kmh:.2f},{karar},'
                               f'{target_x:.3f},{target_y:.3f},{throttle:.1f},{brake:.1f},{steer:.1f},{gear}\n')
            self._csv_since_flush += 1
            if self._csv_since_flush >= 25:   # ~0.5 s @ 50 Hz
                self._csv_fh.flush()
                self._csv_since_flush = 0
        except (OSError, ValueError):
            pass

    def close(self):
        """CSV handle'ı flush+kapat (kapanışta son satırlar kaybolmasın)."""
        fh = getattr(self, '_csv_fh', None)
        if fh is not None:
            try:
                fh.flush()
                fh.close()
            except (OSError, ValueError):
                pass
            self._csv_fh = None


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
# --- Aktif fren (yavaşlama) ---
# Hız PID'i output_min=0 olduğu için fren üretemez; hedef hızın üstündeyken
# bu parametrelerle ORANTILI fren basılır. (karar=slow → 2.5 km/h gerçekten iner.)
SLOWDOWN_BRAKE_MARGIN = 0.4  # km/h - hedefin bu kadar üstündeysek frene başla (deadband/chatter koruması)
SLOWDOWN_BRAKE_GAIN = 20.0   # fren% / (km/h aşım) - frenin dikliği
SLOWDOWN_BRAKE_MAX = 45.0    # % - yavaşlama freni tavanı (yumuşak; DUR/ACIL ayrı tam fren)
# --- /karar staleness watchdog (fail-safe) ---
KARAR_TIMEOUT = 1.0         # saniye - karar node bu süre sessiz kalırsa DUR (karar 10Hz)
DUR_WAIT_TIME = 17.0        # saniye - durakta bekleme (yolcu al/bırak 15-20 sn şartı, +70/nokta — rapor §2/H5; eski 3.0 puan kaybı)
# --- Engelden kaçınma MİMARİSİ (H-A/H-B, 2026-06-24) ---
# KARAR (mimari dönüş): control artık sentetik yanal-offset manevrası YAPMAZ.
# Kör ±offset (eski H2, §12.10/§12.12) iki canlı koşuda da engeli geçemedi —
# 3.5 m tetik mesafesinde golf-cart geometrisi 1.8 m yanalı açacak yanal yetkiyi
# üretemiyor (R≈3.08 m'de ~2.8 m gerek, rampa payı yiyor). Yeni mimari:
#   • Kaçınma = PLANLAYICI işi: karar 6 m'de tetikler, hedef rotayı engelin
#     etrafından (karşı şerit/slalom kenarları) çizer → control DÜZ TAKİP eder.
#   • control = Pure Pursuit + /line düz takip (H-A: offset + abort/turn guard YOK).
#   • Güvenlik ağı = doğrudan latched e-stop (H-B): /obstacles/poses dead-ahead
#     dar koridorda yakınsa tam fren — /karar merdiveninin ÜSTÜNDE, 10 Hz BT
#     gecikmesini atlar. Dar koridor → reroute'u ÖLDÜRMEZ (araç yana açılınca
#     engel koridordan çıkar, e-stop bırakır). Tek-yön şeritte reroute yoksa
#     araç engelde durur (çarpmaz) — güvenli; gerçek geçiş hedef'in slalom
#     kenarlarına bağlı (Samed, Faz 1).
# --- H-B: Doğrudan e-stop güvenlik ağı (/obstacles/poses, latched, Ackermann-farkında) ---
# CANLI BULGU (run 185215): sabit 2.5 m/dar-koridor e-stop, araç reroute dönüşünü
# tam yapmadan dubayı 2.4 m'de koridorda görüp DURDURDU — oysa keskin dönerse aradan
# geçerdi (kullanıcı). Çözüm: e-stop sadece MESAFE değil, mevcut direksiyonun Ackermann
# yayı engele GERÇEKTEN çarpıyorsa tetiklensin (+ çok-yakın hard floor son çare).
OBSTACLE_TOPIC = '/obstacles/poses'  # geometry_msgs/PoseArray (gövde çerçevesi, x ileri / y sol)
OBSTACLE_TIMEOUT = 0.5        # s - tampon bu süre tazelenmezse "stale"; e-stop latch'i KORUNUR (donmada fren bırakmaz)
OBSTACLE_FWD_MIN = 0.3        # m - tamponun dibindeki noktaları (kendi gövdemiz) yok say
ESTOP_HARD_M = 1.0            # m - bu kadar yakın + dar koridor → KOŞULSUZ tam fren (son çare, çok geç)
ESTOP_FWD_M = 2.5             # m - bu menzile kadar Ackermann-yay kontrolü yapılır (çarpacaksa dur)
ESTOP_CORRIDOR_M = 0.7        # m - hard floor DAR koridoru (gerçekten yolun ortasındaki engel)
ESTOP_CHECK_CORRIDOR_M = 1.5  # m - Ackermann kontrolü için (genişçe) aday koridoru; yay yine de geçerse durmaz
# 2B süpürme bandı geometrisi (eski tek ESTOP_SAFE_RADIUS=0.9 'payı' yerine —
# kullanıcı 2026-07-04: bant = araç genişliği; koni dışındaysa dur yok):
ARAC_GENISLIK_M = 1.2         # m - araç gövde genişliği (yarı-gen. 0.6)
ENGEL_YARICAP_M = 0.15        # m - duba/koni yarıçapı (banda eklenir)
ESTOP_BANT_YARIM_M = ARAC_GENISLIK_M / 2.0 + ENGEL_YARICAP_M  # 0.75 m - bant yarı-genişliği; düşür=daha cesur geçiş
LIDAR_ARKA_AKS_M = 1.76       # m - lidar → arka aks mesafesi (sim: urdf lidar x=+0.9, arka aks x=−0.862; Bee1'de sahada kalibre edilecek)
ARAC_BURUN_M = 2.34           # m - arka aks → ön tampon (dönüşte ÖN DIŞ KÖŞE süpürmesi bununla
                              #     hesaplanır; golf.urdf ölçümü 2026-07-15: ön tampon kutusu
                              #     x=1.177+0.3=1.477, arka aks x=−0.862 → 1.477+0.862=2.34)
ESTOP_RELEASE_S = 1.0         # s - tazece TEMİZ kalınca e-stop'u bırakmadan önceki debounce
                              #     (P1 №8 / E5-O5: 0.5→1.0 — bang-bang periyodunu uzatır; canli_params'ta)
ESTOP_HISTEREZIS_M = 0.10     # m - BIRAKMA testinin bant yarı-genişliğine eklenen uzamsal histerezis
                              #     (P1 №8 / E5-O1: tetik ve bırakma AYNI bantla değerlendirildiğinden
                              #      bant KENARINDAKİ engel (E5: 7 tetiğin hepsi yanal +0.57..+0.72 m)
                              #      jitter'la tetik→bırak limit çevrimi kuruyordu; bırakma artık
                              #      +0.10 m geniş bandın da temiz olmasını ister. Tetik yolu AYNEN.)
ESTOP_CRAWL_KMH = 1.5         # km/h - SOFT (Ackermann-yay) e-stop: tam fren YERİNE bu hıza
                              #   sınırla → S-manevra/pursuit engeli sollayabilsin (deadlock fix,
                              #   run 214118). HARD floor (<ESTOP_HARD_M) hâlâ koşulsuz tam fren.
# Keskin dönüş (waypoint'e doğru): heading hatası büyükse Pure Pursuit lookahead'i
# KISALT → daha sert direksiyon → reroute'u tam takip eder (CANLI: 12° yumuşak kaldı).
SHARP_TURN_DEG = 15.0         # derece - lookahead noktasına |açı| bunu aşınca kısa lookahead'e geç
SHARP_LOOKAHEAD = 2.0         # m - keskin dönüşte lookahead tavanı (sert direksiyon)

# Gercek gorev duraklari (hedef_yoneticisi'ndeki GeoJSON ile ayni).
# FIX 2 — boş liste yerine başlangıçta GeoJSON'dan yüklenir; başarısız
# olursa boş kalır (geriye uyum). Mount edilen yol: /missions/gorev.geojson.
GOREV_NOKTALARI = _yukle_gorev_geojson_for_control()
if not GOREV_NOKTALARI:
    # rospy.init_node henüz çağrılmadı (modül load zamanı) → stderr'e yaz
    print("[control.py][WARN] GOREV_NOKTALARI boş — /missions/gorev.geojson "
          "okunamadı veya mount eksik. Durak ziyareti, SLOWDOWN_DISTANCE, "
          "mission_complete devre dışı.", file=sys.stderr, flush=True)
GOREV_THRESHOLD = 3.0       # metre - duraga varis esigi
GOREV_COOLDOWN = 10.0       # saniye - ayni duraga tekrar varildi engeli
WP_NEAR_DISTANCE = 1.5      # metre - WP1'e yakinken WP2'ye gec (lookahead)


# =============================================================================
# ARAÇ PARAMETRELERİ - TÜM AYARLAR BURADA
# =============================================================================

# --- Döngü ---
LOOP_RATE_HZ = 50                            # Ana kontrol döngüsü frekansı (run loop + slew dt)
LOOP_DT = 1.0 / LOOP_RATE_HZ                 # saniye - slew-rate ve zaman hesapları bu sabiti kullanır

# --- Hız Ayarları ---
MAX_SPEED_KMH = 5.0                          # Maksimum hız (km/h)
MAX_SPEED_MS = MAX_SPEED_KMH / 3.6           # Maksimum hız (m/s) - otomatik hesaplanır
# --- Direksiyon Ayarları ---
try:
    MAX_STEER_ANGLE = ackermann.max_bicycle_angle()  # ≈28.95° — Bee1 teker limitlerinden (ackermann.py)
except Exception:
    MAX_STEER_ANGLE = 28.95                  # fallback (ackermann.py import edilemedi)
# Direksiyon SLEW-RATE limiti (anti-oscillation, §17 teşhisi). Direksiyon tek
# tick'te bundan hızlı DEĞİŞEMEZ → flip-flop snap'i (+9°→−30° tek tick) yumuşar,
# line-gate/sharp-gate süreksizliklerinden artan titreme sönümlenir. Gerçek
# direksiyon aktüatörünün fiziksel hız sınırını da taklit eder.
# 200°/s @ 50 Hz = 4°/tick: düz pursuit (~<2°/tick) etkilenmez; 39°'lik snap
# ~0.2 s'de rampalanır; 11°/tick line-chatter 4'e kırpılır. TUNABLE (canlı ayar).
STEER_RATE_MAX_DEG_S = 200.0

# --- Pure Pursuit Direksiyon Kontrolü ---
WHEELBASE = 1.86             # metre - Bee1 dingil mesafesi (golf.urdf da 2026-07-04'te 1.86'ya hizalandı)
LOOKAHEAD_K = 1.0            # lookahead hız katsayısı (Ld = K*v + B, v: m/s)
LOOKAHEAD_B = 2.5            # metre - sabit lookahead bileşeni (v=0'da Ld)
LOOKAHEAD_MIN = 1.5          # metre - lookahead alt sınırı (aşırı direksiyon koruması)
LOOKAHEAD_MAX = 6.0          # metre - lookahead üst sınırı
LINE_GATE_MAX_HEADING = 12.0 # derece - /line düzeltmesi sadece bu heading hatasının altında uygulanır

# --- Waypoint Toleransları ---
ARRIVAL_THRESHOLD = 3.0                      # Waypoint'e ulaşma eşiği (metre)
SLOWDOWN_DISTANCE = 4.0                      # Yavaşlamaya başlama mesafesi (metre)
STOP_DISTANCE = 1.2                          # Tamamen durma mesafesi (metre)

# --- Viraj Yavaşlama (ileriye bakan heading) ---
TURN_SLOWDOWN_THRESHOLD = 10.0               # derece - bir sonraki WP'ye heading hatası bunu aşınca yavaşla
TURN_SLOWDOWN_GAIN = 2.5                     # yavaşlama eğrisinin dikliği (büyük = daha sert yavaşlar)
TURN_MIN_SPEED = 1.5                         # km/h - virajda minimum hedef hız (tekerlek kuvveti için)

# --- CAN Bus Ayarları ---
CAN_INTERFACE = 'vcan0'                      # CAN arayüzü

# --- Şerit Takip (Line Following) Ayarları ---
LINE_TOPIC = '/line'                         # Şerit açısı topic'i
LINE_ENABLED = True                          # Şerit takibi aktif mi?
LINE_WEIGHT = 0.9                            # Şerit düzeltme ağırlığı
LINE_TIMEOUT = 0.5                           # Veri timeout süresi (saniye)
LINE_MAX_ANGLE = 25.0                        # Güvenilir maksimum açı (derece)
LINE_OFFSET = 0.0                            # Kamera kalibrasyonu offset (derece)
# /line BASTIRMA (anti-oscillation, §17 BİRİNCİL kök neden). /line bir CRUISE
# şerit-ortalayıcıdır; engel-yakını/slalom (karar≠normal) bağlamında rota zaten
# yanal manevra yapıyor → /line ters yöne (~−11°) basıp pure-pursuit'le bang-bang
# yapıyordu (heading 12° kapısında titreme). Engel bağlamında /line susturulur;
# karar normal'e döndükten LINE_SUPPRESS_TTL sn sonra tekrar açılır (slalom
# içindeki kısa 'normal' blip'lerinde /line geri gelip titretmesin).
LINE_SUPPRESS_TTL = 2.0                       # saniye - son engel-bağlamı kararından sonra /line susuk kalır

# --- KESKİN S-MANEVRA (slalom, §18) ---
# Kullanıcı direktifi: dubayı gör + reroute WP gelince control deterministik
# TAM SOL yapar, sol-şerit WP hizasına gelene kadar direksiyonu TOPLAMAZ, sonra
# TAM SAĞ ile ikinci keskin dönüşü yapar (simetrik, geniş-salınımlı S). Manevra
# aktifken steering'i o SAHİPLENİR → pursuit + /line + sharp-gate bypass; latched
# WP referansı sayesinde hedef flip-flop'undan (§17 mek-3) ETKİLENMEZ. §16 slalom
# için revize (control düz-takip yetersizdi: pursuit sola zayıf çıkıp sağa
# full-lock snap atıyordu — CANLI run 135822). Tetik: yalnız hedef reroute WP
# sıçraması (kullanıcı Q2; /obstacles bağımlılığı YOK). H-B e-stop güvenlik ağı
# manevranın ÜSTÜNDE çalışmaya devam eder.
SMANEUVER_ENABLED = True
SMANEUVER_REROUTE_LATERAL_M = 1.8   # m - /hedef WP tek güncellemede bu kadar YANA adımlarsa = reroute (slalom) tetik
                                    # (1.2→1.8: keskin track virajında ardışık WP yanal adımı ~1.4m olabiliyor;
                                    #  reroute sıçraması ~2.3m → 1.8 ikisini ayırır. + engel-bağlamı kapısı, aşağı)
SMANEUVER_CTX_TTL = 3.0             # s - tetik yalnız engel-bağlamında (son karar≠normal'den bu süre içinde) geçerli;
                                    # engelsiz track virajı (karar=normal) yanlış S-manevra başlatmasın (§incele algo)
# Faz TAM SOL bitişi: araç (başlangıç-yönü çerçevesinde) sol-WP'nin YANAL hizasına
# gelince VEYA heading swing'i SWING_DEG'i aşınca → TAM SAĞ'a geç. Swing kapısı
# ŞART: golf-cart yavaşken (2.5 km/h, full-lock ~13°/s) "2.2m WP'ye kadar tut"
# ~73° dönüş = AŞIRI DÖNME/döngü demek (offline bisiklet-sim kanıtı). Swing kapısı
# bunu keser; hız yeterse WP yanal hizası baskın olur (kullanıcı niyeti korunur).
SMANEUVER_MAX_SWING_DEG = 45.0     # derece - faz TAM SOL maksimum heading swing'i (döngü-önleme)
SMANEUVER_ALIGN_DEG = 8.0          # derece - faz TAM SAĞ: heading başlangıç-yönüne bu kadar dönünce manevra biter (pursuit devralır)
SMANEUVER_TIMEOUT = 6.0            # s - faz başına süre güvenliği; aşılırsa pursuit'e dön
SMANEUVER_PENDING_TTL = 1.0       # s - reroute tetiği bu kadar taze olmalı (eski sıçramayla başlamasın)

# --- GÜVENLİ DÖNÜŞ KAPISI (kullanıcı 2026-07-15): tam-kilit süpürme alanı ---
# "Eski şeride dönüş" (TOWARD→AWAY) artık koni-farkında: her tick, araç ŞİMDİ
# tam-kilit ters dönüşe başlasa süpüreceği alan (yay halkası + eski-şeritteki
# düz koridor; bkz. tam_kilit_donus_riskli) hesaplanır. Koni bu RİSKLİ ALANDAN
# çıkmadan dönüş BAŞLAMAZ; çıkınca (debounce sonrası) hemen başlar. Swing kapısı
# (45°) döngü-önleme backstop'u olarak kalır. Koni verisi: taze /obstacles/poses
# + dünya-çerçevesi koni hafızası (detektör kare düşürünce erken dönmesin —
# koniler statik). Veri tamamen yoksa eski sol-WP-hizası davranışına düşülür.
# Aynı saf fonksiyon gelecekteki PARK/DURAK manevra dönüşlerinde de kullanılacak
# (şerit terk edip geri girilen HER manevra bu kapıdan geçmeli).
SMAN_DONUS_KAPISI_AKTIF = True
# KORİDOR SINIRI (şartname s.8: "şerit ihlali (2 tekerlek tamamen dışarı)" ceza,
# kurallara uymamak diskalifiye; run 092553Z'de sınırsız DÜZ TUT aracı WP
# hizasının 1.75m ötesine sürükledi → YOLDAN ÇIKMA/ELEME). Reroute WP hizası =
# planlayıcının yasal koridoru; yanal sapma + dönüş yayının EK kazancı bu hizayı
# +ASIM'dan fazla AŞAMAZ — koni riskli olsa da dönüş ZORLANIR (e-stop emekleme
# korur). Öncelik: yolda kal > koni klirensi.
SMAN_KORIDOR_ASIM_M = 0.3     # m - WP hizası üzerine izinli yanal aşım payı
SMAN_DONUS_TAIL_M = 8.0       # m - dönüş sonrası eski-şeritte denetlenen düz koridor (arka akstan; kısa=erken dönüş, uzun=sonraki koniye takılma)
SMAN_DONUS_KLIRENS_M = 0.15   # m - süpürme bandına ek güvenlik payı (ESTOP_BANT_YARIM_M üstüne)
SMAN_DONUS_TEMIZ_S = 0.3      # s - alan bu süre KESİNTİSİZ temiz kalmadan dönüş başlamaz (tek-kare gürültü debounce'u)
KONI_HAFIZA_TTL_S = 3.0       # s - dünya koni hafızası yaşam süresi (dropout köprüsü)
KONI_HAFIZA_ESLE_M = 0.5      # m - yeni tespit bu mesafedeki hafıza kaydını tazeler (aynı koni)
KONI_HAFIZA_MAX = 30          # adet - hafıza tavanı (gürültü patlamasına sigorta)
BASE_ARKA_AKS_M = 0.862       # m - base_link → arka aks (golf.urdf rear_wheel_joint x=−0.862; LIDAR_ARKA_AKS_M ≈ 0.9 + 0.862)
ARAC_ARKA_M = 0.5             # m - arka aks → arka taşma (golf.urdf back_bumper 0.33 + pay; dönüşte arka köşe açısal payı)

# --- Vites Sabitleri (değiştirmeyin) ---
GEAR_NEUTRAL = 1
GEAR_FORWARD = 2
GEAR_REVERSE = 3            # cart_control.msg REVERSE=3 - geri sürüş vitesi

# --- Geri Sürüş (breadcrumb retrace) ---
# İleri sürüşte gönderilen (direksiyon, gaz) çifti "iz" olarak kaydedilir;
# /geri_komut True olunca iz TERS sırayla + vites REVERSE ile oynatılır →
# araç geldiği rotayı geri izler. Direksiyon İŞARETİ AYNI kalır (bisiklet
# modeli: hız ters + aynı direksiyon açısı = aynı yayı geri çizer; negatiflemek
# retrace'i bozar). NOT: arka sensör YOK → yalnız az önce ileri geçilen (temiz)
# izi geri izlemek içindir; ACIL_DURUS geri sürüşte de tam fren'i korur.
GERI_BREADCRUMB_MAXLEN = 20000   # maks iz örneği (50Hz'de ~ hareketli 6-7 dk); eski örnekler düşer
GERI_RECORD_MIN_KMH = 0.2        # km/h - yalnız bu hızın üstünde ilerlerken kaydet (dur/emekle tick'leri izi şişirmesin)
GERI_MAX_THROTTLE = 100.0        # % - geri sürüş gaz tavanı (TUNABLE; düşür=daha yavaş/güvenli ama retrace sadakati azalır)

# --- Sıkışma Kaçışı (stuck → geri) ---
# Araç ilerlemeye ÇALIŞIRKEN (hedef hız var) bu süre boyunca hareketsiz kalırsa
# = kurtulamadığı bir engele saplanmış → izini GERİ_ESCAPE_DISTANCE_M kadar geri
# izleyerek engelden çık. Anti-stall kick-start (FIX 4, ~2 s) önce şansını denesin
# diye eşik ondan uzun. Kaçış tamamlanınca COOLDOWN boyunca yeniden tetiklenmez
# (aynı engelde ileri-geri thrash olmasın; kalıcı kurtuluş planlayıcının işi).
STUCK_ESCAPE_TIME_S = 3.0        # s - ilerlemeye çalışırken bu kadar hareketsiz = sıkıştı
STUCK_ESCAPE_SPEED_KMH = 0.3     # km/h - bu hızın altı "hareketsiz" sayılır
GERI_ESCAPE_DISTANCE_M = 2.0     # m - sıkışma kaçışında geri gidilecek mesafe (kullanıcı 2026-07-04: 1→2 m; 1 m graf start-node'unu değiştirmiyordu → aynı reddedilen rota döngüsü)
STUCK_ESCAPE_COOLDOWN_S = 5.0    # s - kaçıştan sonra yeniden tetikleme bekleme süresi
STUCK_EVAL_GAP_S = 0.5           # s - _stuck_check bundan uzun süre çağrılmadıysa sayaç bayat → sıfırla
                                 #     (P0 №2, inceleme 2026-07-16 E1-O2/E4-O2: DUR/ACİL dallarında
                                 #      _stuck_check hiç koşmadığından sayaç eski bir cruise-flicker'ından
                                 #      donuk kalıp 209 s sonra şans eseri ateşliyordu)

# --- Engel-DUR kilidi kaçışı (P0 №1, inceleme 2026-07-16 E1-O1/E8-R2) ---
# Karar=dur kilidi: reason ENGEL BLOKAJI ise ve araç DUR_KACIS_TIME_S boyunca
# hareketsizse mevcut breadcrumb kaçışı (_start_geri) tetiklenir → araç geri
# çekilir, hedef'in start-node'u değişir, cusp-reddine takılan recalc fizibil
# olur. Reason filtresi: levha/yaya/kırmızı-ışık dur'larında geri kaçış ASLA.
# ACİL dalına bilerek KONMADI (güvenlik: acildurus'ta araç kımıldamaz).
# Reason /karar_decision'dan gelir; topic yoksa kaçış hiç tetiklenmez.
DUR_KACIS_TIME_S = 20.0          # s - engel-DUR'da bu kadar hareketsizlik → geri kaçış
DUR_KACIS_REASONS = ('engel_blokaj_reroute',  # RerouteKarar güvenlik-ağı dur'u
                     'muhur_statik_dur')      # karar mührünün statik-inişi (P0 №3)
DUR_KACIS_EVAL_GAP_S = 1.0       # s - DUR dalı bundan uzun ziyaret edilmediyse sayaç sıfırla
                                 #     (karar churn'ünün tek-tick flicker'ları sayacı BOZMAZ,
                                 #      gerçek sürüşe dönüş sıfırlar)

# --- Kaçış eskalasyonu (P0 №4, inceleme 2026-07-16 E6-O2) ---
# 2 m'lik kaçış aynı izi geri oynatıp aynı dar köşeye dönüyordu (deterministik
# takılma döngüsü). Aynı noktada ikinci tetikte mesafe 2→4 m'ye çıkar.
GERI_ESCAPE_ESKALASYON_M = 4.0        # m - aynı noktada 2. tetikte geri mesafe
GERI_ESCAPE_AYNI_NOKTA_M = 3.0        # m - önceki tetik bu yarıçap içindeyse "aynı nokta"
GERI_ESCAPE_ESKALASYON_PENCERE_S = 120.0  # s - bundan eski tetik "aynı nokta" sayılmaz (tur atıp dönme)

# --- Hız PID Kazançları (PIDPresets buradan okur; canlı değişiklik aktif PID'e uygulanır) ---
PID_SPEED_AGGRESSIVE = {'kp': 5.0, 'ki': 1.0, 'kd': 0.3}   # Hızlı tepki
PID_SPEED_NORMAL     = {'kp': 3.0, 'ki': 0.5, 'kd': 0.2}   # Dengeli (varsayılan mod)
PID_SPEED_SMOOTH     = {'kp': 2.0, 'ki': 0.3, 'kd': 0.1}   # Yumuşak
PID_SPEED_LOW_SPEED  = {'kp': 4.0, 'ki': 0.8, 'kd': 0.2}   # Düşük hız (park/manevra)
PID_ADAPT_VIRAJ      = {'kp': 3.0, 'ki': 0.5}              # adaptif: büyük heading hatası (viraj)
PID_ADAPT_YAKIN      = {'kp': 4.0, 'ki': 0.6}              # adaptif: hedefe yaklaşırken hassas kontrol
SPEED_PID_INTEGRAL_LIMIT = 10.0                            # hız PID integral clamp'i
SPEED_PID_DERIV_FILTER   = 0.3                             # hız PID türev filtresi (0-1)
ADAPTIVE_PID_ENABLED     = True                            # argparse --no-adaptive bunu kapatır
ADAPTIVE_HEADING_ESIK_DEG = 30.0                           # derece - bu heading hatası üstünde viraj kazancı

# =============================================================================
# CANLI PARAMETRELER (restart'sız ayar) — config/canli_params.yaml 'control:'
# bölümü yukarıdaki sabitleri ÇALIŞIRKEN override eder (~1 sn). Türetilmiş
# sabitler + canlı PID nesnesi _canli_degisiklik ile senkron tutulur.
# Bkz: talos_common/canli_params.py
# =============================================================================
_AKTIF_KONTROLCU = None  # CANWaypointFollower örneği (callback canlı PID'e ulaşsın diye)


def _canli_degisiklik(degisenler):
    """Canlı override sonrası türetilmiş sabitleri ve aktif nesneleri senkronla."""
    global MAX_SPEED_MS, LOOP_DT, ESTOP_BANT_YARIM_M
    if 'MAX_SPEED_KMH' in degisenler:
        MAX_SPEED_MS = MAX_SPEED_KMH / 3.6
    if 'LOOP_RATE_HZ' in degisenler:
        LOOP_DT = 1.0 / LOOP_RATE_HZ
    if 'ARAC_GENISLIK_M' in degisenler or 'ENGEL_YARICAP_M' in degisenler:
        ESTOP_BANT_YARIM_M = ARAC_GENISLIK_M / 2.0 + ENGEL_YARICAP_M
    inst = _AKTIF_KONTROLCU
    if inst is None:
        return
    if 'ADAPTIVE_PID_ENABLED' in degisenler:
        inst.adaptive_pid_enabled = ADAPTIVE_PID_ENABLED
    if 'ADAPTIVE_HEADING_ESIK_DEG' in degisenler:
        inst.heading_error_threshold = math.radians(ADAPTIVE_HEADING_ESIK_DEG)
    # Aktif modun preset'i değiştiyse kazançları canlı hız PID'ine bas
    # (adaptif mod bir sonraki _adapt_pid_gains çağrısında zaten günceli okur)
    preset_adi = 'PID_SPEED_' + inst.pid_mode.upper()
    if preset_adi in degisenler and isinstance(degisenler[preset_adi], dict):
        p = degisenler[preset_adi]
        inst.speed_pid.set_gains(kp=p.get('kp'), ki=p.get('ki'), kd=p.get('kd'))


try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle(
        'control', globals(), degisiklik_cb=_canli_degisiklik,
        sinirlar={
            'MAX_SPEED_KMH':        (0.5, 15.0),
            'LIMIT_SLOW':           (0.5, 10.0),
            'ESTOP_CRAWL_KMH':      (0.0, 5.0),
            'STEER_RATE_MAX_DEG_S': (30.0, 720.0),
            'GERI_MAX_THROTTLE':    (0.0, 100.0),
            'SLOWDOWN_BRAKE_MAX':   (0.0, 100.0),
            # Bee1 mekanik limiti ~28.95°; yanlış YAML değeri süpürme-alanı
            # geometrisini (R=L/tanδ) gerçek araçtan koparmasın (güvenlik
            # incelemesi 2026-07-15 — tam_kilit_donus_riskli tüketicisi eklendi)
            'MAX_STEER_ANGLE':      (20.0, 30.0),
            'SMAN_DONUS_TAIL_M':    (2.0, 15.0),
            'SMAN_DONUS_TEMIZ_S':   (0.1, 2.0),
            'KONI_HAFIZA_TTL_S':    (0.5, 10.0),
            # P0 kilit-kırıcı paket (inceleme 2026-07-16): DUR-kaçış bekleme
            # süresi çok kısaltılırsa meşru dur'larda erken geri kaçış riski
            'DUR_KACIS_TIME_S':          (5.0, 120.0),
            'GERI_ESCAPE_ESKALASYON_M':  (2.0, 8.0),
            # P1 №8 (E5-O1/O5): e-stop bırakma debounce'u + uzamsal histerezis
            'ESTOP_RELEASE_S':           (0.2, 3.0),
            'ESTOP_HISTEREZIS_M':        (0.0, 0.5),
        })
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[control.py] canli_params izleyicisi yok, statik parametreler: {_canli_e}",
          file=sys.stderr, flush=True)

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
                 integral_limit=5.0, derivative_filter=0.1, name="PID", angular=False):
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
        self.angular = angular  # Açısal ölçüm için yaw wrap normalizasyonu

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
            meas_diff = measurement - self.prev_measurement
            if self.angular:
                # Yaw ±π sınırında wrap-around spike'ını önle
                while meas_diff > math.pi:  meas_diff -= 2 * math.pi
                while meas_diff < -math.pi: meas_diff += 2 * math.pi
            raw_derivative = -meas_diff / self.dt
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
    """Farklı senaryolar için PID preset'leri.

    Hız preset'leri ÜST PARAMETRE BLOĞUNDA yaşar (PID_SPEED_*, canlı ayar);
    get_speed_preset her çağrıda modül sabitlerini okur → YAML override'ı
    çalışırken de etkilidir.
    """

    # Direksiyon kontrolü preset'leri (Pure Pursuit'e geçildi — kullanılmıyor)
    STEER_AGGRESSIVE = {'kp': 50.0, 'ki': 0.5, 'kd': 8.0}     # Keskin dönüşler
    STEER_NORMAL = {'kp': 40.0, 'ki': 0.0, 'kd': 5.0}         # Dengeli
    STEER_SMOOTH = {'kp': 30.0, 'ki': 0.0, 'kd': 3.0}         # Yumuşak
    STEER_LOW_SPEED = {'kp': 35.0, 'ki': 0.2, 'kd': 4.0}

    @staticmethod
    def get_speed_preset(mode='normal'):
        presets = {
            'aggressive': PID_SPEED_AGGRESSIVE,
            'normal': PID_SPEED_NORMAL,
            'smooth': PID_SPEED_SMOOTH,
            'low_speed': PID_SPEED_LOW_SPEED
        }
        return presets.get(mode, PID_SPEED_NORMAL)

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
        self.target_none_since = None      # dynamic_target None olduğu an (park tespiti için)

        # Gorev duragi takibi
        self.last_gorev_varildi_time = {}  # {durak_idx: timestamp} cooldown per durak
        self.completed_goreve = set()       # Kalıcı tamamlanan duraklar - bir daha tetiklenmez
        self.current_stop_waiting = False   # Durakta bekleme aktif mi
        self.current_stop_wait_start = 0.0  # Durakta bekleme baslangic zamani
        self.last_wp_varildi_time = 0.0     # Mikro-WP varildi throttle
        self.post_stop_time = 0.0           # Son durak bekleme bitiş zamanı (flip-flop grace)
        self._stall_start_time = None       # FIX 4: anti-stall kick-start timer

        # Karar durumu (H-A: control yanal-offset manevrası YAPMAZ → lane_change state YOK)
        self.karar = Karar.NORMAL
        self.last_karar_time = None   # /karar son geliş zamanı (watchdog; ilk karar gelince silahlanır)
        self._last_steer_source = "PURSUIT"  # direksiyon kaynağı geçiş logu için (PURSUIT / PURSUIT+LINE)
        self._prev_cmd_steer = 0.0           # son GERÇEKTEN gönderilen direksiyon (slew-rate + Ackermann e-stop referansı; §17/§incele)
        self._last_obstacle_ctx_t = 0.0      # son engel-bağlamı kararı (slow/sol/sag/dur/acil) zamanı → /line bastırma (§17)
        # --- KESKİN S-MANEVRA (slalom, §18) state ---
        self._sman_phase = 'IDLE'            # IDLE / TOWARD (tam sol) / AWAY (tam sağ)
        self._sman_dir = 0                   # +1 ilk dönüş sola, −1 sağa (reroute WP tarafı)
        self._sman_left_wp = None            # latched reroute (sol-şerit) WP dünya konumu (yanal hiza referansı)
        self._sman_start_xy = None           # manevra başlangıç konumu (yanal ilerleme ölçümü)
        self._sman_fwd_yaw = 0.0             # manevra başlangıç yön'ü (heading swing + nominal-çerçeve referansı)
        self._sman_phase_t = 0.0             # faz başlangıç zamanı (timeout)
        self._sman_pending = None            # (wp_x, wp_y, dir, t) — _hedef_callback'in tetik latch'i
        self._sman_clear_since = None        # güvenli-dönüş kapısı: süpürme alanı temiz olduğundan beri (debounce)
        self._sman_hold = False              # swing tavanında koni hâlâ riskli → DÜZ TUT alt-durumu
        self._sman_koni_izlendi = False      # bu manevrada en az bir koni kanıtı görüldü mü (erken-dönüş yetkisi)
        self._koni_hafiza = []               # [(wx, wy, t_son)] dünya-çerçevesi koni hafızası (_obstacle_lock altında)
        # H-B: /obstacles/poses tamponu (gövde çerçevesi) + tazelik + lock + e-stop latch
        self._obstacle_lock = threading.Lock()
        self._obstacle_points = []           # [(fwd, lat), ...] son PoseArray (gövde çerçevesi)
        self._obstacle_time = 0.0            # son PoseArray geliş zamanı (tazelik)
        self._estop_active = False           # latched e-stop durumu (donmada KORUNUR)
        self._estop_hard = False             # True ise HARD floor (<1.0m) → koşulsuz tam fren;
                                             #   False ise SOFT Ackermann-yay → emekle + S-manevra
        self._estop_clear_since = None       # koridor tazece temiz olduğu an (release debounce)

        # --- GERİ SÜRÜŞ (breadcrumb retrace) state ---
        self._breadcrumb = deque(maxlen=GERI_BREADCRUMB_MAXLEN)  # (steer_deg, throttle_pct) ileri iz; yalnız main-thread yazar
        self._geri_cmd = False        # /geri_komut son değeri (callback yazar)
        self._geri_prev_cmd = False   # rising-edge tespiti (doğal bitişte komut True kalsa da yeniden başlamasın)
        self.geri_mode = False        # geri sürüş oynatımı aktif mi
        self._geri_playback = None    # geri başlarken alınan iz snapshot'ı (liste)
        self._geri_index = 0          # snapshot içinde geriye yürüyen indeks
        self._geri_source = None      # 'manual' (/geri_komut, tam iz) | 'auto' (sıkışma kaçışı, mesafe-sınırlı)
        self._geri_dist_limit = None  # m - geri sürüş mesafe tavanı (None=tam iz); auto kaçışta GERI_ESCAPE_DISTANCE_M
        self._geri_start_xy = None    # geri sürüş başlangıç konumu (kat edilen mesafe ölçümü)
        # --- SIKIŞMA KAÇIŞI (stuck → 1 m geri) state ---
        self._stuck_since = None          # ilerlemeye çalışırken hareketsiz kalınan an (hız tabanlı)
        self._stuck_cooldown_until = 0.0  # bir kaçıştan sonra yeniden tetiklemeyi bu ana kadar beklet (thrash önleme)
        self._stuck_last_eval = None      # _stuck_check son değerlendirme anı (P0 №2 bayatlık koruması)
        # --- ENGEL-DUR KİLİDİ KAÇIŞI (P0 №1) state ---
        self.karar_reason = ""            # /karar_decision son reason (callback yazar; DUR-kaçış filtresi okur)
        self._dur_kacis_since = None      # engel-DUR'da hareketsiz kalınan an
        self._dur_kacis_last_eval = None  # DUR dalı son değerlendirme anı (flicker köprüsü / bayatlık)
        # --- KAÇIŞ ESKALASYONU (P0 №4) state ---
        self._geri_escape_last_xy = None  # son auto-kaçış tetik konumu
        self._geri_escape_last_t = 0.0    # son auto-kaçış tetik anı

        # Araç durumu
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed_ms = 0.0
        self.speed_kmh = 0.0

        # PID modu
        self.pid_mode = pid_mode
        speed_preset = PIDPresets.get_speed_preset(pid_mode)

        # PID Kontrolcüler
        self.speed_pid = PIDController(
            kp=speed_preset['kp'],
            ki=speed_preset['ki'],
            kd=speed_preset['kd'],
            output_min=0.0,
            output_max=100.0,
            integral_limit=SPEED_PID_INTEGRAL_LIMIT,
            derivative_filter=SPEED_PID_DERIV_FILTER,
            name="Speed"
        )

        # Direksiyon kontrolü artık Pure Pursuit (geometrik) — steer PID kaldırıldı.

        # Adaptif PID ayarları (üst blok; canlı değişiklik _canli_degisiklik ile yansır)
        self.adaptive_pid_enabled = ADAPTIVE_PID_ENABLED
        self.heading_error_threshold = math.radians(ADAPTIVE_HEADING_ESIK_DEG)

        # Durum
        self.is_running = True
        self.mission_complete = False
        self.parked = False              # mission-complete'te park() bir kez çağrılsın (el freni)
        self.mission_started = False
        self.autonomous_paused = False   # 0x500=0 -> manuel devralma; True iken bus'a frame yazma
        # manuel_baslat.sh bunu 1 yapar: başlatma-öncesi bus'a fren yazma, sustur.
        # Böylece direksiyon seti ile aynı anda açık durup buton 1'i (0x500=1) bekler.
        self.bus_release_on_start = os.environ.get("TALOS_BUS_RELEASE_ON_START", "0") == "1"

        # Canlı parametre callback'i bu örneğe ulaşsın (PID kazançları, adaptif eşikler)
        global _AKTIF_KONTROLCU
        _AKTIF_KONTROLCU = self

        # ROS Subscriber - Odometri
        self.odom_sub = rospy.Subscriber(
            '/base_pose_ground_truth',
            Odometry,
            self._odom_callback
        )

        # /gorev_durumu publisher - waypoint'e varış bildirimi
        self.pub_gorev = rospy.Publisher('/gorev_durumu', String, queue_size=10)

        # Hedef visualizer marker publisher
        self.pub_marker = rospy.Publisher('/hedef_marker', Marker, queue_size=10)

        # /hedef subscriber - dinamik hedef teslimi
        self.hedef_sub = rospy.Subscriber('/hedef', String, self._hedef_callback)

        # /karar subscriber - karar entegrasyonu
        self.karar_sub = rospy.Subscriber('/karar', String, self._karar_callback)

        # /karar_decision subscriber - karar'ın yapısal mesajı; yalnız reason
        # alanı tüketilir (engel-DUR kaçış filtresi, P0 №1). cart_sim.msg import
        # edilemediyse atlanır → kaçış devre dışı (fail-safe), sürüş etkilenmez.
        if DecisionMsg is not None:
            self.karar_decision_sub = rospy.Subscriber(
                '/karar_decision', DecisionMsg, self._karar_decision_callback)
        else:
            rospy.logwarn("cart_sim.msg.Decision import edilemedi - reason görünmez, "
                          "engel-DUR kaçışı (P0 №1) devre dışı")

        # /obstacles/poses subscriber - control'ün doğrudan engel kanalı.
        # İki tüketici: H-B e-stop güvenlik ağı (_update_estop) + S-manevra
        # güvenli-dönüş kapısı (_donus_alani_engeli; koni hafızasını da besler).
        self.obstacle_sub = rospy.Subscriber(
            OBSTACLE_TOPIC, PoseArray, self._obstacles_callback, queue_size=5)

        # /geri_komut subscriber - geri sürüş (breadcrumb retrace) tetiği.
        # Tamamen control-içi (karar/planlayıcı gerektirmez): True → geldiği rotayı geri izle.
        self.geri_sub = rospy.Subscriber('/geri_komut', Bool, self._geri_callback)

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
        """Hedef tesliminden gelen mikro-waypoint (String: 'x1,y1|x2,y2')

        D* Lite hedef yoneticisi 10Hz'de yuzlerce mikro-waypoint gonderir.
        Flip-flop filtresi: Hedef >3m ziplayip arkaya (>90°) isaret ederse
        (hedef_yoneticisi sapma recalculate kaynaklı) reddedilir.
        Ayrıca KESKİN S-MANEVRA tetiği: reroute WP yanal sıçraması (§18).
        """
        try:
            raw = msg.data.strip()

            if '|' in raw:
                segments = raw.split('|')
            else:
                segments = raw.split(';')

            parts = segments[0].split(',')
            x, y = float(parts[0]), float(parts[1])

            x2, y2 = None, None
            if len(segments) > 1:
                parts2 = segments[1].split(',')
                x2, y2 = float(parts2[0]), float(parts2[1])

            # --- FLIP-FLOP FİLTRESİ ---
            # Hedef >3m ziplayinca (§17: 5→3m): yon kontrolu yap
            # Aracin mevcut yonune gore >90° arkaya isaret ediyorsa REDDET
            # İstisna: Durak sonrası 3 saniye grace period - doğru yeni hedefleri kabul et
            if self.dynamic_target is not None:
                jump_dist = math.sqrt((x - self.dynamic_target[0])**2 + (y - self.dynamic_target[1])**2)
                # 5.0→3.0 m (§17): slalom reroute flip-flop'u ~3 m'lik yanal
                # sıçramalar üretiyor; eşik düştü ki >90° geriye işaret eden
                # flip-flop reversal'ları yakalansın. İleri WP ilerlemesi her
                # zaman <90° önde → reddedilmez (yalnız geriye-dönük sıçrama).
                if jump_dist > 3.0:
                    in_grace = (time.time() - self.post_stop_time) < 3.0
                    if not in_grace:
                        # Yeni hedefin aracin arkasinda olup olmadigini kontrol et
                        heading_to_new = math.atan2(y - self.y, x - self.x)
                        heading_diff = abs((heading_to_new - self.yaw + math.pi) % (2 * math.pi) - math.pi)
                        if heading_diff > math.radians(90):
                            # Arkaya/ters yonde ziplama - flip-flop, reddet
                            return

            old_target = self.dynamic_target
            self.dynamic_target = (x, y)
            if x2 is not None:
                self.next_target = (x2, y2)
            else:
                self.next_target = None
            self.target_none_since = None

            # --- KESKİN S-MANEVRA tetiği (§18): reroute WP YANAL sıçraması ---
            # Yeni WP eskisine göre gövde-çerçevesinde > eşik YANA adımladıysa =
            # planlayıcı karşı şeride reroute verdi (slalom). Latch'le; manevrayı
            # run loop (_sman_update) başlatır. (Tetik yalnız reroute WP — kullanıcı
            # Q2; /obstacles bağımlılığı YOK. İleri WP ilerlemesi ~0 yanal → tetiklemez.)
            # ENGEL-BAĞLAMI KAPISI: yalnız karar≠normal yakınken (cone var) → engelsiz
            # keskin track virajı yanlış S-manevra başlatmasın (§incele algo bulgusu).
            if (SMANEUVER_ENABLED and old_target is not None
                    and (time.time() - self._last_obstacle_ctx_t) < SMANEUVER_CTX_TTL):
                _, lat_new = self._body_frame(x, y)
                _, lat_old = self._body_frame(*old_target)
                lat_step = lat_new - lat_old
                if abs(lat_step) > SMANEUVER_REROUTE_LATERAL_M:
                    self._sman_pending = (x, y, 1 if lat_step > 0 else -1, time.time())

            # Sadece hedef belirgin degistiginde logla (spam onleme)
            if old_target is None or self._distance_between(old_target, (x, y)) > 2.0:
                self.logger.log(f"HEDEF: ({x:.2f}, {y:.2f})"
                                + (f" sonraki: ({x2:.2f}, {y2:.2f})" if self.next_target else ""))
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Hedef parse hatasi: {msg.data} - {e}")

    def _distance_between(self, p1, p2):
        """Iki nokta arasi mesafe"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _publish_hedef_markers(self):
        """Hedef noktalarini RViz'de gorsellestir"""
        if self.dynamic_target:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = rospy.Time.now()
            m.ns = "hedef"
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.dynamic_target[0]
            m.pose.position.y = self.dynamic_target[1]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.5
            m.color.r = 1.0; m.color.g = 1.0; m.color.a = 1.0  # Sari
            self.pub_marker.publish(m)

        if self.next_target:
            m2 = Marker()
            m2.header.frame_id = "map"
            m2.header.stamp = rospy.Time.now()
            m2.ns = "hedef"
            m2.id = 1
            m2.type = Marker.SPHERE
            m2.action = Marker.ADD
            m2.pose.position.x = self.next_target[0]
            m2.pose.position.y = self.next_target[1]
            m2.pose.orientation.w = 1.0
            m2.scale.x = m2.scale.y = m2.scale.z = 1.0
            m2.color.b = 1.0; m2.color.a = 1.0  # Mavi
            self.pub_marker.publish(m2)

    def _karar_callback(self, msg):
        """Karar node'undan gelen durum (String: 'normal'/'slow'/'dur'/'acildurus'/'sag'/'sol').

        H-A: control artık kendi yanal-offset manevrasını YAPMAZ. SAG/SOL yalnız
        bir HIZ ipucu (yavaş kal — engel yakını); rotadan SAPMA hedef planlayıcının
        işidir (karşı-şerit/slalom kenarları). Eski _start_lane_change/turn-guard
        zinciri kaldırıldı (iki canlı koşuda da engeli geçemedi, §12.12 / H-A)."""
        new_karar = msg.data.strip().lower()
        old_karar = self.karar
        now = time.time()
        self.last_karar_time = now   # watchdog: her karar mesajında tazele

        # Engel-bağlamı (normal DIŞI her karar) zaman damgası → /line bastırma (§17).
        # slalom içindeki kısa 'normal' blip'lerinde TTL ile /line susuk kalır.
        if new_karar != Karar.NORMAL:
            self._last_obstacle_ctx_t = now

        if new_karar != old_karar:
            if new_karar == Karar.DUR:
                self.logger.log("KARAR: DUR")
            elif new_karar == Karar.ACIL_DURUS:
                self.logger.log("KARAR: ACIL DURUS!")
            elif new_karar in (Karar.SAG, Karar.SOL):
                self.logger.log(f"KARAR: {new_karar.upper()} - engel yakını, yavaş düz takip "
                                f"(rota sapması planlayıcıda)")
            elif new_karar == Karar.SLOW:
                self.logger.log(f"KARAR: YAVAŞ - hız limiti {LIMIT_SLOW} km/h")
            elif new_karar == Karar.NORMAL:
                self.logger.log("KARAR: NORMAL")

        self.karar = new_karar

    def _karar_decision_callback(self, msg):
        """/karar_decision (cart_sim/Decision): yalnız reason alanı tüketilir —
        DUR-kaçış reason filtresi (P0 №1). Karar string'inin kaynağı /karar
        olmaya devam eder (iki topic aynı publish döngüsünden ~10 Hz gelir)."""
        self.karar_reason = (msg.reason or "").strip()

    def _odom_callback(self, msg):
        """Odometri callback"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Yaw açısı
        # (Eski yaw_rate hesabı kaldırıldı — tek tüketicisi şerit-değiştirme
        # pre-start guard'ıydı, o da H-A ile söküldü; ölü state tutulmuyor.)
        q = msg.pose.pose.orientation
        _, _, new_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
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
    # C1 — DOĞRUDAN ENGEL KANALI (/obstacles/poses)
    # =========================================================================

    def _obstacles_callback(self, msg):
        """`/obstacles/poses` (PoseArray, lidar çerçevesi: x ileri / y sol) →
        tampon + tazelik damgası + DÜNYA koni hafızası.

        Boş array de geçerli (engel yok). E-stop kararını `_update_estop()` verir.

        DÜNYA KONİ HAFIZASI (güvenli-dönüş kapısı için): detektör kareyi
        düşürünce koni tampondan kaybolur → kapı 'alan temiz' sanıp ERKEN dönüş
        başlatabilir. Koniler statiktir: görülen merkezler dünya çerçevesinde
        KONI_HAFIZA_TTL_S boyunca latch'lenir; aynı koninin yeni tespiti
        (KONI_HAFIZA_ESLE_M içinde) kaydı tazeler, görünmeyenler TTL'de düşer."""
        pts = [(p.position.x, p.position.y) for p in msg.poses]  # (fwd, lat) lidar çerçevesi
        now = time.time()
        # Pozu callback anında örnekle (lidar → base_link: +LIDAR_BASE ileri)
        cx, cy_, cyaw = self.x, self.y, self.yaw
        c, s = math.cos(cyaw), math.sin(cyaw)
        lidar_base = LIDAR_ARKA_AKS_M - BASE_ARKA_AKS_M   # ≈0.9 m
        # Hafızaya yalnız GÖVDE DIŞI noktalar girer (run 092553Z: detektör
        # araç-üstü nokta basıyor → dünya hafızasında araçla yürüyen hayalet).
        # _obstacle_points HAM kalır: e-stop kendi FWD_MIN filtresini uygular.
        harici = govde_disi_filtre(
            [(fwd + LIDAR_ARKA_AKS_M, lat) for fwd, lat in pts],
            ARAC_ARKA_M, ARAC_BURUN_M, ARAC_GENISLIK_M / 2.0)
        yeni = [(cx + c * (f_ra - BASE_ARKA_AKS_M) - s * lat,
                 cy_ + s * (f_ra - BASE_ARKA_AKS_M) + c * lat, now)
                for f_ra, lat in harici]
        with self._obstacle_lock:
            self._obstacle_points = pts
            self._obstacle_time = now
            for (wx, wy, t) in self._koni_hafiza:
                if now - t > KONI_HAFIZA_TTL_S:
                    continue   # süresi doldu
                if any(math.hypot(wx - nx, wy - ny) < KONI_HAFIZA_ESLE_M
                       for nx, ny, _ in yeni):
                    continue   # aynı koni tazece görüldü (yeni kayıt zaten listede)
                yeni.append((wx, wy, t))
            self._koni_hafiza = yeni[:KONI_HAFIZA_MAX]

    def _update_estop(self):
        """H-B: doğrudan LATCHED + Ackermann-farkında e-stop güvenlik ağı.

        İki kademe:
          (1) HARD FLOOR: çok yakın (ESTOP_HARD_M) + dar koridor → KOŞULSUZ fren
              (bu mesafede direksiyon kurtaramaz; son çare).
          (2) ACKERMANN: ESTOP_FWD_M içinde aday engel varsa, mevcut direksiyonun
              2B SÜPÜRME BANDI (araç genişliği + koni yarıçapı, arka-aks referanslı
              halka; bkz. ackermann_path_clears) engeli içine alıyorsa fren. Bandın
              dışındaki koni DURDURMAZ → "dubaya çarpacak sanıp erken durma" çözülür
              (kullanıcı isteği, CANLI run 185215 + 2026-07-04 genişlik/ofset revizyonu).
        Latched: tampon donarsa (stale) durum KORUNUR (plan §3.4). Bırakma: tazece
        ESTOP_RELEASE_S temiz kalınca. Döner: True ise bu tick TAM FREN."""
        now = time.time()
        with self._obstacle_lock:
            stale = (now - self._obstacle_time) > OBSTACLE_TIMEOUT
            pts = list(self._obstacle_points)
        if stale:
            # Veri donmuş: latch'i KORU. Mesafeyi artık doğrulayamayız → SOFT iken
            # bile güvenli tarafa kaç (tam fren), emekleyerek köre yaklaşma.
            if self._estop_active:
                self._estop_hard = True
            return self._estop_active

        reason = None
        hard = select_blocking_obstacle(pts, OBSTACLE_FWD_MIN, ESTOP_HARD_M, ESTOP_CORRIDOR_M)
        if hard is not None:
            reason = f"hard floor {hard[0]:.1f}m (yanal {hard[1]:+.2f}m)"
        else:
            # Ackermann yay'ı GERÇEKTEN gönderilen son direksiyona göre değerlendir
            # (`_prev_cmd_steer`, _send_can_command'da her tick tazelenir). Eski
            # `_last_steer_deg` yalnız SÜRÜŞ dalında güncellenirdi → e-stop sürerken
            # donmuş pre-e-stop steer'le (ör. +28°) yay "yol açık" deyip fren'i erken
            # bırakabiliyordu; oysa gerçek komut steer=0 idi (güvenlik bulgusu, §incele).
            # 2026-07-04: TÜM koridor adayları yay testinden geçirilir (en-yakın-aday
            # gölgeleme düzeltmesi — bkz. select_arc_blocking_obstacle docstring).
            cand = select_arc_blocking_obstacle(
                pts, OBSTACLE_FWD_MIN, ESTOP_FWD_M, ESTOP_CHECK_CORRIDOR_M,
                self._prev_cmd_steer, WHEELBASE, ESTOP_BANT_YARIM_M,
                LIDAR_ARKA_AKS_M, ARAC_BURUN_M)
            if cand is not None:
                reason = (f"Ackermann yay çarpıyor: engel {cand[0]:.1f}m "
                          f"(yanal {cand[1]:+.2f}m), steer {self._prev_cmd_steer:+.1f}°")

        if reason is not None:
            self._estop_hard = (hard is not None)   # HARD floor mu, SOFT yay mı?
            if not self._estop_active:
                kademe = "TAM FREN" if self._estop_hard else "YAVAŞLA+MANEVRA"
                self.logger.log(f"[E-STOP] {reason} — {kademe}")
            self._estop_active = True
            self._estop_clear_since = None
        elif self._estop_active:
            # Yol açık (yay geçiyor / koridor temiz) — debounce sonra bırak.
            # P1 №8 (E5-O1): bırakma, +ESTOP_HISTEREZIS_M genişletilmiş bandın da
            # temiz olmasını ister — bant kenarındaki engelde jitter kaynaklı
            # tetik↔bırak limit çevrimini keser (tetik yolu değişmedi).
            cand_h = select_arc_blocking_obstacle(
                pts, OBSTACLE_FWD_MIN, ESTOP_FWD_M, ESTOP_CHECK_CORRIDOR_M,
                self._prev_cmd_steer, WHEELBASE,
                ESTOP_BANT_YARIM_M + ESTOP_HISTEREZIS_M,
                LIDAR_ARKA_AKS_M, ARAC_BURUN_M)
            if cand_h is not None:
                self._estop_clear_since = None   # histerezis bandında hâlâ engel
            elif self._estop_clear_since is None:
                self._estop_clear_since = now
            elif now - self._estop_clear_since >= ESTOP_RELEASE_S:
                self._estop_active = False
                self._estop_clear_since = None
                self.logger.log("[E-STOP] Yol açık — fren bırakıldı")
        return self._estop_active

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
            return LIMIT_SLOW  # engel yakınında (sag/sol/slow) yavaş düz takip
        else:
            return MAX_SPEED_KMH

    def _check_gorev_arrival(self):
        """Robot konumunu gercek gorev duraklarina karsi kontrol et.

        Mikro-waypoint'lere degil, sadece GOREV_NOKTALARI'ndaki duraklara
        varildiginda 'varildi' gonderir.

        Returns:
            True = durakta bekliyor (fren uygula), False = normal surus
        """
        now = time.time()

        # Durakta bekleme aktifse
        if self.current_stop_waiting:
            elapsed = now - self.current_stop_wait_start
            if elapsed < DUR_WAIT_TIME:
                # Hala bekliyoruz
                return True
            else:
                # Bekleme bitti, normal suruse don
                self.current_stop_waiting = False
                self.speed_pid.reset()
                self.post_stop_time = time.time()
                self.logger.log("Durak beklemesi bitti, PID sifirlandi, devam ediliyor")
                return False

        # Her duraga mesafe kontrol et
        for idx, (gx, gy) in enumerate(GOREV_NOKTALARI):
            # Tamamlanan durakları kalıcı olarak atla
            if idx in self.completed_goreve:
                continue
            dist = self._distance_to(gx, gy)
            if dist < GOREV_THRESHOLD:
                # Cooldown kontrolu - ayni duraga tekrar varildi gondermeyi engelle
                last_time = self.last_gorev_varildi_time.get(idx, 0.0)
                if now - last_time < GOREV_COOLDOWN:
                    continue

                # Duraga vardik! Kalıcı olarak tamamlandı işaretle
                self.completed_goreve.add(idx)
                self.last_gorev_varildi_time[idx] = now
                self.logger.log(f"GOREV DURAGI #{idx+1} VARILDI: ({gx:.1f}, {gy:.1f}) mesafe={dist:.1f}m")
                # NOT: varildi mesaji buradan gonderilmez; yalnız run()'daki mikro-WP varış dalı gönderir.
                # Cift varildi hedef_yoneticisi'nde durak atlama bugina neden oluyordu.

                # Son durak mi?
                if idx == len(GOREV_NOKTALARI) - 1:
                    self.mission_complete = True
                    self.logger.log("SON GOREV DURAGI - GOREV TAMAMLANDI!")

                # Durakta kisa bekleme
                self.current_stop_waiting = True
                self.current_stop_wait_start = now
                return True

        return False

    def _nearest_gorev_distance(self):
        """En yakin **tamamlanmamış** gorev duragina mesafe (yavaslamak icin).
        FIX 2-reviewer: completed_goreve filtresi yoktu — duragı geçtikten
        sonra bile o duraga olan mesafe min olarak hesaplanıyordu → SLOWDOWN
        devam ediyordu → araç çıkamıyordu. Şimdi sadece henüz varılmamış
        durakları hesaba katar."""
        min_dist = float('inf')
        for idx, (gx, gy) in enumerate(GOREV_NOKTALARI):
            if idx in self.completed_goreve:
                continue
            dist = self._distance_to(gx, gy)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    # =========================================================================
    # PID VE SÜRÜŞ YARDIMCI FONKSİYONLARI
    # =========================================================================

    def _adapt_pid_gains(self, heading_error, distance):
        """
        Hız PID kazançlarını duruma göre adapte et

        - Büyük açı hatası: Hızı düşür (virajda yavaşla)
        - Küçük mesafe: Daha hassas kontrol

        Not: Direksiyon kontrolü Pure Pursuit ile yapıldığı için burada
        yalnızca hız PID'i adapte edilir.
        """
        if not self.adaptive_pid_enabled:
            return

        abs_heading_error = abs(heading_error)

        # === HIZ PID ADAPTASYONU ===
        if abs_heading_error > self.heading_error_threshold:
            # Büyük açı hatası - hızı biraz düşür ama durma (virajda hız lazım)
            self.speed_pid.set_gains(kp=PID_ADAPT_VIRAJ.get('kp', 3.0), ki=PID_ADAPT_VIRAJ.get('ki', 0.5))
        elif distance < SLOWDOWN_DISTANCE:
            # Yaklaşıyoruz - hassas kontrol
            self.speed_pid.set_gains(kp=PID_ADAPT_YAKIN.get('kp', 4.0), ki=PID_ADAPT_YAKIN.get('ki', 0.6))
        else:
            # Normal mod
            preset = PIDPresets.get_speed_preset(self.pid_mode)
            self.speed_pid.set_gains(kp=preset['kp'], ki=preset['ki'])

    def _can_listener(self):
        """CAN mesajlarını okuyan arka plan thread'i"""
        while self.is_running and not rospy.is_shutdown():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    if msg.arbitration_id == 0x500:
                        # Sistem Komutları (Byte 0: 1=Start/Devam, 0=Durdur/Manuel devral)
                        if msg.data[0] == 1:
                            if not self.mission_started:
                                self.mission_started = True
                                self.logger.log(">>> CAN Başlatma komutu alındı (0x500) <<<")
                            elif self.autonomous_paused:
                                self.autonomous_paused = False
                                self.logger.log(">>> CAN Devam komutu (0x500=1) - otonom devraldı <<<")
                        elif msg.data[0] == 0 and self.mission_started and not self.autonomous_paused:
                            self.autonomous_paused = True
                            self.logger.log(">>> CAN Durdurma komutu (0x500=0) - manuel devraldı, bus serbest <<<")

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
            steer_clamped = float(np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE))
            # Slew-rate limiti referansını GERÇEK gönderilen değere senkronla (§17):
            # tüm dallar (sürüş + e-stop/dur/acil = steer 0) buradan geçer → bir
            # sonraki tick'in rampası fiziksel direksiyon konumundan başlar.
            self._prev_cmd_steer = steer_clamped
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

    def _select_lookahead_point(self, primary, secondary, ld):
        """Pure Pursuit lookahead noktasını seç.

        Araç merkezli ld yarıçaplı çember ile primary->secondary
        segmentinin kesişimini bulur; böylece lookahead noktası ayrık
        waypoint'ler arasında da ld mesafesinde kalır.

        primary   : mevcut hedef mikro-waypoint (x, y)
        secondary : bir sonraki waypoint (x, y) veya None
        ld        : istenen lookahead mesafesi (metre)
        """
        # primary zaten yeterince uzaktaysa ya da uzatacak nokta yoksa
        if self._distance_to(*primary) >= ld or secondary is None:
            return primary

        ax, ay = primary
        bx, by = secondary
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-6:
            return primary

        # |A + t*(B-A) - C|^2 = ld^2  ->  a*t^2 + b*t + c = 0
        fx, fy = ax - self.x, ay - self.y
        a = seg_len_sq
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - ld * ld
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            # Segment tamamen çemberin içinde - en uzak ucu kullan
            return secondary
        disc = math.sqrt(disc)
        t = (-b + disc) / (2.0 * a)  # ileri yöndeki (büyük) kök
        if t <= 0.0:
            return primary
        if t >= 1.0:
            return secondary
        return (ax + t * dx, ay + t * dy)

    def _body_frame(self, px, py):
        """Dünya noktasını araç gövde çerçevesine çevir → (fwd ileri, lat sol)."""
        dx = px - self.x
        dy = py - self.y
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _sman_swing_deg(self):
        """Manevra başlangıç yönüne göre işaretli heading swing (derece, dir yönünde +)."""
        d = self.yaw - self._sman_fwd_yaw
        while d > math.pi:  d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return math.degrees(self._sman_dir * d)

    def _sman_lateral_progress(self):
        """Başlangıç-yönü (nominal) çerçevesinde aracın dir yönünde kat ettiği yanal mesafe (m)."""
        sx, sy = self._sman_start_xy
        dx, dy = self.x - sx, self.y - sy
        lat = -math.sin(self._sman_fwd_yaw) * dx + math.cos(self._sman_fwd_yaw) * dy
        return self._sman_dir * lat

    def _sman_wp_lateral(self):
        """Latched sol-WP'nin nominal çerçevedeki dir-yönlü yanal offset'i (m) — faz1 hedefi."""
        sx, sy = self._sman_start_xy
        dx, dy = self._sman_left_wp[0] - sx, self._sman_left_wp[1] - sy
        lat = -math.sin(self._sman_fwd_yaw) * dx + math.cos(self._sman_fwd_yaw) * dy
        return self._sman_dir * lat

    def _sman_donus_yanal_kazanc(self, swing_deg):
        """Tam-kilit dönüş (AWAY) heading'i ALIGN'a getirene dek kazanılacak EK
        yanal sapma (m): ΔL = R·(cos(ALIGN) − cos(swing)). Dönüş boyunca heading
        hep dış tarafa baktığından sapma ARTMAYA devam eder — koridor sınırı bu
        kaçınılmaz kazancı ÖNGÖREREK dönüşü erken zorlar (şartname şerit ihlali)."""
        R = WHEELBASE / math.tan(math.radians(MAX_STEER_ANGLE))
        a = math.radians(SMANEUVER_ALIGN_DEG)
        s = math.radians(max(swing_deg, SMANEUVER_ALIGN_DEG))
        return R * (math.cos(a) - math.cos(s))

    def _donus_alani_engeli(self, swing_deg, donus_yonu):
        """GÜVENLİ DÖNÜŞ KAPISI: araç şimdi tam-kilit donus_yonu dönüşüne başlasa
        süpüreceği alanda (yay + eski-şerit koridoru) engel var mı?

        Kaynaklar: taze /obstacles/poses (lidar çerçevesi) + dünya koni hafızası
        (dropout köprüsü); ikisi de ARKA AKS çerçevesine çevrilip
        tam_kilit_donus_riskli'ye verilir. PARK/DURAK manevra dönüşleri de aynı
        kapıyı kullanmalı (swing/yon parametrik — manevraya özgü değil).

        Döner: (engel | None, gorgu). gorgu=True yalnız POZİTİF kanıt varken
        (şu an en az bir nokta canlıda/hafızada). "Tampon taze ama BOŞ" gorgu
        DEĞİLDİR — canlı-ama-kör detektör (koni var, hiç tespit yok) "yol kesin
        temiz" diye yorumlanmasın (güvenlik incelemesi 2026-07-15: yokluk
        kanıtı ≠ kanıt yokluğu); erken-dönüş yetkisini çağıran, manevra
        boyunca en az bir kez gorgu görmüş olmaya bağlar."""
        now = time.time()
        with self._obstacle_lock:
            taze = (now - self._obstacle_time) <= OBSTACLE_TIMEOUT
            pts = list(self._obstacle_points) if taze else []
            hafiza = [(wx, wy) for (wx, wy, t) in self._koni_hafiza
                      if now - t <= KONI_HAFIZA_TTL_S]
        points_ra = [(fwd + LIDAR_ARKA_AKS_M, lat) for fwd, lat in pts]
        for wx, wy in hafiza:
            bf, bl = self._body_frame(wx, wy)
            points_ra.append((bf + BASE_ARKA_AKS_M, bl))
        # Kendi gövde noktalarını ele (run 092553Z: detektör araç-üstü nokta
        # basıyor → alan hiç temizlenmiyordu; bkz. govde_disi_filtre docstring)
        points_ra = govde_disi_filtre(points_ra, ARAC_ARKA_M, ARAC_BURUN_M,
                                      ARAC_GENISLIK_M / 2.0)
        if not points_ra:
            return None, False
        engel = tam_kilit_donus_riskli(
            points_ra, swing_deg, donus_yonu, MAX_STEER_ANGLE, WHEELBASE,
            ESTOP_BANT_YARIM_M + SMAN_DONUS_KLIRENS_M, ARAC_BURUN_M,
            ARAC_ARKA_M, SMAN_DONUS_TAIL_M)
        return engel, True

    def _sman_update(self):
        """KESKİN S-MANEVRA durum makinesi (§18). Aktifse direksiyon (derece)
        döner ve steering'i SAHİPLENİR (pursuit/​/line/sharp-gate bypass); IDLE
        ise None döner (pursuit devralır). Faz geçişleri latched WP + nominal
        çerçeveden türer; canlı `target` KULLANILMAZ (flip-flop-bağımsızlık).

        Akış (kullanıcı tarifi + golf-cart geometri düzeltmesi):
          IDLE   → reroute WP tetiği taze + manevra kapalı → TAM SOL (TOWARD)
          TOWARD → steer = dir·MAX (TAM SOL); direksiyon TOPLANMAZ. Çıkış
                   önceliği (şartname s.8, run 092553Z eleme dersi):
                   (1) KORİDOR SINIRI — yanal sapma + dönüş yayının ek kazancı
                       WP hizası + SMAN_KORIDOR_ASIM_M'i aşacaksa koni ne olursa
                       olsun → AWAY (yoldan çıkma/şerit ihlali > koni klirensi;
                       e-stop emekleme çarpmayı önler).
                   (2) GÜVENLİ DÖNÜŞ KAPISI — koni, tam-kilit ters dönüşün
                       süpürme alanından çıkıp SMAN_DONUS_TEMIZ_S temiz kalınca
                       → AWAY (erken, koni-farkında dönüş).
                   (3) Swing SWING_DEG tavanı: koni riskli değilse → AWAY;
                       riskliyse koridor içinde DÜZ TUT (steer 0; sapma dönerek
                       artmaz, koni geçilir; sınıra dayanınca (1) devralır).
          AWAY   → steer = −dir·MAX (TAM SAĞ, ikinci keskin dönüş); heading
                   başlangıç-yönüne ALIGN_DEG'e dönünce → IDLE (pursuit devralır)
        Her fazda TIMEOUT güvenliği. Latched WP + nominal çerçeve sayesinde hedef
        flip-flop'undan (§17 mek-3) bağımsız — manevra kendi geometrisini sürer."""
        if not SMANEUVER_ENABLED:
            return None
        now = time.time()

        if self._sman_phase == 'IDLE':
            # Atomik al-ve-temizle (TOCTOU): callback thread aynı anda yeni tetik
            # yazarsa kaybolmasın diye tek ifadede swap (ROS bulgusu, §incele).
            p, self._sman_pending = self._sman_pending, None
            if p is not None and (now - p[3]) < SMANEUVER_PENDING_TTL:
                self._sman_dir = p[2]
                self._sman_left_wp = (p[0], p[1])
                self._sman_start_xy = (self.x, self.y)   # nominal çerçeve sıfırı
                self._sman_fwd_yaw = self.yaw            # başlangıç yönü (dönmeden ÖNCE)
                self._sman_phase = 'TOWARD'
                self._sman_phase_t = now
                self._sman_clear_since = None            # güvenli-dönüş kapısı debounce sıfırı
                self._sman_hold = False                  # düz-tut alt-durumu sıfırı
                self._sman_koni_izlendi = False          # koni-kanıtı bayrağı sıfırı
                yon = "SOL" if p[2] > 0 else "SAĞ"
                self.logger.log(f"S-MANEVRA BAŞLADI: TAM {yon} (reroute WP "
                                f"({p[0]:.1f},{p[1]:.1f}), yanal hiza={self._sman_wp_lateral():.1f}m'e "
                                f"veya swing {SMANEUVER_MAX_SWING_DEG:.0f}°'e kadar tut)")
                return self._sman_dir * MAX_STEER_ANGLE
            return None

        # Süre güvenliği — takılırsa pursuit'e dön
        if (now - self._sman_phase_t) > SMANEUVER_TIMEOUT:
            self.logger.log(f"S-MANEVRA İPTAL: {self._sman_phase} timeout "
                            f"({SMANEUVER_TIMEOUT:.0f}s) → pursuit")
            self._sman_phase = 'IDLE'
            self._sman_hold = False
            return None

        if self._sman_phase == 'TOWARD':
            # ── GÜVENLİ DÖNÜŞ KAPISI + KORİDOR SINIRI (2026-07-15) ──────────
            # Öncelik sırası (şartname s.8 — yoldan çıkma/şerit ihlali > koni):
            #   1) KORİDOR SINIRI: yanal sapma + dönüş yayının kaçınılmaz ek
            #      kazancı WP hizası + ASIM'ı aşacaksa → koni ne olursa olsun
            #      DÖNÜŞ ZORLANIR (e-stop emekleme çarpmayı önler). Run 092553Z
            #      dersi: sınırsız bekletme aracı yoldan çıkardı → ELEME.
            #   2) Kapı: koni süpürme alanından çıktıysa (debounce) erken dön.
            #   3) Swing tavanı: koni riskli DEĞİLSE dön; riskliyse koridor
            #      içinde kaldığı sürece DÜZ TUT (sapma artmaz, koni geçilir).
            swing = self._sman_swing_deg()
            swing_capped = swing >= SMANEUVER_MAX_SWING_DEG
            koridor_siniri = (self._sman_lateral_progress()
                              + self._sman_donus_yanal_kazanc(swing)
                              >= self._sman_wp_lateral() + SMAN_KORIDOR_ASIM_M)
            donus_hazir = False
            neden = None
            if SMAN_DONUS_KAPISI_AKTIF:
                engel, gorgu = self._donus_alani_engeli(swing, -self._sman_dir)
            else:
                engel, gorgu = None, False
            if gorgu:
                self._sman_koni_izlendi = True   # bu manevrada pozitif koni kanıtı var
            koni_riskli = False
            if engel is not None:
                koni_riskli = True
                self._sman_clear_since = None
                rospy.loginfo_throttle(
                    1.0, f"[S-MANEVRA] dönüş bekletiliyor: koni tam-kilit "
                         f"süpürme alanında (ileri {engel[0]:.1f}m, "
                         f"yanal {engel[1]:+.1f}m)")
            elif self._sman_koni_izlendi:
                # Alan temiz VE elimizde koni kanıtı var(dı) → debounce sonrası dön.
                if self._sman_clear_since is None:
                    self._sman_clear_since = now
                if now - self._sman_clear_since >= SMAN_DONUS_TEMIZ_S:
                    donus_hazir = True
                    neden = "koni süpürme alanından çıktı"
            if koridor_siniri or donus_hazir or (swing_capped and not koni_riskli):
                self._sman_phase = 'AWAY'
                self._sman_phase_t = now
                self._sman_hold = False
                yon = "SAĞ" if self._sman_dir > 0 else "SOL"
                if koridor_siniri:
                    neden = ("koridor sınırı — WP hizası aşılmadan dönüş "
                             "(şerit ihlali önleme)"
                             + (" [koni hâlâ süpürmede, e-stop korur]"
                                if koni_riskli else ""))
                elif not donus_hazir:
                    neden = f"swing {SMANEUVER_MAX_SWING_DEG:.0f}° (backstop)"
                self.logger.log(f"S-MANEVRA: {neden} → TAM {yon} (dönüş)")
                return -self._sman_dir * MAX_STEER_ANGLE
            if swing_capped and koni_riskli:
                # DÜZ TUT: sapmayı döndürerek artırma (döngü-önleme) ama koni
                # süpürme alanından çıkmadan da DÖNME — heading sabit ilerle.
                # KORİDOR SINIRLI: yanal sapma yukarıdaki sınıra dayanınca bu
                # dal artık seçilmez (dönüş zorlanır). phase_t bir kez tazelenir
                # (toplam backstop ≤ 2×SMANEUVER_TIMEOUT, sonrası pursuit).
                if not self._sman_hold:
                    self._sman_hold = True
                    self._sman_phase_t = now
                    self.logger.log(
                        f"S-MANEVRA: swing {SMANEUVER_MAX_SWING_DEG:.0f}° tavanı "
                        f"+ koni süpürmede → DÜZ TUT (koridor sınırına kadar)")
                return 0.0
            return self._sman_dir * MAX_STEER_ANGLE

        # AWAY (TAM SAĞ): heading başlangıç-yönüne düzelince pursuit'e bırak
        if self._sman_swing_deg() <= SMANEUVER_ALIGN_DEG:
            self.logger.log("S-MANEVRA BİTTİ: heading düzeldi → pursuit devraldı")
            self._sman_phase = 'IDLE'
            self._sman_hold = False
            return None
        # Geçiş sonrası KÖR OLMA (algoritma incelemesi 2026-07-15): kalan dönüş
        # süpürmesi her tick denetlenir — yeni/yeniden görünen koni kalan yaya
        # girerse dönüşü DURAKLAT (düz tut; hız run()'da LIMIT_SLOW'a iner),
        # temizlenince sür. AWAY phase_t TAZELENMEZ → 6s timeout'u strict kalır.
        # KORİDOR SINIRLI: duraklama da yanal sapmayı büyütür — sınıra dayanınca
        # duraklamak YASAK, dönüş sürer (şerit ihlali > koni; e-stop korur).
        if SMAN_DONUS_KAPISI_AKTIF:
            away_swing = max(0.0, self._sman_swing_deg())
            koridor_izni = (self._sman_lateral_progress()
                            + self._sman_donus_yanal_kazanc(away_swing)
                            < self._sman_wp_lateral() + SMAN_KORIDOR_ASIM_M)
            engel, _ = self._donus_alani_engeli(away_swing, -self._sman_dir)
            if engel is not None and koridor_izni:
                if not self._sman_hold:
                    self._sman_hold = True
                    self.logger.log("S-MANEVRA: AWAY duraklatıldı — koni kalan "
                                    "dönüş süpürmesinde (düz tut)")
                rospy.loginfo_throttle(
                    1.0, f"[S-MANEVRA] AWAY beklemede: koni süpürmede "
                         f"(ileri {engel[0]:.1f}m, yanal {engel[1]:+.1f}m)")
                return 0.0
            if self._sman_hold:
                self._sman_hold = False
                self.logger.log("S-MANEVRA: AWAY sürüyor — "
                                + ("süpürme temizlendi" if engel is None
                                   else "koridor sınırı (bekleme yasak, e-stop korur)"))
        return -self._sman_dir * MAX_STEER_ANGLE

    def _pure_pursuit_steer(self, primary, secondary):
        """Pure Pursuit geometrik direksiyon kontrolü (düz takip — H-A).

            delta = atan2(2 * L * sin(alpha), Ld)

        L     : dingil mesafesi (WHEELBASE)
        alpha : araç yönü ile lookahead noktası arasındaki açı
        Ld    : araçtan lookahead noktasına gerçek mesafe

        Lookahead mesafesi hıza göre uyarlanır: Ld = K*v + B (eski KTR raporu).
        Düşük hızda kısa, yüksek hızda uzun lookahead -> salınımsız takip.

        H-A: sentetik yanal-offset kaçınma KALDIRILDI (kör manevra engeli geçemedi,
        §12.12). Kaçınma artık planlayıcıda; control rotayı düz takip eder.

        KESKİN DÖNÜŞ (§12.14): waypoint'e açı büyükse (reroute) lookahead'i KISALT →
        daha sert direksiyon → reroute'u tam takip eder (CANLI: uzun lookahead ile
        reroute dönüşü 12°'de yumuşak kalıp duba koridordan çıkmıyordu).

        Dönüş: direksiyon açısı (derece, + sol / - sağ).
        """
        ld_desired = float(np.clip(
            LOOKAHEAD_K * self.speed_ms + LOOKAHEAD_B,
            LOOKAHEAD_MIN, LOOKAHEAD_MAX
        ))
        lx, ly = self._select_lookahead_point(primary, secondary, ld_desired)
        alpha = self._heading_error(lx, ly)

        # Keskin dönüş gerekiyorsa (lookahead noktasına büyük açı) lookahead'i kısalt
        # ve yeniden seç → çok daha sert direksiyon (reroute'u keskin takip).
        if abs(alpha) > math.radians(SHARP_TURN_DEG) and ld_desired > SHARP_LOOKAHEAD:
            lx, ly = self._select_lookahead_point(primary, secondary, SHARP_LOOKAHEAD)
            alpha = self._heading_error(lx, ly)

        ld_actual = max(self._distance_to(lx, ly), LOOKAHEAD_MIN)
        delta_rad = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld_actual)
        return math.degrees(delta_rad)

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

    def _geri_callback(self, msg):
        """/geri_komut (std_msgs/Bool): True → geri sürüş (breadcrumb retrace) iste.
        Yalnız bool'u set eder; breadcrumb'a DOKUNMAZ — snapshot main-thread'de
        run() içinde alınır (deque'e ileri sürüş de yazdığı için thread güvenliği)."""
        self._geri_cmd = bool(msg.data)

    def _geri_replay_step(self):
        """Geri sürüş: kaydedilen ileri izini ters sırayla bir adım ilerlet.
        Dönüş: (steer_deg, throttle_pct, done). İz bittiğinde (0.0, 0.0, True)."""
        if not self._geri_playback or self._geri_index <= 0:
            return 0.0, 0.0, True
        self._geri_index -= 1
        steer_deg, throttle_pct = self._geri_playback[self._geri_index]
        # Vites REVERSE'te aynı gaz → aynı hız profili geri (retrace sadakati).
        # Güvenlik tavanı: düşürmek geri sürüşü yavaşlatır ama gaz-vs-tick
        # eşleşmesini bozacağından retrace'i hafif saptırır (TUNABLE).
        throttle_pct = float(min(throttle_pct, GERI_MAX_THROTTLE))
        # Direksiyon İŞARETİ AYNI — negatifleme YOK (aynı yay geri çizilir).
        return steer_deg, throttle_pct, False

    def _start_geri(self, dist_limit, source):
        """Geri sürüşü başlat. Breadcrumb snapshot'ı + mesafe ölçüm referansı burada
        (main-thread) alınır. dist_limit=None → tam iz (manuel); değer → o kadar metre
        geri (sıkışma kaçışı). Kayıt sürer ama biz donmuş kopyayı yürütürüz."""
        self._geri_playback = list(self._breadcrumb)
        self._geri_index = len(self._geri_playback)
        self._geri_dist_limit = dist_limit
        self._geri_start_xy = (self.x, self.y)
        self._geri_source = source
        self.geri_mode = True
        limit_str = "tam iz" if dist_limit is None else f"{dist_limit:.1f} m"
        self.logger.log(f"[GERİ] BAŞLADI ({source}) - {self._geri_index} örnek, hedef: {limit_str}")

    def _kacis_mesafesi(self):
        """Auto kaçışın geri mesafesi (P0 №4, E6-O2): önceki tetik yakın zamanda
        ve aynı noktadaysa 2→4 m eskalasyon — 2 m'lik kaçış aynı izi geri oynatıp
        aynı dar köşeye döndüğünden deterministik takılma döngüsü kuruyordu.
        Çağrı tetik konumunu günceller (bir sonraki kıyasın referansı)."""
        now = time.time()
        dist = GERI_ESCAPE_DISTANCE_M
        if (self._geri_escape_last_xy is not None
                and now - self._geri_escape_last_t < GERI_ESCAPE_ESKALASYON_PENCERE_S
                and math.hypot(self.x - self._geri_escape_last_xy[0],
                               self.y - self._geri_escape_last_xy[1]) < GERI_ESCAPE_AYNI_NOKTA_M):
            dist = GERI_ESCAPE_ESKALASYON_M
        self._geri_escape_last_xy = (self.x, self.y)
        self._geri_escape_last_t = now
        return dist

    def _stuck_check(self, target_speed_kmh, current_speed_kmh):
        """Sıkışma kaçışı tetiği: araç ilerlemeye ÇALIŞIRKEN (hedef hız var)
        STUCK_ESCAPE_TIME_S boyunca hareketsiz kalırsa True döner. Yalnız gerçek
        sürüş bağlamında (bu metod drive bölümünde çağrılır) değerlendirilir; DUR/
        e-stop hard/durak dalları buraya ulaşmadan continue eder. Herhangi bir
        hareket veya 'ilerleme beklenmiyor' durumu sayacı sıfırlar (bayat referans yok)."""
        now = time.time()
        # P0 №2 (E1-O2/E4-O2): bu metod sürüş dalı dışında hiç çağrılmadığından
        # sayaç, DUR kilidi öncesindeki tek-tick'lik bir cruise-flicker'ından
        # DONUK kalabiliyordu → 209 s sonra ikinci flicker'da bayat ateşleme
        # (20260716T181851Z kanıtı). Değerlendirme boşluğu görülürse sayaç bayattır.
        if self._stuck_last_eval is not None and (now - self._stuck_last_eval) > STUCK_EVAL_GAP_S:
            self._stuck_since = None
        self._stuck_last_eval = now
        trying = target_speed_kmh > 0.5
        moving = current_speed_kmh > STUCK_ESCAPE_SPEED_KMH
        if (not trying) or moving or (now < self._stuck_cooldown_until):
            self._stuck_since = None
            return False
        if self._stuck_since is None:
            self._stuck_since = now
            return False
        if now - self._stuck_since > STUCK_ESCAPE_TIME_S:
            self._stuck_since = None
            return True
        return False

    def run(self):
        """Ana kontrol döngüsü"""
        rate = rospy.Rate(50)  # 50 Hz

        # Başlatma komutunu bekle.
        # bus_release_on_start: manuel modda direksiyon seti bus'ı sürüyor; biz
        # frame yazmayız, sadece 0x500=1 (buton 1) gelmesini bekleriz.
        while not rospy.is_shutdown() and self.is_running and not self.mission_started:
            if not self.bus_release_on_start:
                self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
            time.sleep(0.1)

        self.logger.log("GÖREV BAŞLATILIYOR! /hedef bekleniyor...")

        while not rospy.is_shutdown() and self.is_running:

            # ========== MANUEL DEVRALMA (0x500=0) ==========
            # Direksiyon seti bus'ı devraldı: hiçbir frame gönderme ki 0x100/0x201
            # çakışması olmasın. 0x500=1 gelince devam edilir.
            if self.autonomous_paused:
                rospy.loginfo_throttle(2.0, "[MANUEL] Direksiyon seti devraldı - otonom duraklatıldı")
                rate.sleep()
                continue

            # ========== GERİ SÜRÜŞ (breadcrumb retrace) ==========
            # /geri_komut True: ileri sürüşte kaydedilen (direksiyon, gaz) izini TERS
            # sırayla oynat + vites REVERSE → araç geldiği rotayı geri izler.
            # Snapshot/başlatma yalnız burada (main-thread) yapılır; callback yalnız
            # bool set eder → deque'e tek thread yazar. Rising-edge tetik: doğal
            # bitişte komut True kalsa bile yeniden başlamaz. İleri-bakan soft e-stop
            # geri sürüşte ATLANIR (geri giderken öndeki engel çarpma yolu değil);
            # ACIL_DURUS tam fren yetkisini KORUR (arka sensör YOK).
            # Manuel /geri_komut: rising-edge'de tam-iz geri sürüş başlat; False iptal
            # eder — ancak YALNIZ manuel kaynaklı geri sürüşü (auto sıkışma kaçışını
            # /geri_komut=False iptal etmez, o mesafe/iz ile kendi biter).
            cmd = self._geri_cmd
            if cmd and not self._geri_prev_cmd and not self.geri_mode:
                self._start_geri(dist_limit=None, source='manual')
            elif not cmd and self.geri_mode and self._geri_source == 'manual':
                self.geri_mode = False
                self._geri_playback = None
                self.logger.log("[GERİ] İPTAL (komut False)")
            self._geri_prev_cmd = cmd

            if self.geri_mode:
                if self.karar == Karar.ACIL_DURUS:
                    self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
                    rospy.loginfo_throttle(1.0, "[GERİ] ACIL_DURUS - tam fren")
                    rate.sleep()
                    continue
                steer_deg, throttle_pct, done = self._geri_replay_step()
                # Mesafe tavanı (auto sıkışma kaçışı = 1 m): kat edilen geri mesafe
                # limite ulaşınca dur — iz tükenmese bile.
                reason = "iz tükendi"
                if not done and self._geri_dist_limit is not None:
                    traveled = math.hypot(self.x - self._geri_start_xy[0],
                                          self.y - self._geri_start_xy[1])
                    if traveled >= self._geri_dist_limit:
                        done = True
                        reason = f"{traveled:.2f} m geri gidildi"
                if done:
                    if self._geri_source == 'auto':
                        # Kaçış bitti → thrash önlemek için cooldown başlat
                        self._stuck_cooldown_until = time.time() + STUCK_ESCAPE_COOLDOWN_S
                    self.geri_mode = False
                    self._geri_playback = None
                    self._geri_source = None
                    self._geri_dist_limit = None
                    self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_NEUTRAL)
                    self.logger.log(f"[GERİ] Geri sürüş tamam ({reason}), DUR")
                    rate.sleep()
                    continue
                # Slew-rate + direksiyon limitleri (sürüş dalıyla birebir; CAN süreksizliği olmasın)
                max_delta = STEER_RATE_MAX_DEG_S * LOOP_DT
                steer_deg = float(np.clip(steer_deg,
                                          self._prev_cmd_steer - max_delta,
                                          self._prev_cmd_steer + max_delta))
                steer_deg = float(np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE))
                self._send_can_command(throttle_pct=throttle_pct, brake_pct=0,
                                       steer_deg=steer_deg, gear=GEAR_REVERSE)
                self.logger.csv(self.x, self.y, self.yaw, self.speed_ms * 3.6,
                                self.karar, self.x, self.y, throttle_pct, 0.0, steer_deg, 'R')
                rospy.loginfo_throttle(0.5,
                    f"[GERİ] kalan={self._geri_index} | Dir: {steer_deg:+.1f} | Gaz: {throttle_pct:.0f}")
                rate.sleep()
                continue

            # ========== H-B: DOĞRUDAN E-STOP GÜVENLİK AĞI (/karar merdiveni ÜSTÜNDE) ==========
            # karar BT'nin 10 Hz + String gecikmesini ATLA: dead-ahead dar koridorda
            # engel yakınsa control DOĞRUDAN /obstacles/poses'tan tam fren basar.
            # Latched (tampon donarsa korunur), dar koridor reroute'u öldürmez (§3.4 / H-B).
            # Her tick çağrılır → latch/release debounce düzgün işler.
            estop_active = self._update_estop()
            if estop_active and self._estop_hard:
                # (1) HARD FLOOR (<ESTOP_HARD_M, dar koridor): gerçek acil → KOŞULSUZ
                #     tam fren + vites N, latched. Bu kademe DOKUNULMAZ (son çare).
                self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
                rospy.logwarn_throttle(1.0, "[E-STOP] Hard floor engel — doğrudan tam fren")
                rate.sleep()
                continue
            # (2) SOFT Ackermann-yay e-stop (ESTOP_HARD_M..ESTOP_FWD_M): DURMA.
            #     Eski "steer=0 + tam fren + continue" düz yayı hiç bırakmadığı için
            #     dead-ahead dubada KİLİTLENİYORDU (run 214118). Artık döngüye düşürülür:
            #     hız aşağıda ESTOP_CRAWL_KMH'e sınırlanır + S-manevra/pursuit direksiyonu
            #     sürer → dönen yay engeli kaçırınca _update_estop debounce ile bırakır.
            #     Backstop hâlâ HARD floor (1.0m) + S-manevra timeout/swing kapısı.
            if estop_active:
                rospy.logwarn_throttle(1.0, "[E-STOP] Soft yay engeli — yavaşla + manevra")

            # ========== /karar STALENESS WATCHDOG (fail-safe) ==========
            # karar node çökerse/donarsa control son karara asılı kalmasın → DUR.
            # Yalnız en az bir karar alındıktan sonra silahlanır (karar hiç
            # başlamadıysa eski "normal devam" davranışını bozmaz).
            if (self.last_karar_time is not None
                    and time.time() - self.last_karar_time > KARAR_TIMEOUT):
                self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_FORWARD)
                rospy.logwarn_throttle(
                    2.0,
                    f"[WATCHDOG] /karar {time.time() - self.last_karar_time:.1f}s sessiz "
                    f"- DUR (fail-safe)")
                rate.sleep()
                continue

            # ========== KARAR: ACIL DURUS ==========
            # Geri kaçış bu dala bilerek KONMADI (güvenlik: acildurus = koşulsuz
            # hareketsizlik). Statik engelde kilitten çıkış yolu: karar mührü
            # statik-durumda kararı 'dur'a indirir (P0 №3) → aşağıdaki DUR dalı
            # kaçışı devralır.
            if self.karar == Karar.ACIL_DURUS:
                self._stuck_since = None   # P0 №2: sürüş sıkışma sayacı ACİL'de bayatlamasın
                self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
                rospy.loginfo_throttle(1.0, "[ACIL DURUS] Tam fren, vites N")
                rate.sleep()
                continue

            # ========== KARAR: DUR ==========
            if self.karar == Karar.DUR:
                self._stuck_since = None   # P0 №2: sürüş sıkışma sayacı DUR'da bayatlamasın
                # P0 №1 (E1-O1/E8-R2): engel-kaynaklı DUR kilidi kaçışı. Yalnız
                # reason engel blokajıysa (DUR_KACIS_REASONS — levha/yaya/ışık
                # dur'ları HARİÇ) ve araç DUR_KACIS_TIME_S boyunca hareketsizse
                # mevcut breadcrumb kaçışı tetiklenir (yeni mekanizma yok).
                # Sayaç, DUR dalı DUR_KACIS_EVAL_GAP_S'ten uzun ziyaret edilmeyince
                # sıfırlanır: karar churn'ünün tek-tick flicker'ları birikimi
                # bozmaz ama gerçek sürüşe dönüş sayacı temizler (bayat ateşleme yok).
                now = time.time()
                if (self._dur_kacis_last_eval is not None
                        and now - self._dur_kacis_last_eval > DUR_KACIS_EVAL_GAP_S):
                    self._dur_kacis_since = None
                self._dur_kacis_last_eval = now
                hareketsiz = (self.speed_ms * 3.6) < STUCK_ESCAPE_SPEED_KMH
                if (self.karar_reason in DUR_KACIS_REASONS and hareketsiz
                        and now >= self._stuck_cooldown_until):
                    if self._dur_kacis_since is None:
                        self._dur_kacis_since = now
                    elif now - self._dur_kacis_since > DUR_KACIS_TIME_S:
                        self._dur_kacis_since = None
                        kacis_m = self._kacis_mesafesi()
                        self._start_geri(dist_limit=kacis_m, source='auto')
                        self.logger.log(
                            f"[SIKIŞMA] DUR({self.karar_reason}) "
                            f"{DUR_KACIS_TIME_S:.0f}s hareketsiz → {kacis_m:.1f} m geri kaçış")
                else:
                    self._dur_kacis_since = None
                self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_FORWARD)
                rospy.loginfo_throttle(2.0, "[DUR] Karar: dur")
                rate.sleep()
                continue

            # ========== GOREV DURAGI KONTROLU ==========
            # Gercek gorev duraklarina varildi mi kontrol et
            if self._check_gorev_arrival():
                # Durakta bekliyoruz - fren uygula
                self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_FORWARD)
                rospy.loginfo_throttle(1.0, "[DURAK] Gorev duraginda bekleniyor...")
                rate.sleep()
                continue

            # Mission complete ise park et — el freni ÇEK (0x102), bir kez (H5).
            # park() ~1 s bloklar (kabul edilebilir, görev bitti) + el frenini
            # latch'ler; sonraki tick'ler fren+N ile tutar. +60/tur (rapor §2).
            if self.mission_complete:
                if not self.parked:
                    self.park()
                    self.parked = True
                else:
                    self._send_can_command(throttle_pct=0, brake_pct=100, steer_deg=0, gear=GEAR_NEUTRAL)
                rospy.loginfo_throttle(2.0, "[PARK] Gorev tamamlandi - el freni çekili")
                rate.sleep()
                continue

            # ========== HEDEF KONTROLÜ ==========
            target = self.dynamic_target

            if target is None:
                # Hedef YOK → ANINDA DUR, hareket etme (rapor H5; eski 8 sn'lik
                # yavaş-creep kaldırıldı). dynamic_target ilk /hedef gelene kadar
                # None; araç nereye gideceğini bilmeden ilerlememeli (start'ta
                # körlemesine creep DQ riski). Fren basıp bekle.
                if self.target_none_since is None:
                    self.target_none_since = time.time()
                    self.logger.log("Hedef bekleniyor - araç DURUYOR (creep yok)")
                self._send_can_command(throttle_pct=0, brake_pct=80, steer_deg=0, gear=GEAR_FORWARD)
                rospy.loginfo_throttle(2.0, "[BEKLE] Hedef yok - fren, hareket yok")
                rate.sleep()
                continue
            else:
                # Hedef var, bekleme sayacını sıfırla
                self.target_none_since = None

            target_x, target_y = target
            lookahead_secondary = self.next_target  # Pure Pursuit lookahead uzatması (WP2)

            # WP1'e mesafe
            distance = self._distance_to(target_x, target_y)

            # WP1'e yakinken: varildi gonder (hedef yoneticisi wp_index ilerlesin) + WP2'ye lookahead
            if distance < WP_NEAR_DISTANCE:
                now = time.time()
                if now - self.last_wp_varildi_time > 0.15:  # ~7Hz throttle
                    self.pub_gorev.publish("varildi")
                    self.last_wp_varildi_time = now
                if self.next_target is not None:
                    target_x, target_y = self.next_target
                    distance = self._distance_to(target_x, target_y)
                    lookahead_secondary = None  # zaten WP2'ye terfi edildi

            # Açı hatasını hesapla
            heading_error = self._heading_error(target_x, target_y)

            # U-dönüşü koruması: Hedef aracın arkasındaysa (>90°). S-MANEVRA
            # aktifken ATLA (§18) — manevra steering'i sahiplenir; planlayıcı
            # restore edince hedef geçici 'arkada' görünse de manevra latched
            # sol-WP'ye göre sürdüğü için durmamalı (yoksa manevra yarıda kesilir).
            # GÜVENLİK (§incele): bypass sırasında DUR yetkisi kapalı ama manevra
            # SINIRLI — TOWARD swing kapısı (≤45°+rampa), AWAY heading'i geri
            # yakınsar, her faz SMANEUVER_TIMEOUT=6s backstop'ı var ve H-B e-stop
            # bu döngünün ÜSTÜNDE çalışmaya devam eder → serbest dönme/runaway sınırlı.
            if (self._sman_phase == 'IDLE'
                    and abs(heading_error) > math.radians(90)
                    and distance < ARRIVAL_THRESHOLD * 3):
                resolved = False
                if self.next_target is not None:
                    # WP2'ye yonlenmeyi dene
                    nx, ny = self.next_target
                    nh = self._heading_error(nx, ny)
                    if abs(nh) <= math.radians(90):
                        # WP2 onde - ona yonlen
                        target_x, target_y = nx, ny
                        distance = self._distance_to(nx, ny)
                        heading_error = nh
                        lookahead_secondary = None  # WP2'ye yönlenildi
                        resolved = True

                if not resolved:
                    # Hem WP1 hem WP2 arkada (veya WP2 yok)
                    # Donmeye calisma, dur ve varildi gondermeye devam et
                    now = time.time()
                    if now - self.last_wp_varildi_time > 0.15:
                        self.pub_gorev.publish("varildi")
                        self.last_wp_varildi_time = now
                    self._send_can_command(throttle_pct=0, brake_pct=40, steer_deg=0, gear=GEAR_FORWARD)
                    rospy.loginfo_throttle(1.0, "[U-DONUS] Hedef arkada - dur + varildi gonderiliyor")
                    rate.sleep()
                    continue

            # Adaptif PID ayarlarını güncelle
            self._adapt_pid_gains(heading_error, distance)

            # ========== HIZ KONTROLÜ ==========
            # Karar durumuna göre hız limiti
            speed_limit = self._get_speed_limit()

            max_speed = min(speed_limit, MAX_SPEED_KMH)
            base_speed = max_speed

            # Gorev duragina yaklasirken yavasla (mikro-WP mesafesi degil)
            gorev_dist = self._nearest_gorev_distance()
            if gorev_dist < SLOWDOWN_DISTANCE:
                distance_factor = max(0.3, gorev_dist / SLOWDOWN_DISTANCE)
                base_speed *= distance_factor

            # Viraj öngörüsü: yavaşlamayı bir sonraki WP'ye (ileriye) bakarak tetikle.
            # Anlık WP'ye olan açı yoğun mikro-waypoint akışında virajda bile küçük
            # kaldığı için tek başına yavaşlatmaya yetmiyordu; next_target ~bir WP
            # ileride olduğu için viraj çok daha erken görülür.
            if self.next_target is not None:
                turn_heading_error = abs(self._heading_error(*self.next_target))
            else:
                turn_heading_error = abs(heading_error)

            if turn_heading_error > math.radians(TURN_SLOWDOWN_THRESHOLD):
                heading_factor = max(0.25, 1.0 - (turn_heading_error / math.pi) * TURN_SLOWDOWN_GAIN)
                base_speed *= heading_factor

            target_speed_kmh = max(base_speed, TURN_MIN_SPEED)

            # SOFT Ackermann-yay e-stop aktif: yalnız EMEKLE → manevranın engeli
            # kaçırması için alan aç (hard floor 1.0m backstop). Hard floor/stale
            # zaten yukarıda continue ile tam fren bastı; buraya yalnız soft düşer.
            if estop_active:
                target_speed_kmh = min(target_speed_kmh, ESTOP_CRAWL_KMH)

            # DÜZ TUT (S-manevra bekletmesi, TOWARD veya AWAY) aktif: heading
            # sapmışken şerit farkındalığı olmadan ilerleniyor → hızı LIMIT_SLOW'a
            # sınırla ki bekletme penceresindeki yol-dışı sapma küçük kalsın
            # (güvenlik incelemesi 2026-07-15; karar zaten slow diyorsa fark etmez).
            if self._sman_hold and self._sman_phase != 'IDLE':
                target_speed_kmh = min(target_speed_kmh, LIMIT_SLOW)

            # Hız hatası
            current_speed_kmh = self.speed_ms * 3.6
            speed_error = target_speed_kmh - current_speed_kmh

            # ── SIKIŞMA KAÇIŞI (stuck → 1 m geri) ────────────────────
            # Buraya yalnız gerçek sürüş bağlamı düşer (DUR/e-stop-hard/durak/hedef-yok
            # zaten yukarıda continue etti). İlerlemeye çalışırken (hedef hız var)
            # STUCK_ESCAPE_TIME_S boyunca hareketsiz kalındıysa = kurtulamayacağı engele
            # saplanmış → izi GERI_ESCAPE_DISTANCE_M (1 m) kadar geri izle, sonra normale dön.
            if self._stuck_check(target_speed_kmh, current_speed_kmh):
                kacis_m = self._kacis_mesafesi()   # P0 №4: aynı noktada 2. tetikte 2→4 m
                self._start_geri(dist_limit=kacis_m, source='auto')
                self.logger.log(
                    f"[SIKIŞMA] {STUCK_ESCAPE_TIME_S:.0f}s hareketsiz "
                    f"(hedef {target_speed_kmh:.1f} km/h, gerçek {current_speed_kmh:.2f}) "
                    f"→ {kacis_m:.1f} m geri kaçış")
                self._send_can_command(throttle_pct=0, brake_pct=60, steer_deg=0, gear=GEAR_NEUTRAL)
                rate.sleep()
                continue

            # PID çıkışı
            throttle = self.speed_pid.compute(speed_error, measurement=current_speed_kmh)

            # ── FIX 4: Anti-stall kick-start ─────────────────────────
            # Bug: static friction + düşük PID throttle = araç hareket etmez.
            # SLOWDOWN_DISTANCE içinde base_speed * 0.3 → target ~1.5 km/h →
            # PID kp=4 × error=1.5 = ~6 throttle birim (0-100 ölçek) →
            # static friction altı → durdu. Kalkış için kısa süreli (2 sn)
            # min 15 throttle birim (% gibi), sonrası normal.
            # NOT: speed_pid output_max=100 → throttle 0-100 ölçek. 15 birim
            # ≈ %15 gaz. Reviewer-fix: önceki "0.15" 0-1 sanıldı, no-op'tu.
            # Koşullar:
            #   target_speed_kmh > 0.5  (hedef hız var)
            #   speed_limit > 0         (DUR/ACIL_DURUS değil)
            #   current < target - 0.3  (hızlanma fazı)
            #   throttle < 15.0         (PID yetersiz, 0-100 ölçekte)
            # Reset: current > target/2  (hızlanma başladı, %50 hedef)
            now_ts = time.time()
            stall_start = self._stall_start_time
            if (target_speed_kmh > 0.5
                    and speed_limit > 0
                    and current_speed_kmh < target_speed_kmh - 0.3
                    and throttle < 15.0):
                if stall_start is None:
                    self._stall_start_time = now_ts
                    stall_start = now_ts
                stall_duration = now_ts - stall_start
                if stall_duration < 2.0:
                    throttle = 15.0  # kick-start moment (0-100 ölçek)
                    rospy.loginfo_throttle(
                        1.0,
                        f"[STALL-KICK] {stall_duration:.1f}s kick-start: "
                        f"target={target_speed_kmh:.1f} cur={current_speed_kmh:.1f}")
                # 2 sn sonra kick-start kapanır — gerçek engel olabilir,
                # FIX 5 (stuck detector) bu durumda devreye girer.
            elif current_speed_kmh > target_speed_kmh * 0.5:
                # Araç hız aldı (hedefin yarısı üstünde) → stall timer sıfırla
                self._stall_start_time = None

            # Gaz/fren kararı
            # Hedef hızın belirgin üstündeysek AKTİF FREN uygula. Hız PID'i
            # output_min=0 olduğu için negatif (fren) üretemez; bu yüzden eski
            # "throttle<0 → fren" dalı ölüydü ve slow/viraj/yaklaşma yavaşlaması
            # yalnız gazı kesip serbest yuvarlanmaya bırakıyordu → 5 km/h'den
            # 2.5'e inmiyordu (CSV kanıtı: karar=slow satırlarında hız 4.94-5.00).
            # Şimdi hedef hıza göre orantılı fren basılır (karar=slow dahil her
            # yavaşlama kaynağında: slow limiti, durak yaklaşımı, viraj).
            speed_overshoot = current_speed_kmh - target_speed_kmh
            if speed_overshoot > SLOWDOWN_BRAKE_MARGIN:
                throttle_pct = 0
                brake_pct = float(np.clip(speed_overshoot * SLOWDOWN_BRAKE_GAIN,
                                          0.0, SLOWDOWN_BRAKE_MAX))
            elif throttle > 0 and current_speed_kmh < speed_limit:
                throttle_pct = throttle
                brake_pct = 0
            else:
                # Hedefe yakın / limitte → gaz yok, serbest yuvarlanma
                throttle_pct = 0
                brake_pct = 0

            # ========== DİREKSİYON ==========
            # Önce KESKİN S-MANEVRA (§18): reroute WP gelince aktifleşir ve
            # steering'i SAHİPLENİR (TAM SOL → hiza → TAM SAĞ). Pursuit/​/line/
            # sharp-gate bypass; latched WP sayesinde hedef flip-flop'undan bağımsız.
            sman_steer = self._sman_update()
            if sman_steer is not None:
                steer_deg = sman_steer
                steer_source = "S-MANEVRA"
            else:
                # Düz takip — H-A: Pure Pursuit + /line (kaçınma planlayıcıda).
                #   PURSUIT      : saf Pure Pursuit (virajda /line bastırılır)
                #   PURSUIT+LINE : düz kesimde /line düzeltmesi eklenir
                steer_deg = self._pure_pursuit_steer((target_x, target_y), lookahead_secondary)
                steer_source = "PURSUIT"
                # /line yalnız (a) düz kesimde (virajda PP zaten yönetir) VE
                # (b) engel-bağlamı DIŞINDA uygulanır (§17 BİRİNCİL fix). Engel
                # yakını/slalomda (karar≠normal, TTL'li) /line bastırılır: rota yanal
                # manevra yaparken /line ters basıp 12° kapısında bang-bang üretiyordu.
                line_suppressed = (time.time() - self._last_obstacle_ctx_t) < LINE_SUPPRESS_TTL
                if (not line_suppressed
                        and abs(heading_error) < math.radians(LINE_GATE_MAX_HEADING)):
                    steer_deg += self._get_line_correction()
                    steer_source = "PURSUIT+LINE"

            if steer_source != self._last_steer_source:
                self.logger.log(f"DİREKSİYON KAYNAĞI: {self._last_steer_source} → {steer_source}")
                self._last_steer_source = steer_source

            # SLEW-RATE limiti (anti-oscillation, §17): direksiyon son GERÇEKTEN
            # gönderilen değerden (_prev_cmd_steer, _send_can_command'da tazelenir)
            # tek tick'te STEER_RATE_MAX_DEG_S·dt'den fazla sapamaz → flip-flop
            # snap'i (+9°→−30°) rampaya iner + line/sharp-gate artığı titreme
            # sönümlenir. E-stop/dur dalları steer=0 gönderdiğinde _prev_cmd_steer
            # de 0 olur → sürüş dönünce 0'dan rampalar (fiziksel olarak doğru).
            max_delta = STEER_RATE_MAX_DEG_S * LOOP_DT
            steer_deg = float(np.clip(steer_deg,
                                      self._prev_cmd_steer - max_delta,
                                      self._prev_cmd_steer + max_delta))
            # Direksiyon limitlerini uygula (son komut _send_can_command'da
            # _prev_cmd_steer'e yazılır → Ackermann e-stop + slew referansı oradan).
            steer_deg = float(np.clip(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE))

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

            # ── GERİ SÜRÜŞ İZ KAYDI ────────────────────────────────
            # Gönderilen (direksiyon, gaz) çiftini kaydet; /geri_komut gelince bu iz
            # TERS sırayla + vites REVERSE ile oynatılır → geldiği rota geri izlenir.
            # Yalnız GERÇEKTEN ilerlerken kaydet (dur/emekle tick'leri izi şişirmesin →
            # retrace'in mesafe-vs-tick sadakati korunur).
            if current_speed_kmh > GERI_RECORD_MIN_KMH:
                self._breadcrumb.append((steer_deg, throttle_pct))

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

            # Hedef markerlarini yayinla (RViz icin)
            self._publish_hedef_markers()

            rate.sleep()

        # Temizlik
        self.is_running = False
        self.stop()
        self.bus.shutdown()
        self.logger.log("CAN Waypoint Follower kapatıldı.")
        self.logger.close()  # CSV son satırları diske yaz


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
    print("  Kontrol Ayarları:")
    speed_p = PIDPresets.get_speed_preset(args.mode)
    print(f"    Hız PID: kp={speed_p['kp']}, ki={speed_p['ki']}, kd={speed_p['kd']}")
    print(f"    Dir: Pure Pursuit | L={WHEELBASE}m, Ld={LOOKAHEAD_K}*v+{LOOKAHEAD_B} "
          f"[{LOOKAHEAD_MIN}-{LOOKAHEAD_MAX}m]")
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
