#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_smaneuver.py — §18 KESKİN S-MANEVRA + GÜVENLİ DÖNÜŞ KAPISI regresyon testi.

Manevra trajektoriyi değiştirir (kapalı döngü) → kayıt replay'i yetmez; bisiklet
modeliyle simüle edip kontrol edilir:
  1. Manevra TAMAMLANIR (IDLE'a döner, timeout YOK, DÖNGÜ yok).
  2. Dubayı GEÇER (gövde-dikdörtgeni klirensi; nokta-mesafesi değil — dönüşte
     çarpan şey ÖN DIŞ KÖŞE, nokta metriği onu görmüyordu).
  3. Direksiyon osilasyonu YOK (slew-bounded, ters-dönüş yok).
  4. GÜVENLİ DÖNÜŞ KAPISI (2026-07-15): eski şeride dönüş (TOWARD→AWAY) koni,
     tam-kilit dönüşün süpürme alanından (yay+paralel koridor) çıkmadan BAŞLAMAZ;
     swing tavanında koni hâlâ riskliyse DÜZ TUT. Kapı regresyonları:
       - ofsetli koni: eski mantık dönüşte TEMAS ederdi, kapı klirensi kurtarır
       - dropout: koni hafızası (dünya-frame TTL) erken dönüşü köprüler
       - veri yok: eski sol-WP-hizası davranışına düşer (fallback birebir)
  5. REGRESYON: eski "sol-WP abeam'e kadar full-lock" tasarımı DÖNGÜ yapar (neden
     swing kapısı ŞART — bunu koruyalım).
  6. tam_kilit_donus_riskli saf-fonksiyon birim testleri (GERÇEK koddan import).

Geçiş mantığı control.py _sman_update'in BİREBİR kopyası (rospy'siz);
tam_kilit_donus_riskli ve tüm sabitler GERÇEK control.py'den alınır.
Çalıştır: python3 control/test_smaneuver.py
"""
import math
import sys
import types
import os

# Sabitler GERÇEK control.py'den alınır (kopya DEĞİL). ROS tepe-import'ları stub'lanır.
for _name in ('rospy', 'can', 'tf'):
    sys.modules.setdefault(_name, types.ModuleType(_name))
def _stub_module(name, attrs):
    m = types.ModuleType(name)
    for a in attrs:
        setattr(m, a, object)
    sys.modules[name] = m
_stub_module('nav_msgs', []); _stub_module('nav_msgs.msg', ['Odometry'])
_stub_module('std_msgs', []); _stub_module('std_msgs.msg', ['Bool', 'Float32', 'String'])
_stub_module('visualization_msgs', []); _stub_module('visualization_msgs.msg', ['Marker'])
_stub_module('geometry_msgs', []); _stub_module('geometry_msgs.msg', ['Point', 'PoseArray'])
_stub_module('tf.transformations', ['euler_from_quaternion'])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control as C

MAX_STEER = C.MAX_STEER_ANGLE
WHEELBASE = C.WHEELBASE
STEER_RATE_MAX_DEG_S = C.STEER_RATE_MAX_DEG_S
DT = C.LOOP_DT
SMANEUVER_MAX_SWING_DEG = C.SMANEUVER_MAX_SWING_DEG
SMANEUVER_ALIGN_DEG = C.SMANEUVER_ALIGN_DEG
SMANEUVER_TIMEOUT = C.SMANEUVER_TIMEOUT
HALF_BANT = C.ESTOP_BANT_YARIM_M + C.SMAN_DONUS_KLIRENS_M
YARI_GEN = C.ARAC_GENISLIK_M / 2.0

_fail = 0
def chk(cond, msg):
    global _fail
    print(f"  [{'OK  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def body_clearance(x, y, yaw, cone):
    """Koni merkezinin araç gövde dikdörtgenine uzaklığı − koni yarıçapı.
    Dikdörtgen (arka-aks çerçevesi): x∈[−ARAC_ARKA, ARAC_BURUN], |y|≤yarı-gen.
    (x,y) = base_link (sim durumu); arka aks BASE_ARKA_AKS geride. ≤0 = temas."""
    dx, dy = cone[0] - x, cone[1] - y
    cc, ss = math.cos(yaw), math.sin(yaw)
    bf = cc * dx + ss * dy + C.BASE_ARKA_AKS_M
    bl = -ss * dx + cc * dy
    ox = max(-C.ARAC_ARKA_M - bf, bf - C.ARAC_BURUN_M, 0.0)
    oy = max(abs(bl) - YARI_GEN, 0.0)
    return math.hypot(ox, oy) - C.ENGEL_YARICAP_M


def simulate(start, fwd_yaw, left_wp, v_kmh, cone, mode='gate',
             use_swing_gate=True, dropout_after=None, self_point=False,
             self_point_filtre=True):
    """control.py _sman_update geçiş mantığı + slew + bisiklet modeli kapalı-döngü.

    mode: 'gate'   = güvenli dönüş kapısı (koni görünür; dropout_after ile kesilir)
          'nodata' = koni verisi hiç yok → WP-hizası fallback
          'old'    = eski mantık (WP-hizası/swing) — kıyas tabanı
    Döner: (sonuç, min_klirens, dönüş_min_klirens, süre, away_t, ters_dönüş, max_dsteer)."""
    x, y, yaw = start[0], start[1], fwd_yaw
    v = v_kmh / 3.6
    sx, sy = x, y
    sdir = 1 if (-math.sin(fwd_yaw)*(left_wp[0]-sx) + math.cos(fwd_yaw)*(left_wp[1]-sy)) > 0 else -1
    wp_lat = sdir * (-math.sin(fwd_yaw)*(left_wp[0]-sx) + math.cos(fwd_yaw)*(left_wp[1]-sy))
    phase, phase_t, prev, t = 'TOWARD', 0.0, 0.0, 0.0
    mind, mind_ret, steers = 1e9, 1e9, []
    clear_since, last_seen, away_t, hold = None, None, None, False
    izlendi = False   # bu manevrada en az bir koni kanıtı görüldü mü
    max_lat = 0.0     # maks yanal sapma (KORİDOR SINIRI kanıtı — şerit ihlali)
    R_TK = WHEELBASE / math.tan(math.radians(MAX_STEER))
    def donus_kazanc(sw):
        a = math.radians(SMANEUVER_ALIGN_DEG)
        return R_TK * (math.cos(a) - math.cos(math.radians(max(sw, SMANEUVER_ALIGN_DEG))))
    while t < 20.0:
        d = (yaw - fwd_yaw + math.pi) % (2*math.pi) - math.pi
        swing = math.degrees(sdir * d)
        lat_prog = sdir * (-math.sin(fwd_yaw)*(x-sx) + math.cos(fwd_yaw)*(y-sy))
        max_lat = max(max_lat, lat_prog)
        if phase == 'TOWARD':
            tgt = sdir * MAX_STEER
            capped = use_swing_gate and swing >= SMANEUVER_MAX_SWING_DEG
            koridor = (mode != 'old') and (lat_prog + donus_kazanc(swing)
                                           >= wp_lat + C.SMAN_KORIDOR_ASIM_M)
            hazir, riskli = False, False
            if mode == 'gate':
                # --- control.py TOWARD kapısının aynası ---
                visible = dropout_after is None or t < dropout_after
                if visible:
                    last_seen = t   # koni hafızası: dünya-frame latch (statik koni)
                pts = []
                if last_seen is not None and (t - last_seen) <= C.KONI_HAFIZA_TTL_S:
                    dx, dy = cone[0]-x, cone[1]-y
                    cc, ss = math.cos(yaw), math.sin(yaw)
                    bf, bl = cc*dx + ss*dy, -ss*dx + cc*dy
                    pts.append((bf + C.BASE_ARKA_AKS_M, bl))
                if self_point:
                    # run 092553Z canlı bulgusu: detektör araç-üstü nokta basıyor
                    pts.append((0.55, -0.01))
                if self_point_filtre:
                    pts = C.govde_disi_filtre(pts, C.ARAC_ARKA_M, C.ARAC_BURUN_M,
                                              C.ARAC_GENISLIK_M / 2.0)
                if pts:
                    izlendi = True   # pozitif koni kanıtı (erken-dönüş yetkisi)
                engel = C.tam_kilit_donus_riskli(
                    pts, max(0.0, swing), -sdir, MAX_STEER, WHEELBASE,
                    HALF_BANT, C.ARAC_BURUN_M, C.ARAC_ARKA_M, C.SMAN_DONUS_TAIL_M)
                if engel is not None:
                    riskli = True
                    clear_since = None
                elif izlendi:
                    if clear_since is None:
                        clear_since = t
                    hazir = (t - clear_since) >= C.SMAN_DONUS_TEMIZ_S
            elif mode == 'nodata':
                pass   # kanıt yok: yalnız koridor sınırı + swing tavanı
            else:   # 'old' → eski WP-hizası referansı
                hazir = lat_prog >= wp_lat
            if mode == 'old':
                if hazir or capped:
                    phase, phase_t, away_t = 'AWAY', t, t
            elif koridor or hazir or (capped and not riskli):
                phase, phase_t, away_t = 'AWAY', t, t
            elif capped and riskli:
                tgt = 0.0   # DÜZ TUT (koridor sınırına kadar)
                if not hold:
                    hold = True
                    phase_t = t   # düz-tutuşa kendi timeout penceresi (≤2×)
        elif phase == 'AWAY':
            tgt = -sdir * MAX_STEER
            if swing <= SMANEUVER_ALIGN_DEG:
                phase = 'IDLE'; tgt = 0.0
            elif mode == 'gate':
                # AWAY duraklatma aynası: kalan süpürmede koni → düz tut,
                # AMA yalnız koridor izin verdiği sürece (şerit ihlali sınırı)
                koridor_izni = (lat_prog + donus_kazanc(swing)
                                < wp_lat + C.SMAN_KORIDOR_ASIM_M)
                visible = dropout_after is None or t < dropout_after
                if visible:
                    last_seen = t
                pts = []
                if last_seen is not None and (t - last_seen) <= C.KONI_HAFIZA_TTL_S:
                    dx, dy = cone[0]-x, cone[1]-y
                    cc, ss = math.cos(yaw), math.sin(yaw)
                    pts.append((cc*dx + ss*dy + C.BASE_ARKA_AKS_M, -ss*dx + cc*dy))
                if self_point:
                    pts.append((0.55, -0.01))
                if self_point_filtre:
                    pts = C.govde_disi_filtre(pts, C.ARAC_ARKA_M, C.ARAC_BURUN_M,
                                              C.ARAC_GENISLIK_M / 2.0)
                if koridor_izni and pts and C.tam_kilit_donus_riskli(
                        pts, max(0.0, swing), -sdir, MAX_STEER, WHEELBASE,
                        HALF_BANT, C.ARAC_BURUN_M, C.ARAC_ARKA_M,
                        C.SMAN_DONUS_TAIL_M) is not None:
                    tgt = 0.0
        else:
            tgt = 0.0
        if phase != 'IDLE' and (t - phase_t) > SMANEUVER_TIMEOUT:
            return ('TIMEOUT', mind, mind_ret, t, away_t, 0, 0.0, max_lat)
        md = STEER_RATE_MAX_DEG_S * DT
        st = max(prev - md, min(prev + md, tgt))
        st = max(-MAX_STEER, min(MAX_STEER, st)); prev = st
        steers.append(st)
        yaw += v / WHEELBASE * math.tan(math.radians(st)) * DT
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        clr = body_clearance(x, y, yaw, cone)
        mind = min(mind, clr)
        if away_t is not None:
            mind_ret = min(mind_ret, clr)
        t += DT
        if phase == 'IDLE':
            # IDLE sonrası mevcut heading'de 4 m düz devam (pursuit yaklaşıklaması)
            for _ in range(int(4.0 / (v * DT))):
                st = max(prev - md, min(prev + md, 0.0)); prev = st
                yaw += v / WHEELBASE * math.tan(math.radians(st)) * DT
                x += v * math.cos(yaw) * DT
                y += v * math.sin(yaw) * DT
                clr = body_clearance(x, y, yaw, cone)
                mind = min(mind, clr); mind_ret = min(mind_ret, clr)
            rev = sum(1 for i in range(1, len(steers))
                      if abs(steers[i]-steers[i-1]) > 15 and (steers[i] > 0) != (steers[i-1] > 0))
            mx = max(abs(steers[i]-steers[i-1]) for i in range(1, len(steers)))
            return ('OK', mind, mind_ret, t, away_t, rev, mx, max_lat)
    return ('NOEND', mind, mind_ret, t, away_t, 0, 0.0, max_lat)


# Senaryo: run 124844/135822 — araç (−4,−34.27) yaw0, reroute WP 2.2m sola, duba lane-merkez
START, FYAW, LEFT_WP, CONE = (-4.0, -34.27), 0.0, (0.1, -32.05), (-0.3, -34.04)
CLEAR_NEED = 0.20   # m - gövde-dikdörtgeni klirensi (0'ın altı temas)
WP_LAT = -32.05 + 34.27   # 2.22 m — reroute WP'nin yanal hizası (yasal koridor)
# KORİDOR SINIRI (şartname s.8 şerit ihlali / run 092553Z ELEME dersi): manevra
# boyunca yanal sapma WP hizası + ASIM + küçük ayrıklaştırma payını AŞAMAZ.
KORIDOR_MAX = WP_LAT + C.SMAN_KORIDOR_ASIM_M + 0.25

print("== KESKİN S-MANEVRA kapalı-döngü (§18, güvenli dönüş kapısı AKTİF) ==")
for v in (2.5, 4.0):
    res, mind, mret, t, at, rev, mx, mlat = simulate(START, FYAW, LEFT_WP, v, CONE, 'gate')
    print(f"  v={v}km/h → {res}  min_klirens={mind:.2f}m  dönüş_min={mret:.2f}m  süre={t:.1f}s  max_yanal={mlat:.2f}m")
    chk(res == 'OK', f"v={v}: manevra TAMAMLANDI (IDLE, timeout/döngü yok)")
    chk(mind >= CLEAR_NEED, f"v={v}: dubayı GEÇER (gövde klirensi {mind:.2f}≥{CLEAR_NEED})")
    chk(rev == 0, f"v={v}: direksiyon ters-dönüş YOK (osilasyonsuz)")
    chk(mx <= STEER_RATE_MAX_DEG_S*DT + 1e-9, f"v={v}: slew-bounded (max|Δ/tick|≤{STEER_RATE_MAX_DEG_S*DT:.0f}°)")
    chk(mlat <= KORIDOR_MAX, f"v={v}: KORİDOR SINIRI korunur (max yanal {mlat:.2f}≤{KORIDOR_MAX:.2f}m — şerit ihlali yok)")

print("\n== GÜVENLİ DÖNÜŞ KAPISI: ofsetli koni — eski mantık dönüşte TEMAS ederDİ ==")
# Koni reroute tarafına ofsetli (0.0,−33.0): eski (WP/swing-only) mantıkta dönüş
# fazı klirensi ≤0 (temas); kapı koniyi süpürme alanında görüp dönüşü erteler/erken alır.
KONI_OFSET = (0.0, -33.0)
ro = simulate(START, FYAW, LEFT_WP, 2.5, KONI_OFSET, 'old')
rn = simulate(START, FYAW, LEFT_WP, 2.5, KONI_OFSET, 'gate')
print(f"  eski: dönüş_min={ro[2]:.2f}m | kapı: dönüş_min={rn[2]:.2f}m (away {ro[4]:.1f}s → {rn[4]:.1f}s)")
chk(ro[2] < 0.05, f"eski mantık ofsetli konide dönüşte temas/sıyırma ({ro[2]:.2f}m) — kapının varlık nedeni")
chk(rn[0] == 'OK' and rn[2] >= CLEAR_NEED, f"kapı dönüş klirensini kurtarır ({rn[2]:.2f}≥{CLEAR_NEED})")

print("\n== GÜVENLİ DÖNÜŞ KAPISI: yandaki koni — koridor İÇİNDE kaldığı kadar beklet ==")
KONI_YAN = (2.0, -33.5)
ro = simulate(START, FYAW, LEFT_WP, 2.5, KONI_YAN, 'old')
rn = simulate(START, FYAW, LEFT_WP, 2.5, KONI_YAN, 'gate')
print(f"  eski: dönüş_min={ro[2]:.2f}m away={ro[4]:.1f}s | kapı: dönüş_min={rn[2]:.2f}m away={rn[4]:.1f}s max_yanal={rn[7]:.2f}m")
chk(rn[4] > ro[4], f"kapı dönüşü BEKLETİR (away {ro[4]:.1f}s → {rn[4]:.1f}s)")
chk(rn[2] >= ro[2] - 1e-9, f"dönüş klirensi kötüleşmez ({ro[2]:.2f} → {rn[2]:.2f}m)")
chk(rn[0] == 'OK', "bekletme manevrayı KİLİTLEMEZ (yine tamamlanır)")
chk(rn[7] <= KORIDOR_MAX, f"bekletme KORİDORU AŞMAZ (max yanal {rn[7]:.2f}≤{KORIDOR_MAX:.2f}m) — 092553Z'deki sınırsız sürüklenme kapandı")

print("\n== KONİ HAFIZASI: detektör dropout'u erken dönüşe yol açmaz ==")
r_full = simulate(START, FYAW, LEFT_WP, 2.5, KONI_YAN, 'gate')
r_drop = simulate(START, FYAW, LEFT_WP, 2.5, KONI_YAN, 'gate', dropout_after=1.5)
print(f"  kesintisiz: away={r_full[4]:.1f}s | 1.5s'te kör: away={r_drop[4]:.1f}s (TTL={C.KONI_HAFIZA_TTL_S}s köprüsü)")
chk(r_drop[4] >= min(r_full[4], 1.5 + C.KONI_HAFIZA_TTL_S) - 0.2,
    f"hafıza dropout'u köprüler: dönüş {r_drop[4]:.1f}s (kör anı 1.5s değil)")
chk(r_drop[0] == 'OK', "dropout'ta manevra yine tamamlanır")

print("\n== FALLBACK: koni verisi hiç yoksa eski WP-hizası davranışı birebir ==")
ro = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'old')
rf = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'nodata')
print(f"  eski: away={ro[4]:.2f}s min={ro[1]:.2f} | veri-yok: away={rf[4]:.2f}s min={rf[1]:.2f}")
chk(abs(ro[4] - rf[4]) < 1e-9 and abs(ro[1] - rf[1]) < 1e-9,
    "veri yokken davranış eski mantıkla ÖZDEŞ (güvenli fallback)")

print("\n== CANLI-AMA-KÖR detektör: koni gerçek ama HİÇ tespit yok → erken dönüş YASAK ==")
# Güvenlik incelemesi 2026-07-15 (Kritik #1): tampon taze+boş iken 'alan temiz'
# sayılıp 0.3s'te dönülüyordu (gövde klirensi −0.15 = TEMAS). Kanıt-şartı fixi:
# koni hiç görülmediyse WP-hizası fallback'ine düşülmeli (eski davranışla özdeş).
rb = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'gate', dropout_after=0.0)
print(f"  kör: away={rb[4]:.2f}s min={rb[1]:.2f} | eski: away={ro[4]:.2f}s min={ro[1]:.2f}")
chk(abs(rb[4] - ro[4]) < 1e-9 and abs(rb[1] - ro[1]) < 1e-9,
    "kör detektörde davranış eski mantıkla ÖZDEŞ (0.3s'te erken dönüş YOK)")
chk(rb[1] >= CLEAR_NEED, f"kör detektörde de temas yok (klirens {rb[1]:.2f}≥{CLEAR_NEED})")

print("\n== ARAÇ-ÜSTÜ NOKTA (run 092553Z): en kötü sensör bug'ında bile koridor korunur ==")
# Canlı bulgu: /obstacles/poses araç gövdesinden (arka-aks +0.55m) KALICI nokta
# basıyor → kapı 'alan hiç temiz değil' der. 092553Z'de bu, sınırsız DÜZ TUT ile
# YOLDAN ÇIKMAYA (eleme) yol açtı. İki katmanlı fix:
#   1) govde_disi_filtre: araç-üstü nokta elenir → kapı anlamlı çalışır,
#   2) KORİDOR SINIRI: filtre başarısız olsa bile dönüş WP hizasında ZORLANIR —
#      manevra kilitlenmez, şerit ihlali olmaz (savunma derinliği).
r_ham = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'gate', self_point=True, self_point_filtre=False)
r_fil = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'gate', self_point=True, self_point_filtre=True)
r_ref = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'gate')
print(f"  filtresiz: {r_ham[0]} max_yanal={r_ham[7]:.2f}m | filtreli: {r_fil[0]} away={r_fil[4]:.2f}s | referans away={r_ref[4]:.2f}s")
chk(r_ham[0] == 'OK' and r_ham[7] <= KORIDOR_MAX,
    f"filtre OLMASA BİLE koridor sınırı dönüşü zorlar (sonuç {r_ham[0]}, max yanal {r_ham[7]:.2f}≤{KORIDOR_MAX:.2f}m — 092553Z sınıfı eleme kapandı)")
chk(r_fil[0] == 'OK' and abs(r_fil[4] - r_ref[4]) < 0.5,
    f"gövde filtresi kapıyı anlamlı tutar (away {r_fil[4]:.2f}≈{r_ref[4]:.2f}s)")
chk(r_fil[1] >= CLEAR_NEED, f"filtreli koşuda temas yok (klirens {r_fil[1]:.2f}≥{CLEAR_NEED})")

print("\n== swing kapısı manevrayı SIKI/HIZLI tutar (yavaş golf-cart'ta aşırı dönmeyi keser) ==")
r_g = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'old', use_swing_gate=True)
r_ng = simulate(START, FYAW, LEFT_WP, 2.5, CONE, 'old', use_swing_gate=False)
print(f"  v=2.5  swing kapısı AÇIK → {r_g[0]} {r_g[3]:.1f}s  |  KAPALI → {r_ng[0]} {r_ng[3]:.1f}s")
chk(r_ng[3] - r_g[3] > 2.0, f"swing kapısı süreyi belirgin kısaltır ({r_ng[3]:.1f}s→{r_g[3]:.1f}s): WP'ye kadar "
                            f"full-lock yavaş golf-cart'ta ~73° döndürür (aşırı), kapı 45°'de keser")

print("\n== tam_kilit_donus_riskli birim testleri (saf fonksiyon, arka-aks çerçevesi) ==")
R_MIN = WHEELBASE / math.tan(math.radians(MAX_STEER))
print(f"  R_min = {R_MIN:.2f} m (L={WHEELBASE}, δ={MAX_STEER:.2f}°)")
def riskli(pts, swing, yon):
    return C.tam_kilit_donus_riskli(pts, swing, yon, MAX_STEER, WHEELBASE,
                                    HALF_BANT, C.ARAC_BURUN_M, C.ARAC_ARKA_M,
                                    C.SMAN_DONUS_TAIL_M)
chk(riskli([], 30.0, -1) is None, "boş nokta listesi → temiz")
chk(riskli([(3.0, 0.0)], 0.0, -1) is not None,
    "swing=0, tam önde 3m koni → KUYRUK koridorunda (riskli) — dönüş = düz devam üstünden geçer")
chk(riskli([(3.0, 1.6)], 0.0, -1) is None,
    f"swing=0, önde ama {1.6}m yanda → bant ({HALF_BANT:.2f}m) dışı, temiz")
chk(riskli([(-1.5, 0.0)], 30.0, -1) is None,
    "arkada kalmış koni (−1.5m) → yay sektörü + kuyruk dışı, temiz")
# Yay halkası içi/dışı: sağ dönüş ICR=(0,−R); gidiş yönünde θ=60°'lik nokta
p60 = (R_MIN * math.sin(math.radians(60)), -R_MIN + R_MIN * math.cos(math.radians(60)))
chk(riskli([p60], 45.0, -1) is not None,
    "yay üstünde θ=60° nokta, swing 45 (+burun payı) → sektör İÇİ, riskli")
chk(riskli([p60], 5.0, -1) is None,
    "aynı nokta swing 5 → dönüş yayı oraya uzanmaz, temiz")
uzak = (0.0, -2.0 * R_MIN - 1.0)
chk(riskli([uzak], 45.0, -1) is None, "halkanın dışında (ICR ötesi) → temiz")
chk(riskli([(-C.ARAC_ARKA_M, 0.85)], 20.0, 1) is not None,
    "REGRESYON (incele 2026-07-15): arka-iç köşe (ICR tarafı) bitişiğindeki koni "
    "riskli — açısal paylar r_ic ile hesaplanmalı, R ile değil")
# Sol/sağ ayna simetrisi
p_sag = [(2.0, -1.0)]; p_sol = [(2.0, 1.0)]
chk((riskli(p_sag, 30.0, -1) is None) == (riskli(p_sol, 30.0, 1) is None),
    "sol/sağ dönüş ayna-simetrik")
chk(riskli([(9.5, 0.0)], 0.0, -1) is None,
    f"kuyruk ufku ({C.SMAN_DONUS_TAIL_M}m arka-akstan) ötesindeki nokta → temiz")
# govde_disi_filtre birim testleri
gf = lambda pts: C.govde_disi_filtre(pts, C.ARAC_ARKA_M, C.ARAC_BURUN_M, C.ARAC_GENISLIK_M/2.0)
chk(gf([(0.55, -0.01)]) == [], "araç-üstü nokta (0.55,−0.01) → elenir (run 092553Z)")
chk(gf([(2.34, 0.0)]) == [], "burun ucu nokta → elenir (gövde içi)")
chk(gf([(0.5, 0.9)]) == [(0.5, 0.9)], "gövde yanı 0.9m (yarı-gen 0.6+pay dışı) → KALIR (gerçek koni)")
chk(gf([(3.0, 0.0)]) == [(3.0, 0.0)], "önde 3m → KALIR")
chk(gf([(-0.8, 0.0)]) == [(-0.8, 0.0)], "arkada 0.8m (gövde −0.5'ten geride) → KALIR")

print()
if _fail == 0:
    print("TÜM TESTLER GEÇTİ ✅")
else:
    print(f"{_fail} TEST BAŞARISIZ ❌")
    raise SystemExit(1)
