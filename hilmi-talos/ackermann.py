#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ackermann Direksiyon Geometrisi — Beemobs Bee1 (gerçek yarışma aracı)

Pure Pursuit kontrolcüsü tek bir "bisiklet modeli" direksiyon açısı (delta)
üretir: ön aksın ortasındaki sanal tekerleğin açısı. Gerçek araçta ise iki
ön tekerlek farklı açılarla döner — iç tekerlek dış tekerlekten daha keskin
döner, çünkü ikisi aynı dönüş merkezi etrafında farklı yarıçaplı yaylar çizer.
Bu modül, bisiklet-modeli açısı ile gerçek ön tekerlek açıları arasındaki
Ackermann dönüşümünü sağlar.

Araç parametreleri "2026 Robotaksi Binek Otonom Araç Yarışması — Hazır Araç
Genel Bilgilendirme Dokümanı" (Beemobs Bee1) belgesinden alınmıştır.

NOT — bu modül aktüatör yolunda ZORUNLU DEĞİLDİR:
Hem simülasyondaki araç modeli (`CartPlugin.cc`) hem de gerçek Bee1'in kremayer
(rack-and-pinion) mekanizması Ackermann geometrisini kendisi gerçekler. Gerçek
araca tekerlek başına ayrı açı gönderilmez; tek bir direksiyon hedefi verilir
(`/beemobs/AUTONOMOUS_SteeringMot_Control` PWM ya da dahili PID için
`/beemobs/steering_target_value`) ve mekanizma Ackermann dağıtımını yapar.
Bu modül bir MODEL KATMANIDIR; kontrolcünün bisiklet-modeli çıktısını gerçek
araç geometrisine bağlamak için kullanılır:
    1. Limit doğrulama   — Pure Pursuit delta'sını, hiçbir tekerlek mekanik
                           limiti (32.5° / 30.0°) aşmayacak şekilde sınırlama
    2. Geri besleme yorumu — /beemobs/FeedbackSteeringAngle (tekerlek açısı)
                           değerini bisiklet-modeli eşdeğerine çevirme
    3. Analiz            — minimum dönüş yarıçapı, dönüş merkezi hesabı
"""

import math

# =============================================================================
# BEEMOBS BEE1 ARAÇ PARAMETRELERİ
# Kaynak: 2026 Robotaksi Hazır Araç Genel Bilgilendirme Dokümanı
# =============================================================================
WHEELBASE = 1.860            # m       - dingil mesafesi (L)
FRONT_TRACK = 0.886          # m       - ön iz genişliği (T)
MAX_INNER_WHEEL_DEG = 32.5   # derece  - maksimum iç teker açısı
MAX_OUTER_WHEEL_DEG = 30.0   # derece  - maksimum dış teker açısı

# Türetilmiş sabit: iz genişliğinin dingil mesafesine oranı (T / 2L)
_TRACK_RATIO = FRONT_TRACK / (2.0 * WHEELBASE)


def bicycle_to_wheel_angles(delta_center_deg):
    """
    Bisiklet-modeli direksiyon açısını sol/sağ ön tekerlek açılarına çevirir.

    Girdi:
        delta_center_deg : Pure Pursuit'in ürettiği bisiklet-modeli açısı
                           (derece). + sol, - sağ.
    Çıktı:
        (sol_teker_deg, sag_teker_deg) : iki ön tekerleğin Ackermann açıları
                                         (derece, dönüş işareti korunur)

    Formül (delta = |delta_center|):
        delta_ic  = atan( tan(delta) / (1 - (T/2L)*tan(delta)) )
        delta_dis = atan( tan(delta) / (1 + (T/2L)*tan(delta)) )
    Sola dönüşte (delta > 0) sol tekerlek iç tekerlektir; sağa dönüşte sağ.
    """
    if abs(delta_center_deg) < 1e-6:
        return 0.0, 0.0

    sign = 1.0 if delta_center_deg > 0 else -1.0
    tan_d = math.tan(math.radians(abs(delta_center_deg)))

    denom_inner = 1.0 - _TRACK_RATIO * tan_d
    if denom_inner <= 1e-9:
        # Fiziksel olarak ulaşılamayan açı (delta ~ 76°+); güvenli sınıra çek
        denom_inner = 1e-9

    inner_deg = math.degrees(math.atan(tan_d / denom_inner))
    outer_deg = math.degrees(math.atan(tan_d / (1.0 + _TRACK_RATIO * tan_d)))

    if sign > 0:                       # sola dönüş: sol = iç, sağ = dış
        return sign * inner_deg, sign * outer_deg
    else:                              # sağa dönüş: sol = dış, sağ = iç
        return sign * outer_deg, sign * inner_deg


def wheel_to_bicycle_angle(wheel_angle_deg, is_inner):
    """
    Bir ön tekerleğin açısından bisiklet-modeli eşdeğer açıyı geri hesaplar.
    Gerçek araçta `/beemobs/FeedbackSteeringAngle` yorumlanırken kullanılır.

    Girdi:
        wheel_angle_deg : ölçülen tekerlek açısı (derece)
        is_inner        : True ise iç tekerlek, False ise dış tekerlek
    Çıktı:
        bisiklet-modeli eşdeğer açı (derece)
    """
    if abs(wheel_angle_deg) < 1e-6:
        return 0.0

    sign = 1.0 if wheel_angle_deg > 0 else -1.0
    tan_phi = math.tan(math.radians(abs(wheel_angle_deg)))
    half_track = FRONT_TRACK / 2.0

    # Tekerlek açısı -> dönüş yarıçapı (arka aks merkezine göre):
    #   iç tekerlek:  R - T/2 = L / tan(phi)
    #   dış tekerlek: R + T/2 = L / tan(phi)
    if is_inner:
        radius = WHEELBASE / tan_phi + half_track
    else:
        radius = WHEELBASE / tan_phi - half_track

    if radius <= 1e-9:
        return sign * 90.0

    return sign * math.degrees(math.atan(WHEELBASE / radius))


def turning_radius(delta_center_deg):
    """
    Bisiklet-modeli açısı için dönüş yarıçapı (m), arka aks merkezine göre.
    Düz gidişte (delta ~ 0) sonsuz döner.
    """
    if abs(delta_center_deg) < 1e-6:
        return float('inf')
    return WHEELBASE / math.tan(math.radians(abs(delta_center_deg)))


def max_bicycle_angle():
    """
    İki tekerlek mekanik limiti (iç 32.5°, dış 30.0°) göz önüne alındığında
    izin verilen maksimum bisiklet-modeli açısı (derece). Hangi tekerlek
    limitine önce ulaşılıyorsa o belirleyicidir.
    """
    from_inner = abs(wheel_to_bicycle_angle(MAX_INNER_WHEEL_DEG, is_inner=True))
    from_outer = abs(wheel_to_bicycle_angle(MAX_OUTER_WHEEL_DEG, is_inner=False))
    return min(from_inner, from_outer)


def clamp_to_wheel_limits(delta_center_deg):
    """
    Bisiklet-modeli açısını, hiçbir ön tekerlek mekanik limitini aşmayacak
    şekilde sınırlar. Pure Pursuit çıkışına uygulanır.
    """
    limit = max_bicycle_angle()
    return max(-limit, min(limit, delta_center_deg))


if __name__ == '__main__':
    # Hızlı doğrulama / dönüşüm tablosu
    print(f"Beemobs Bee1 — L={WHEELBASE} m, T_ön={FRONT_TRACK} m")
    print(f"Mekanik limitler: iç {MAX_INNER_WHEEL_DEG}°, dış {MAX_OUTER_WHEEL_DEG}°")
    print(f"Etkin maks. bisiklet açısı: {max_bicycle_angle():.2f}°\n")
    print(f"{'δ (bisiklet)':>13} | {'sol teker':>10} | {'sağ teker':>10} | {'R (m)':>8}")
    print("-" * 52)
    for d in (0, 5, 10, 15, 20, 25, 28.95, 30):
        sol, sag = bicycle_to_wheel_angles(d)
        r = turning_radius(d)
        r_str = "∞" if r == float('inf') else f"{r:.2f}"
        print(f"{d:>13.2f} | {sol:>10.2f} | {sag:>10.2f} | {r_str:>8}")
