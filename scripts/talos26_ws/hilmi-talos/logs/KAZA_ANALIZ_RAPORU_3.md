# TALOS KAZA ANALIZ RAPORU #3
**Tarih:** 22 Subat 2026
**Analiz Eden:** Otonom Kontrol Sistemi Analizi (Claude)
**Konu:** Kuzey bolgede bati duvarina carpma - hedef gecikme sorunu

---

## YONETICI OZETI

Arac, 14 duraklik tam harita turunda 13 waypoint'i basariyla tamamladiktan sonra
kuzey bolgedeki bati duvarina carpti. Carpma noktasi: **(x=-5.37, y=9.72)**.

**Onceki duzeltmeler CALISTI:** SAG komutu virajda basariyla engellendi (8 reddetme,
3 iptal loglandi). U-donusu korumasi 2 arkadaki hedefi atladi. Tunel virajlari sorunsuz gecildi.

**KOK NEDEN:** Uc katmanli bir sorun:
1. DUR komutu calismıyor - her DUR 5-100ms icinde NORMAL ile eziliyor
2. Hedef gec geliyor - tamamlama sonrasi 3.5-8 saniye gecikme
3. Gecikme sirasinda arac hedefsiz ilerliyor → bati duvarina ulastyor

---

## A) BASARILI KISIMLAR

| Durak | Konum | Sonuc |
|-------|-------|-------|
| G1 | (-5.22, -34.32) | BASARILI |
| ara | (5.28, -34.47) | BASARILI |
| G2 | (9.28, -34.52) | BASARILI |
| ara | (10.88, -21.97) → (14.53, -21.97) | BASARILI - viraj 1 gecildi |
| G4 | (18.88, -21.87) | BASARILI |
| ara | (24.58, -17.47) | BASARILI - **SAG 8x reddedildi, 3x iptal edildi** |
| G5 | (24.53, -12.22) | BASARILI - tunel viraji gecildi! |
| ara | (24.33, -2.17) → (18.63, -2.02) | BASARILI |
| G9 | (10.53, 10.33) | BASARILI - kuzey bolgeye girildi |
| ara | (5.23, 10.48) | BASARILI |
| G10 | (-3.77, 10.68) | BASARILI |
| sonraki | (5.23, 10.48) - dogu donus | **BASARISIZ - BATI DUVARINA CARPMA** |

**13/14 waypoint basarili.** Tunel virajlari ve SAG engellemesi sorunsuz calisti.

---

## B) DUR KOMUTU SORUNU

### DUR/NORMAL Flip-Flop Ornegi (hedef tamamlandiktan sonra):
```
[15:05:00.114] HEDEF TAMAMLANDI: (18.88, -21.87)
[15:05:00.117] KARAR: DUR - bekleme basladi     ← 3ms sonra DUR
[15:05:00.128] KARAR: NORMAL                      ← 11ms sonra NORMAL!
[15:05:00.261] KARAR: DUR - bekleme basladi       ← tekrar DUR
[15:05:00.328] KARAR: NORMAL                      ← 67ms sonra NORMAL!
... (9 kez tekrar)
```

**Toplam 94+ DUR/NORMAL cifti** - hicbir DUR 100ms'den fazla surmuyor.

**Neden oluyor:** Karar sistemi (`/karar` topic) hedef_yoneticisi'nden bagimsiz calisiyor.
`hedef_yoneticisi` "dur" gonderiyor ama karar node'u hemen "normal" gonderiyor.
Iki farkli publisher ayni topic'e yaziyor ve birbirlerini eziyor.

---

## C) HEDEF GECIKME ANALIZI

| Tamamlanan Hedef | Tamamlanma | Sonraki Hedef Gelisi | Gecikme | Etki |
|-----------------|------------|---------------------|---------|------|
| (-5.22, -34.32) | 15:04:18.7 | (5.28, -34.47) 15:04:22.6 | **3.9s** | Hafif sapma |
| (9.28, -34.52) | 15:04:31.9 | (10.88, -21.97) 15:04:39.7 | **7.8s** | 2x tekrar |
| (18.88, -21.87) | 15:05:00.1 | (24.58, -17.47) 15:05:04.2 | **4.1s** | Hafif sapma |
| (24.53, -12.22) | 15:05:15.1 | (24.33, -2.17) 15:05:23.0 | **7.9s** | 2x tekrar |
| (18.63, -2.02) | 15:05:32.4 | (10.53, 10.33) 15:05:40.2 | **7.8s** | 2x tekrar |
| (5.23, 10.48) | 15:05:52.7 | (-3.77, 10.68) 15:05:56.6 | **3.9s** | Hafif sapma |
| **(-3.77, 10.68)** | **15:06:02.5** | **(5.23, 10.48) 15:06:10.4** | **7.9s** | **OLUMCUL** |

**Patern:**
- Normal gecikme: ~3.5-4.0 saniye (1 DUR dongusu)
- Cift gecikme: ~7.8 saniye (hedef yoneticisi eski hedefi tekrar gonderiyor → aninda tamamlaniyor →
  ikinci DUR dongusu)

---

## D) OLUMCUL OLAY KRONOLOJISI

### Faz 1: (-3.77, 10.68) Tamamlandi (15:06:02.540)
- Konum: x≈-4.2, y≈10.5
- Yaw: ~-175° (batiya bakiyor)
- Hiz: ~2.15 km/h (yavasliyordu ama durmadi)
- DUR/NORMAL flip-flop basladi

### Faz 2: Hedefsiz Ilerleme (15:06:02.5 → 15:06:06.0, 3.5 saniye)
- Arac batiya dogru ilerlemeye devam etti
- 9 DUR/NORMAL flip-flop - arac durmadi
- x: -4.2 → -4.5 (batiya kaydi)

### Faz 3: Atlamalar (15:06:06.0)
```
[15:06:06.024] HEDEF ATLANDI (arkada): (0.88, 10.53) heading_err=172° mesafe=3.2m
[15:06:06.426] HEDEF ATLANDI (arkada): (5.23, 10.48) heading_err=174° mesafe=7.8m
```
- Iki hedef de aracin arkasinda → atlandilar
- Bu 2. DUR dongusu baslatti (8 flip-flop daha)

### Faz 4: Eski Hedef Tekrari (15:06:09.840)
- (-3.77, 10.68) tekrar geldi → aninda tamamlandi (zaten gecmis)
- Bu 3. bekle-DUR dongusu olabilirdi

### Faz 5: Sonunda Gercek Hedef (15:06:10.392)
- (5.23, 10.48) alindi - 10.6m geride, doguya
- Arac x=-4.23, yaw=-175° (batiya gidiyor)
- Steer=30 (max sola) ile U-donusu baslatildi

### Faz 6: Bati Duvarına Carpma (15:06:12.9)
- Arac U-donusu yaparken x=-5.34'e ulasti
- Bati duvari - hiz 0.17 km/h'ye dustu
- Sonsuza kadar takildi: x=-5.37, steer=30, throttle=9.4

---

## E) COK HEDEF COZUMU (ONERILEN)

### Sorun: Tek hedef sistemi
Mevcut sistem sadece 1 hedef biliyor. Hedef tamamlaninca yeni hedef gelene kadar
3.5-8 saniye hedefsiz kaliyor.

### Cozum: Iki hedef sistemi
`hedef_yoneticisi` ayni anda **mevcut hedef + sonraki hedef** gondersin.
`control.py` mevcut hedefi tamamlayinca hemen sonraki hedefe gecsin, gecikme olmaz.

**Avantajlar:**
1. Hedef arasi gecikme sifira iner
2. Arac hedefsiz kalmaz
3. DUR sorunu onem kaybeder (arac zaten bir sonraki hedefe gidiyor)
4. Ara waypoint'lerde durma gerekmez, akici gecis

### Uygulama:
- `/hedef` mesaj formati: `"x1,y1;x2,y2"` (noktalı virgul ile ayrilmis)
- control.py: Birinci hedefe varilinca ikinciye aninda gec
- hedef_yoneticisi: Her zaman 2 waypoint yayinla

---

*Bu rapor, 22 Subat 2026 tarihli surus loglarinin analizi sonucu hazirlanmistir.*
