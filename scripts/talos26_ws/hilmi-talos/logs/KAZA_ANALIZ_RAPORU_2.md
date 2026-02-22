# TALOS TUNEL KAZASI ANALIZ RAPORU #2
**Tarih:** 22 Subat 2026
**Analiz Eden:** Otonom Kontrol Sistemi Analizi (Claude)
**Konu:** Onceki duzeltme sonrasi aracin AYNI virajda tekrar duvara carpmasi

---

## YONETICI OZETI

Arac, 21 Subat'taki kazanin ardindan `_heading_error()` fonksiyonu duzeltildikten sonra
22 Subat'ta tekrar test edildi. Arac **ayni virajda** (viraj 2, x~24 bolgesinde dogudan
kuzeye donus) **tekrar tunel duvarina carpti**. Bu sefer carpma noktasi **(x=26.915, y=-17.311)**
olup, onceki kazadan (x=25.97) **0.95m daha doguya** savrulmus durumda.

**KOK NEDEN:** `_heading_error()` duzeltmesi dogru yapilmisti, ancak bu kazanin nedeni
tamamen farkli: **Virajin tam ortasinda gelen SAG (saga serit degistirme) komutu** direksiyonu
aniden **+24 dereceden -20 dereceye** (tam tersi yone) cevirdi. Yaklasik 1.1 saniye boyunca
arac virajin tersine donduruldu. PID kontrol geri alindiginda arac zaten tunel merkezinin
dogusundaydi ve duvar kacilmaz hale geldi.

---

## A) VERI KAYNAKLARI

| Dosya | Tarih | Aciklama | Sonuc |
|-------|-------|----------|-------|
| control_20260221_171030.csv | 21 Subat | Eski kaza (weighted heading) | CARPMA (x=25.97) |
| control_20260222_131525.csv | 22 Subat | Duzeltme sonrasi test | CARPMA (x=26.92) |

---

## B) CARPMA KRONOLOJISI

### Faz 1: Viraja Giris (Row 1673, t=16:16:05.83)
| Parametre | Deger |
|-----------|-------|
| Konum | x=21.307, y=-21.258 |
| Yaw | -4.0 derece (doguya bakiyor) |
| Hiz | 5.00 km/h |
| Direksiyon | +27.2 (sola, dogru) |
| Hedef | (24.58, -17.47) - viraj ara noktasi |
| Merkezden sapma | -3.19m (guvenli bolge, batida) |

Arac viraj 2'ye girmis, PID dogrudan maks direksiyonu (+27) uygulamis. Her sey normal.

### Faz 2: Normal Donus (Row 1673-1760)
| Row | x | Yaw | Steer | Hiz | Durum |
|-----|------|------|-------|------|-------|
| 1673 | 21.31 | -4.0° | +27.2 | 5.00 | Viraj baslangiici |
| 1700 | 21.84 | 5.0° | +30.0 | 4.20 | Donus basliyor |
| 1723 | 22.44 | 13.7° | +29.8 | 3.80 | Iyi donuyor |
| 1740 | 22.72 | 20.0° | +26.5 | 3.70 | Donus devam |
| 1760 | 23.07 | 28.2° | +23.9 | 3.58 | **SAG'dan hemen once** |

Bu fazda her sey normal. Arac kuzey-doguya donerek ilerliyor, x hala merkezin batisinda.

### Faz 3: KRITIK OLAY - SAG KOMUTU (Row 1761, t=16:16:07.61)

**DIREKSIYON ANIDEN TERSINE DONDU!**

| Row | x | Yaw | Steer | Hiz | Karar |
|-----|------|------|-------|------|-------|
| 1760 | 23.07 | 28.2° | **+23.9** | 3.58 | normal |
| **1761** | **23.10** | **28.9°** | **-20.0** | **3.58** | **sag** |
| 1770 | 23.34 | 30.5° | -20.0 | 3.25 | sag |
| 1775 | 23.50 | 29.8° | -20.0 | 3.10 | normal |
| 1780 | 23.63 | 28.0° | -20.0 | 2.98 | sag (2. dalga) |
| 1790 | 23.80 | 24.5° | -20.0 | 2.85 | normal |
| 1803 | 23.98 | 20.5° | -20.0 | 2.91 | normal |

**Ne oldu:**
- SAG komutu 2 kez ardi ardina geldi (~200ms arayla)
- Toplam 1.1 saniye boyunca direksiyon -20 derecede (SAGA) kaldi
- Bu surede yaw **30.5°'den 20.5°'ye DUSTU** (donus tersine cevirildi!)
- x konumu 23.07'den 23.98'e artti (merkeze dogru ilerledi ama donmeden)

### Faz 4: PID Gec Kaliyor (Row 1803-1860)

SAG bittikten sonra PID tekrar kontrolu aldi, ama steer hala -20.0'de kaliyordu:

| Row | x | Yaw | Steer | Hiz | Aciklama |
|-----|------|------|-------|------|----------|
| 1803 | 23.98 | 20.5° | -20.0 | 2.91 | Son hedef (24.53, -12.22) alindi |
| 1830 | 24.38 | 14.5° | -20.0 | 2.70 | Hala saga doniyor! |
| 1843 | 24.58 | 11.8° | -20.0 | 2.60 | Merkezi gecti (x>24.5) |
| 1860 | 24.83 | 7.8° | **+29.5** | 2.51 | PID nihayet sola cevirdi |

**Sorun:** PID duzeltme baslattigi anda arac x=24.83 pozisyonunda, yaw sadece 7.8 derece
(neredeyse doguya bakiyor), ve hiz 2.5 km/h. Artik cok gec - bu hiz ve konumdan max steer
ile duvari onlemek cok zor.

### Faz 5: Umitsiz Kurtarma Denemesi (Row 1860-2275)

| Row | x | Yaw | Steer | Hiz | Aciklama |
|-----|------|------|-------|------|----------|
| 1883 | 25.08 | 9.9° | +30.0 | 1.76 | Max sola donuyor ama cok yavas |
| 1943 | 25.58 | 20.9° | +30.0 | 1.51 | Donuyor ama x hala artiyor |
| 2003 | 25.97 | 30.5° | +30.0 | 1.40 | Onceki kaza noktasini gecti! |
| 2063 | 26.30 | 40.1° | +30.0 | 1.42 | Duvar yaklastyor |
| 2143 | 26.65 | 53.9° | +30.0 | 1.60 | Hizlanmaya basliyor |
| 2223 | 26.87 | 70.0° | +28.7 | 1.88 | |
| 2263 | 26.89 | 79.5° | +23.4 | 2.29 | Duvara 10cm |
| **2275** | **26.91** | **79.9°** | **+30.0** | **0.39** | **CARPMA!** |

Arac max direksiyon (+30) ile donmeye calisiyordu ama:
- Hiz cok dusuktu (1.4-2.0 km/h) → tekerlek kuvveti yetersiz
- x=24.83'ten basladigi icin 2.1m yol kat etmesi gerekiyordu → yetersiz mesafe

### Faz 6: Duvarda Takilma (Row 2275-2519)
- Konum: x=26.915, y=-17.311
- Yaw: ~80 derece
- Hiz: 0.01 km/h
- Throttle: 15.6 (gaz veriyor)
- Steer: 30.0 (max sol)
- **Sonsuza kadar takilmis durumda**

---

## C) IKI KAZA KARSILASTIRMASI

| Parametre | 21 Subat (Kaza 1) | 22 Subat (Kaza 2) |
|-----------|-------------------|-------------------|
| Kok neden | Weighted heading hatasi | SAG komutu virajda |
| Carpma x | 25.97 | 26.92 |
| Carpma y | -17.43 | -17.31 |
| Son yaw | 81.9° | 80.2° |
| Merkezden sapma | 1.47m | 2.42m |
| Carpma hizi | 4.3 km/h | 0.39 km/h |
| Kurtarma sansi | Direksiyon yetersiz (14→5°) | Direksiyon max (+30) ama gec |
| Takilma suresi | ~330 row | ~240 row |

---

## D) KOK NEDEN: SAG KOMUTU VIRAJ ORTASINDA

### Neden SAG Komutu Geldi?
- `/karar` topic'inden "sag" mesaji viraj sirasinda yayinlandi
- Bu muhtemelen karar algoritmasi (engel tespit) tarafindan gonderildi
- Karar algoritmasi aracin virajda oldugunu BILMIYORDU

### Neden Bu Kadar Yikici?
1. **Direksiyon aninda tersine dondu:** +24° → -20° = 44 derecelik ani degisim
2. **Donus momentumu yok edildi:** Yaw 30°'den 20°'ye dustu (10° geri gitti)
3. **PID tepki suresi:** ~1 saniye boyunca PID direksiyon kontrolunu geri alamadi
4. **Geri donus noktasi yoktu:** Arac merkezin dogusuna gectiginde max steer bile yetersiz

### Neden Onceki Kazada Olmadi?
Onceki kaza suruslerinde SAG komutu ya virajda gelmedi ya da gelmedigi icin sorun
gozlemlenmedi. Bu testde tam kotu zamanda geldi.

---

## E) UYGULANAN DUZELTME

### control.py Degisiklikleri:

**1. `_start_lane_change()` - Virajda serit degistirmeyi engelle:**
```python
def _start_lane_change(self, direction):
    # Virajda serit degistirmeyi engelle
    if self.dynamic_target is not None:
        heading_err = abs(self._heading_error(...))
        if heading_err > math.radians(20):
            self.logger.log("SERIT DEGISTIRME REDDEDILDI: Virajda")
            return
    self.lane_change_active = True
    ...
```

**2. `_get_lane_change_steer()` - Aktif serit degistirmeyi virajda iptal et:**
```python
def _get_lane_change_steer(self):
    ...
    # Virajda serit degistirmeyi iptal et
    if self.dynamic_target is not None:
        heading_err = abs(self._heading_error(...))
        if heading_err > math.radians(25):
            self.lane_change_active = False
            self.logger.log("SERIT DEGISTIRME IPTAL: Viraj algilandi")
            return None
    ...
```

### Duzeltme Mantigi:
- Heading error > 20° ise yeni serit degistirme BASLATILMAZ
- Heading error > 25° ise aktif serit degistirme IPTAL EDILIR
- 20° esigi: Aracin aktif bir virajda oldugunu gosteren yeterli aci
- 25° esigi: Biraz daha yuksek (histerezis), gereksiz iptal onleme

---

## F) GELECEK ICIN ONERILER

### Kisa Vadeli:
1. **Karar algoritmasi viraj bilinci:** `/karar` topic'ine viraj bilgisi eklenebilir
2. **Duvara yakinlik alarmi:** x pozisyonu tunel duvarına 0.5m'den fazla yaklasinca
   tum override komutlarini (SAG/SOL) otomatik iptal et

### Orta Vadeli:
3. **Komut onceliklendirme:** Guvenlik-kritik komutlar (viraj tamamlama) her zaman
   serit degistirmeden oncelikli olmali
4. **Duvar algilama entegrasyonu:** Sensorden gelen duvar mesafesi bilgisi PID'e
   ek yanal duzeltme olarak eklenebilir

---

## SONUC

Bu kaza, ilk kazadan tamamen farkli bir kok nedene sahip. Ilk kazadaki `_heading_error()`
duzeltmesi dogru yapilmisti. Bu kazanin nedeni, karar sisteminin viraj sirasinda serit
degistirme komutu gondermesi ve kontrol sisteminin bunu sorgulamadan uygulamasidir.

Uygulanan duzeltme, kontrol sistemine "virajda serit degistirme yapma" kurali ekleyerek
bu tarz kazalari onleyecektir.

---
*Bu rapor, 22 Subat 2026 tarihli surus loglarinin analizi sonucu hazirlanmistir.*
