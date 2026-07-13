#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
try:
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import Pose2D
except ImportError:
    rospy = None
    String = None
    Pose2D = None
import math
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
import heapq
import networkx as nx

# Kalıcı tanı logu (opsiyonel) — import edilemezse node yine çalışır.
try:
    from hedef_logger import HedefLogger
except Exception as _e:  # noqa: BLE001
    HedefLogger = None
    sys.stderr.write(f"[hedef_yoneticisi] hedef_logger yok, loglama kapalı: {_e}\n")

YESIL  = "\033[92m"
KIRMIZI = "\033[91m"
SARI   = "\033[93m"
SIFIRLA = "\033[0m"

GOREV_GEOJSON = {
  "type": "FeatureCollection",
  "features": [
    {"type": "Feature", "properties": {"name": "durak_1", "description": "1. Durak (Eski Park Cebi - Sağ)"},
     "geometry": {"type": "Point", "coordinates": [37.0, -4.5]}},
    {"type": "Feature", "properties": {"name": "durak_2", "description": "2. Durak (Yeni Park Yolu - Sol)"},
     "geometry": {"type": "Point", "coordinates": [7.35, -13.9]}},
    {"type": "Feature", "properties": {"name": "durak_1_donus", "description": "Ara durak_1 (park dönüşü için tekrar)"},
     "geometry": {"type": "Point", "coordinates": [37.0, -4.5]}},
    {"type": "Feature", "properties": {"name": "park", "description": "Park (Spot lane, demo node 7)"},
     "geometry": {"type": "Point", "coordinates": [-21.78, -13.92]}}
  ]
}

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER  —  hepsi burada, kolay düzenleme için
# ════════════════════════════════════════════════════════════════════════

# ==============================================================================
#   SENARYO PROFİLLERİ (Yarışmadaki etaplar için hızlı geçiş)
# ==============================================================================
# Seçenekler: "GENEL", "SOLLAMA_ETABI", "YOLCU_ALMA_ETABI"
SECILEN_SENARYO = "GENEL"

SENARYO_AYARLARI = {
    "GENEL": {
        "TURN_AWARE_AKTIF": True,
        "BLOK_SERT_AKTIF": True,
        "IKI_YONLU_DURAK_AKTIF": True,
        "SLALOM_ENJEKSIYON_AKTIF": True,
        "HESAP_KILIDI_AKTIF": True,
        "B_PLAN_YAW_ROTA_AKTIF": False,
        "SADECE_ENGELDE_YENIDEN_PLANLA": False,
        "SNAP_YAW_AGIRLIK": 15.0,
        "KONUM_FILTRE_AKTIF": True,
        "KONUM_FILTRE_ALPHA": 0.85,
        "KONUM_JUMP_LIMIT_MS": 6.0,
    },
    "SOLLAMA_ETABI": {
        "TURN_AWARE_AKTIF": True,
        "BLOK_SERT_AKTIF": True,
        "IKI_YONLU_DURAK_AKTIF": False,   # Sollama etabında duraklara iki yönlü girilmesine gerek yok
        "SLALOM_ENJEKSIYON_AKTIF": True,  # Slalom/Overtake aktif
        "HESAP_KILIDI_AKTIF": True,       # Manevra sırasında U dönüşünü engellemek için kilit aktif
        "B_PLAN_YAW_ROTA_AKTIF": False,
        "SADECE_ENGELDE_YENIDEN_PLANLA": True,
        "SNAP_YAW_AGIRLIK": 25.0,
        "KONUM_FILTRE_AKTIF": True,
        "KONUM_FILTRE_ALPHA": 0.85,
        "KONUM_JUMP_LIMIT_MS": 6.0,
    },
    "YOLCU_ALMA_ETABI": {
        "TURN_AWARE_AKTIF": True,
        "BLOK_SERT_AKTIF": False,         # Durak cepleri çevresinde sert silme yerine ceza puanı (fail-safe)
        "IKI_YONLU_DURAK_AKTIF": True,    # Duraklara her iki yönden de girilebilmesi kritik
        "SLALOM_ENJEKSIYON_AKTIF": False, # Durak yaklaşımında ani şerit değiştirmeyi kapat
        "HESAP_KILIDI_AKTIF": False,      # Rota kilitlenmesin, sürekli en kısa yolu arasın
        "B_PLAN_YAW_ROTA_AKTIF": False,
        "SADECE_ENGELDE_YENIDEN_PLANLA": False,
        "SNAP_YAW_AGIRLIK": 15.0,
        "KONUM_FILTRE_AKTIF": True,
        "KONUM_FILTRE_ALPHA": 0.85,
        "KONUM_JUMP_LIMIT_MS": 6.0,
    }
}

# Seçilen profile göre aktif ayarları çek
_aktif_ayarlar = SENARYO_AYARLARI.get(SECILEN_SENARYO, SENARYO_AYARLARI["GENEL"])

# ── GUI ──────────────────────────────────────────────────────────────────
ENABLE_GUI          = True   # Matplotlib penceresi (False = headless/penceresiz)

# ── Start seçimi & sapma / reroute (Faz 1-5) ─────────────────────────────
ILERI_MESAFE_M      = 2.0    # start + sapma "burun" noktası: aracın yaw yönünde bu kadar ileri (m)
# BLOK (sollama/kenar_blok) aktifken start ileri-projeksiyonu: 0 → path aracın
# GERÇEK konumundan başlar. Aksi halde 2m ileri-projeksiyon, aracın o anki
# konumundaki karşı-şerit GİRİŞ crossing'ini atlayıp girişi engele ~2m yaklaştırır
# → yanal manevra cone'da tamamlanmaz, açıklık engelin İLERİSİNDE oluşur
# (engel geç algılanırsa açıklık ~0 → çarpma). Blokta lead-in kritik → 0.
ILERI_MESAFE_BLOK_M = 0.0    # blok aktifken start ileri-projeksiyonu (m); lead-in için 0
SAPMA_ESIK_METRE    = 2.5    # burun en yakın WP'den bu kadar uzaksa → reroute (debounce sonrası) (m)
SAPMA_TEMIZ_METRE   = 1.5    # histerezis: burun bu kadarın ALTINA inince sayaç sıfırlanır (band 1.5..2.5) (m)
SAPMA_DEBOUNCE_SURE = 1.5    # sapma reroute için bu kadar saniye KESİNTİSİZ süregelmeli (s)
GOREV_YAKINLIK_M    = 2.0    # bu mesafede durak/görev tamamlandı sayılır (m)
YON_FILTRE_ACIISI   = math.pi - 0.3   # start'tan geri-yön kenar filtre açısı (~162°)

# ── Map-matching (Faz 3) ─────────────────────────────────────────────────
MATCH_PENCERE       = 6      # current_wp_index ileri snap penceresi (kaç WP)
MATCH_KORIDOR_M     = 4.5    # snap koridoru — graf max kenar 6.13m → yarı 3.06m'den BÜYÜK olmalı (m)

# ── CEZA PUANLARI (0-100)  —  rota / şerit tercihleri ────────────────────
# Eşleme:  kenar_agirligi = mesafe * (1 + CEZA/100 * CEZA_ETKI)
#   CEZA_ETKI=4 ile:  0p→1.0x · 8p→1.32x · 15p→1.6x · 25p→2.0x · 50p→3.0x · 90p→4.6x · 100p→5.0x
#   (eski 6x slalom ≈ 125p idi — çok abartı; 0-100 aralığına çekildi)
CEZA_ETKI             = 4.0  # global ölçek: ceza puanının çarpana etkisi

# Aktif (D* grafında kullanılıyor):
CEZA_DUZ_SERIT        = 0    # düz şeritte sürüş (1.0x) — referans, en tercih edilen
CEZA_BAGLANTI         = 8    # bağlantı/dönüş yolu (1.32x) — düz şeridi hafif tercih et
# Placeholder — DİNAMİK akış için, henüz hiçbir canlı kod yoluna BAĞLI DEĞİL
# (karar↔hedef / control arayüzü gelince kullanılacak; şimdi değiştirmek bir şeyi etkilemez):
CEZA_SERIT_DEGISTIRME = 15   # şerit değiştirme/sollama (1.6x) — karar-güdümlü
# CEZA_TERS_YON: YALNIZ eski ağırlık-şişirme bloğunda (_agirlik_blok, BLOK_SERT_AKTIF=False)
# aktif → o modda engel kenarını çarpımsal şişirir. Varsayılan SERT blokta kullanılmaz.
CEZA_TERS_YON         = 90   # ters yön / geri sürüş (4.6x) — _agirlik_blok fallback'ı

# ── İki-yönlü durak erişimi ──────────────────────────────────────────────
# Tek-yönlü graf, durağa kuş uçuşu yakınken aracı onca yolu dolaştırabiliyordu
# (ör. (21,10)→durak_1: düz 19.8m ama tek-yön rota 61.5m loop; (33,-1)→durak_1
# 86m). Çözüm: her durağın R_DURAK_M yarıçapındaki TÜM kenarlar (cep içi lane +
# giriş/çıkış connection'ları) iki yönlü yapılır → durağa her iki uçtan girilebilir.
# Ana tek-yön loop (A,B,C,...) bundan uzakta olduğu için DOKUNULMAZ; cep çıkmaz
# yapı olduğundan iki yönlü olması ana trafiğe yeni geçiş rotası açmaz.
# Ölçüm: (21,10) 61.5→31.3m, (33,-1) 86→6m. (Kullanıcı: "durağa illaki o
# girişinden girmek gerekmez, iki yönlü de girilebilir".)
IKI_YONLU_DURAK_AKTIF  = _aktif_ayarlar["IKI_YONLU_DURAK_AKTIF"]
R_DURAK_M              = 7.0    # durak goal'ünün bu yarıçapındaki kenarlar iki yönlü (m)

# ── Bisiklet-modeli dönüş cezası (kolay-dönülebilir rota tercihi) ─────────
# Keskin dönüşler açı + mesafe ile cezalanır (60°/3m ≫ 60°/10m). Yönlü-kenar
# durumları üzerinde turn-aware arama (find_path_turn_aware): her köşede
# Δθ (baş açısı değişimi) yerel mesafeye bölünür → eğrilik κ=Δθ/d → gereken
# direksiyon δ=atan(L·κ). Yumuşak dönüş ucuz, keskin dönüş pahalı, U-dönüşü
# (cusp) neredeyse yasak. Böylece iki-yönlü durakta DÖNÜLEBİLİR girişi seçer.
TURN_AWARE_AKTIF       = _aktif_ayarlar["TURN_AWARE_AKTIF"]
ARAC_DINGIL_M          = 1.86   # Bee1 dingil mesafesi (wheelbase, m)
ARAC_MAX_DIREKSIYON    = math.radians(32.5)  # Bee1 maks teker açısı (iç, rad)
DONUS_CEZA_AGIRLIK     = 1.5    # (δ/δ_max)² → dönüş cezası (m-eşdeğeri); yumuşak tercih
DONUS_CEZA_MAX         = 4.0    # tek dönüş cezası üst sınırı (m) — büyük mesafe kazancını ezmesin
DONUS_CUSP_ESIK        = math.radians(150)   # bu açının üstü U-dönüşü → ağır ceza (pratikte yasak)
DONUS_CUSP_CEZA        = 1000.0 # cusp eşiği üstü dönüş için ek ceza (m)

# ── Karar → hedef komutu (/hedef_komut) — sollama / kenar bloğu ───────────
# karar (BT) sollamaya çıkınca engelin DÜNYA konumunu /hedef_komut ile yollar
# (String: "komut;taraf;x;y;etiket;yaricap"). Komutlar: sollama / kenar_blok
# (engeli blokla) · kenar_serbest (bloku kaldır) · replan (yalnız yeniden hesapla).
# Hedef tarafı (plan §3.2): engeli SERT SİLMEZ — ağırlığını ŞİŞİRİR
# (ceza_carpani(CEZA_TERS_YON), recalc'ta find_path öncesi uygula, finally'de
# geri al) → alternatif varsa planlayıcı kenardan dolanır; yoksa düşük-bozulmayla
# yine geçer (fail-safe). Sert silme Faz1-5 debounce'la çakışıp restore anında
# aracı yanlış şeride sokabilirdi. Yanal manevranın kendisini control yapar;
# hedef kapalı-döngüyü (engeli rota-körü tekrar üretmeme + dönüş zamanı) kapatır.
HEDEF_KOMUT_AKTIF      = True
BLOK_TTL_S             = 3.0    # sollama bloğu bu kadar sn tazelenmezse düşer (karar ~1s'de tazeler)
BLOK_MARJIN_M          = 1.5    # blok yarıçapına eklenen pay (m): araç ön çıkıntısı + kenar uzunluğu
# Duba KONUM güncellemesi (kullanıcı 2026-06-27 "duba konumu tetiklesin güncellemeyi"):
# Aynı duba (≤2m) tekrar gelince blok konumu HER ZAMAN en tazeye güncellenir; konum bu
# kadar TAŞINDIYSA reroute de tetiklenir. Yoksa ilk (gürültülü, ör. track-dışı) konum
# bloğun TTL'i (15s) boyunca donuyor, düzelmiş on-lane konum gelse de sollama gecikiyordu
# (canlı 211236Z: duba (21.23,-35.95)→(20.53,-34.26) 1.83m taşındı ama 15s güncellenmedi).
KONUM_DEGISIM_ESIK_M   = 0.5    # duba konumu bu kadar taşınınca güncelle + reroute (m)

# ── SERT BLOK (engel çemberi) — kullanıcı kararı 2026-06-26 ───────────────
# Kullanıcı: "karardan gelen engelin konumundan bir çember çiz ve 1m yarıçapındaki
# bütün waypointleri kullanılamaz yap → ceza puan sisteminde mecbur ters şeritten
# gidecek rota bulacağız." Eski ağırlık-şişirme (BLOK_SERT_AKTIF=False, aşağıda)
# SONLU ceza verdiğinden iki sorun çıkardı: (a) cone'a TOPLAMSAL BLOK_EK_CEZA=50
# eklenince yakındaki yol ~50m pahalanıyor → planlayıcı kısa bir alternatif yerine
# FARKLI BİR SOKAĞA sapabiliyordu ("slalom yerine başka yola döndün"); (b) ters
# şeritte KALMA bedava (CEZA_TERS_KALMA=0) olduğundan araç karşı şeritte gereğinden
# UZUN kalabiliyordu ("çok fazla ters şeride geçiyoruz"). SERT blok bunu kapatır:
# engelin yarıçapındaki kenarları grafdan GEÇİCİ SİLER → ileri şerit engelde
# KESİLİR; planlayıcı yalnız engelin ÇEVRESİNDEKİ karşı-şerit crossing'leriyle
# (ceza puanı altında) dolanabilir → tek giriş + tek çıkış, dar/yerel slalom.
# Farklı sokak cazip değil (toplamsal ceza yok); karşı şeritte kalma da yalnız
# engel boyunca (ÇIKMA pahalı → erken döner). Geri-alınabilir (recalc finally).
BLOK_SERT_AKTIF        = _aktif_ayarlar["BLOK_SERT_AKTIF"]   # True → engel çemberini SERT sil (varsayılan); False → eski ağırlık-şişirme
BLOK_YARICAP_M         = 1.0    # engel çemberi yarıçapı (m); etkin = max(engel_r, bu). Kullanıcı: 1m.

# ── HESAPLAMA KİLİDİ — sollama-kararı + sağ-şerit (kullanıcı kararı 2026-06-26/27) ──
# Kullanıcı: "sollama yapmaya KARAR verdiği AN kilitlesin; SAĞ şeride döndüğü AN
# kilitlemesi bitsin." (Eski "yol ayrımı geçince kilitle" ÇOK GEÇ tetikliyordu —
# A-şeridi ilk ayrımı x=10, dubalardan sonra.)
# Mekanizma: recalc bir SOLLAMA rotası commit ettiğinde (rota KARŞI/sol şeride
# giriyorsa → sollama kararı verildi) rota HESABI KİLİTLENİR (_hesap_kilitli=True).
# Kilitliyken planlayıcı YENİDEN HESAPLAMAZ → araç sol şeride çıkmışken (mid-manevra)
# cusp/U-dönüşü re-plan'ı biter. Kilit, araç SOL şeride girip SAĞ (forward) şeride
# DÖNÜNCE açılır + o stabil konumdan tek temiz recalc (sonraki dubayı sağ şeritten
# planlar). Fail-safe: sol şeritte takılırsa DURMA_BEKLEME_SN durunca açılır.
HESAP_KILIDI_AKTIF     = _aktif_ayarlar["HESAP_KILIDI_AKTIF"]   # False → kilit yok (her tetikte recalc — eski davranış)
DURMA_BEKLEME_SN     = 15.0   # fail-safe: sol şeritte takılırsa durup bu kadar bekleyince aç (s)
DURMA_YARICAP_M      = 2.5    # araç bu yarıçapta DURMA_BEKLEME_SN kalırsa "durdu" (m; ≈eski 0.15 m/s)
DURMA_HIZ_ESIK_MS    = 0.15   # ardışık poz'dan tahmini hız bunun altıysa "durdu" sayılır (m/s)
# Kilit açma (sağ-şerit/15s) recalc COOLDOWN'u: araç şeritler arasında yanal salınınca
# _sag_seritte titreyip kilit aç-kapa yapıp her açılışta recalc tetikliyordu (canlı
# 203548Z: 426 recalc / 1s churn). Açma recalc'ı bu süreden sık tetiklenmez.
KILIT_COOLDOWN_SN    = 1.5    # kilit açma recalc'ları arası min süre (s) — churn engeli
KILIT_BYPASS_COOLDOWN_SN = 0.3 # bypass recalc'ları arası min süre (s) — ardışık duba planlaması için kısa tutulur
# Şerit tespiti histerezisi: araç forward(sağ) ve karşı(sol) şerit düğümlerine eşit
# mesafedeyse (boundary) durumu DEĞİŞTİRME (öncekini koru) → boundary'de flicker biter.
SERIT_MARJIN_M       = 0.6    # sağ/sol şerit kararını değiştirmek için gereken net mesafe farkı (m)

# ── Karşı-şerit (slalom) enjeksiyonu — plan §16 (S-A/S-B/S-C) ──────────────
# Yalnız ağırlık şişirme rotayı SAPTIRMIYORDU (§10.3 bilinen kısıt): tek-yön
# şeritte alternatif yoksa engel kenarı pahalanır ama planlayıcı yine düz geçer.
# Çözüm (§16, hedef-yönlü karşı-şerit reroute): kenar_blok/sollama geldiğinde
# engelin ÇEVRESİNE iki tür kenar GEÇİCİ enjekte edilir:
#   1) crossing (A↔B geçiş, "ters şerite ÇIKMA")  → CEZA_TERS_CIKIS (PAHALI)
#   2) karşı-şerit boylamasına segment (B'de ileri seyir, "ters şeritte KALMA")
#                                                  → CEZA_TERS_KALMA (UCUZ)
# Ayrım (kullanıcı içgörüsü): çıkış riskli → az sayıda yap (tek giriş+çıkış,
# zig-zag değil); bir kez çıktıysan engeli geçene kadar karşı şeritte KAL
# (yanal açıklık). VARSAYILAN SERT blokta (BLOK_SERT_AKTIF=True) ileri şerit zaten
# silindiğinden engelden geçmek imkânsız → bu crossing'ler tek detour. (Eski ağırlık
# modunda — _agirlik_blok — bloklu kenara ayrıca TOPLAMSAL BLOK_EK_CEZA eklenir;
# SERT blokta BLOK_EK_CEZA kullanılmaz.) Taban graf TEK-YÖNLÜ kalır: enjekte yalnız
# (a) blok aktifken, (b) engel çevresinde (R), (c) recalc sonunda geri alınır →
# "ters şeritten kolay yol" bug'ı (§0) dönmez. Sürülemez U-dönüşlerini turn-aware
# cusp kapısı eler → enjeksiyon YALNIZ TURN_AWARE iken.
SLALOM_ENJEKSIYON_AKTIF = _aktif_ayarlar["SLALOM_ENJEKSIYON_AKTIF"]
# SLALOM_YALNIZ_GEREKINCE (kullanıcı 2026-06-26 "duba sol şeritte → hep sollamaya
# çalışıyorsun"): enjeksiyonu recalc'ta ÖNCE DENEMEZ. SERT blok uygulanır, rota
# enjeksiyonSUZ aranır → engel KARŞI ŞERİTTEYSE (kendi şeridini tıkamıyorsa) rota
# zaten bulunur → karşı şeride GEÇİLMEZ (sollama YAPILMAZ, dubanın olduğu şeride
# girilmez). YALNIZ kendi şerit gerçekten tıkalıysa (rota None) crossing enjekte
# edilip tekrar aranır. Böylece "her duba görünce sollama" davranışı, "yalnız
# kendi şeridindeki dubada dar/yerel sollama"ya iner. False → eski (her blokta
# enjekte) davranış. Reversible.
SLALOM_YALNIZ_GEREKINCE = True
# "Engel kendi rotada mı" eşiği = SERT blok çemberiyle BİREBİR aynı: max(engel_r,
# BLOK_YARICAP_M). Engel çemberi bloksuz baseline rotayı kesiyorsa rotayı tıkıyor
# sayılır → sollama gerek. Aynı çember sert_blok'ta rota kenarını da sileceğinden
# tutarlı (eşik küçük seçilse büyük-r engelde "rota dışı" derken sert_blok rotayı
# kesip enjeksiyonsuz farklı-sokak yapabilirdi). r≈1m default'ta şerit ayrımı ~2m
# olduğundan (A y≈-34.27 ↔ B y≈-32.29) kendi şerit (~0m) ile karşı şeridi (~2m) net ayırır.
CEZA_TERS_CIKIS         = 90    # ters şerite ÇIKMA (crossing) cezası (4.6x) — PAHALI: az geçiş
CEZA_TERS_KALMA         = 0     # ters şeritte KALMA (karşı-şerit ileri seyir) cezası (1.0x) — UCUZ: solda kal
BLOK_EK_CEZA            = 50.0  # bloklu kenara TOPLAMSAL ceza (m) — detour'u baskın yap; sonlu (fail-safe)
SLALOM_ENJEKSIYON_R     = 6.0   # crossing/segment endpoint'i engele bu kadar yakınsa enjekte (m) — 8→6 azalt (lokalize slalom)
# Giriş crossing'i engeli SIYIRMASIN ("bir waypoint önce dön", kullanıcı 2026-06-24):
# tek-dip girişi (A→B diyagonali) engele AT başlıyordu → engeli ~0.06m sıyırıp
# (path engel DÜĞÜMÜnden geçince) karar `engel_blokaj` verip durduruyordu. Çözüm:
# (a) cone'a r+KENAR_GUVENLI_M'den yakın geçen crossing'e toplamsal SIYIRMA cezası
# (yakınlıkla orantılı), (b) o crossing'in BİR ÖNCEKİ düğümden (predecessor)
# başlayan versiyonunu da enjekte et → planlayıcı engeli daha geniş geçen, bir
# waypoint ÖNCE dönen girişi seçer (engel düğümünü atlar). Mid-cone'da giriş zaten
# açık → ceza 0, değişmez. Köşe-cone (şeridin ilk düğümü) açıklık 0.06→1.0m.
KENAR_GUVENLI_M         = 1.2   # crossing engele bundan + r kadar yaklaşırsa "sıyırıyor" sayılır (m)
CEZA_SIYIRMA_M          = 80.0  # sıyıran crossing'e metre-başına toplamsal ceza (geniş geçeni tercih ettir)

# ── Yolcu Alma Etabı Karşı Şerit Geçiş Cezaları ──
CEZA_KARSIN_GECIS       = 50.0  # Sağ şeritten karşı şeride geçiş (crossing) cezası (metre-eşdeğeri)
CEZA_KARSIN_SEYIR       = 10.0  # Karşı şeritte seyretme cezası (metre-eşdeğeri / kenar başına)

# ── B PLANI (varsayılan KAPALI) — yaw-tabanlı sentetik slalom rotası ───────
# Kullanıcı (2026-06-26): "gerekirse slalom noktasına geldiğimizde karardan bir
# bilgi bekle ve aracın yaw değerine bağlı yeni rota oluştur. bunu B planı yapalım,
# güncelde kullanma ama gerektiğinde açabilelim." A planı (SERT blok + karşı-şerit
# enjeksiyon) GRAF-tabanlıdır ve normalde yeterli. B planı GRAF-BAĞIMSIZ bir
# açık-döngü kaçış: karar engeli bildirince (aktif blok) aracın YAW'ına dik
# (sol = karşı şerit) sentetik yanal-offset waypoint'leri (giriş/orta/çıkış)
# üretip engeli sollar, sonra çıkış noktasına en yakın graf düğümünden hedefe
# normal rotayla devam eder. VARSAYILAN KAPALI; yalnız A planı sahada yetersiz
# kalırsa elle açılır (B_PLAN_YAW_ROTA_AKTIF=True). Kapalıyken hiçbir etkisi yok.
B_PLAN_YAW_ROTA_AKTIF  = _aktif_ayarlar["B_PLAN_YAW_ROTA_AKTIF"]  # True → A planı yerine yaw-tabanlı sentetik slalom kullan
B_PLAN_OFFSET_M        = 2.3    # karşı şeride yanal offset (m, ~pist genişliği)
B_PLAN_OFFSET_MARJIN_M = 1.0    # engel r'sine eklenen pay → offset = max(r+pay, OFFSET_M)
B_PLAN_LEAD_M          = 3.0    # offset waypoint'lerin engel önü/arkası mesafesi (m)

# ── ROTA KİLİTLEME (Yalnızca engelde yeniden planla) ───────────────────────
SADECE_ENGELDE_YENIDEN_PLANLA = _aktif_ayarlar["SADECE_ENGELDE_YENIDEN_PLANLA"]  # True → İlk rotadan 3sn sonra rotayı sabitle, engel yoksa recalculate yapma

# ── AÇISAL SNAP CEZA AĞIRLIĞI ──────────────────────────────────────────────
SNAP_YAW_AGIRLIK = _aktif_ayarlar["SNAP_YAW_AGIRLIK"]  # Açısal snap ceza katsayısı (ters yönlü düğüm engelleme)

# ── GPS KONUM VE YAW FİLTRELEME ───────────────────────────────────────────
KONUM_FILTRE_AKTIF  = _aktif_ayarlar["KONUM_FILTRE_AKTIF"]   # True -> EMA + Outlier filtrelemeyi aktif yap
KONUM_FILTRE_ALPHA  = _aktif_ayarlar["KONUM_FILTRE_ALPHA"]   # 0.0 - 1.0 arası yumuşatma katsayısı (küçük -> daha yumuşak)
KONUM_JUMP_LIMIT_MS = _aktif_ayarlar["KONUM_JUMP_LIMIT_MS"]  # Ardışık okumalar arası max hız sınırı (outlier tespiti için, m/s)

# ── Görselleştirme Renk Paleti ───────────────────────────────────────────
BG        = '#2e2d2a'
PANEL_BG  = '#272622'
EDGE_COL  = '#3d3c38'
NODE_COL  = '#4a4945'
ROTA_MAIN = '#ff4d5e'
ROTA_GLOW = '#e63946'
ROTA_SHIN = '#ff9aa2'
ROTA_PAST = '#e63946'
DURAK_COL = '#00d9ff'
WP1_COL   = '#f4d03f'
WP2_COL   = '#e040fb'
ARABA_COL = '#39ff14'
TEXT_COL  = '#7a7a6e'
