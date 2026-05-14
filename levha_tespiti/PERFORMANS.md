# Levha (Trafik İşareti) Modeli — Performans Raporu

**Tarih:** 2026-05-14
**Test edilen model:** `İndirilenler/best.pt` → `levha_tespiti/yolov8_ros/scripts/best.pt` olarak kuruldu
**Donanım:** NVIDIA GeForce RTX 4060 (CUDA)

> Not: Bu rapor yalnızca **levha modelini** kapsar. İlk istekte geçen "şerit takibi" kısmı,
> indirilen modelin aslında bir levha (trafik işareti) modeli olduğu anlaşılınca kapsam dışı bırakıldı.

---

## 1. Özet

İndirilenlerdeki `best.pt`, **26 sınıflı bir trafik işareti tespit modeli** (yolov10n tabanlı). Mevcut
levha modeliyle (yolov10s) birebir aynı sınıf setine sahip ama daha küçük ve daha yeni. Model:

- **Hız:** Gerçek front-camera feed üzerinde **~159 FPS / 6.3 ms** — kamera hızının (25 FPS) ~6 katı, bol pay var.
- **Doğruluk:** Simülasyon kamerasında işaretleri doğru sınıf ve sıkı kutu ile tespit ediyor (örnek: `sola_donulmez` @ 0.95).
- **Karar bağlantısı:** `yolov8_ros_node_fixed.py` zaten `/trafik_levha` ve `/yaya_gecidi` topic'lerini `karar-node`'a yayınlıyor; yeni model bu hattı olduğu gibi besliyor.

---

## 2. Model Bilgileri

| Özellik | Değer |
|---|---|
| Dosya | `levha_tespiti/yolov8_ros/scripts/best.pt` (md5 `ea1ff0b5…`) |
| Mimari | YOLOv10n (nano), task: `detect` |
| Sınıf sayısı | 26 |
| Eğitim tabanı | `yolov10n.pt` |
| Eğitim tarihi | 2025-07-17 |
| Eğitim ayarları | 100 epoch, imgsz 640, batch 32 |
| Ultralytics sürümü (eğitim) | 8.3.155 |
| Dosya boyutu | ~5.75 MB (eski yolov10s model 16.5 MB idi) |

**Sınıflar (26):** `ada_etrafinda_donunuz`, `durak`, `iki_yonlu_yol`, `ileri_mecburi_yon`,
`ileri_ve_sola_mecburi_yon`, `lamba_kirmizi`, `lamba_yesil`, `park_yeri`, `ileri_ve_saga_mecburi_yon`,
`girisi_olmayan_yol`, `park_etmek_yasaktir`, `saga_mecburi_yon`, `saga_donulmez`, `sola_mecburi_yon`,
`ileriden_sola_mecburi_yon`, `sola_donulmez`, `yaya_gecidi`, `isikli_isaret`, `sagdan_gidiniz`,
`soldan_gidiniz`, `serit_duzenleme_saga_yonel`, `serit_duzenleme_sola_yonel`, `tunel`, `dur`,
`ileriden_saga_mecburi_yon`, `lamba_sari`.

---

## 3. Gömülü Eğitim/Validation Metrikleri

Checkpoint içine gömülü `train_metrics` (eğitim datasetinin validation böleninde ölçülmüş — gerçek
sim performansı değil, eğitim verisi performansıdır):

| Metrik | Değer |
|---|---|
| precision (B) | **0.9837** |
| recall (B) | **0.9827** |
| mAP@50 (B) | **0.9914** |
| mAP@50-95 (B) | **0.9028** |
| val/box_loss | 0.9547 |
| val/cls_loss | 0.3557 |
| val/dfl_loss | 1.6410 |
| fitness | 0.9117 |

Karşılaştırma — kaldırılan eski levha modeli (yolov10s) yedeklendi
(`levha_tespiti/yolov8_ros/scripts/best.pt.yolov10s.bak`): mAP@50 0.9917, mAP@50-95 0.9102.
İki modelin eğitim-seti metrikleri pratikte eşit; İndirilenler modeli daha küçük (nano) ve daha hızlı.

> ⚠️ Bu metrikler gerçek-dünya Türk trafik işareti datasetinde ölçüldü. Simülasyondaki işaretlerin
> görünümü farklı olduğundan, sim içi gerçek doğruluk bu sayılardan düşük olabilir — bkz. Bölüm 4.

---

## 4. Feed Benchmark (gerçek front-camera)

**Test girdisi:** `talos-sim-yedek/new.bag` → `/cart/front_camera/image_raw/compressed`
(2187 kare, 1280×960, 72 sn) — yani `traffic-node`'un canlıda tükettiği topic'in birebir aynısı.
Eşik: `conf=0.5` (node ile aynı), `imgsz=640`.

### Hız
| Metrik | Değer |
|---|---|
| Ortalama gecikme | **6.28 ms** |
| p50 / p95 / max gecikme | 5.95 / 8.01 / 16.73 ms |
| Verim (throughput) | **159.2 FPS** |

→ Kamera 25 FPS yayınlıyor; model ~6× pay bırakıyor. GPU'da gerçek zamanlı için fazlasıyla yeterli.

### Tespit
| Metrik | Değer |
|---|---|
| Tespit içeren kare | 314 / 2187 (**%14.4**) |
| Karar-ilişkili sınıf içeren kare | 21 / 2187 (**%1.0**) |
| Toplam tespit | 314 |
| Kare başına ort. tespit | 0.144 |
| Bir karedeki maks. tespit | 1 |

### Güven (confidence) dağılımı (tespit edilen kutular)
| Metrik | Değer |
|---|---|
| Ortalama | **0.776** |
| Medyan (p50) | 0.794 |
| Min / Max | 0.501 / 0.945 |

### Sınıf bazında tespit sayıları (bu kayıtta)
| Sınıf | Tespit |
|---|---|
| `sola_donulmez` | 281 |
| `ileriden_saga_mecburi_yon` | 21 |
| `durak` | 12 |

> Bu 72 sn'lik kayıtta araç yalnızca birkaç işaretin yanından geçtiği için sadece 3 sınıf göründü.
> 281 `sola_donulmez` tespiti, aynı fiziksel işaretin araç yaklaşırken ardışık karelerde tekrar
> tekrar görülmesidir — ayrı 281 işaret değil.

### Görsel doğrulama
En yüksek güvenli kare (idx 258): model `sola_donulmez` işaretini **0.95** güvenle, sıkı ve doğru
bir kutuyla yakaladı. Sınıf da doğru. Yani model sim'in front-camera görüntüsünde gerçekten çalışıyor.
(Çıktı: benchmark sırasında `/tmp/levha_best_detection.jpg`.)

---

## 5. Kısıtlar ve Notlar

- **`.mp4` dosyaları test için uygun değil.** `talos-yeni-test.mp4`, `kaza.mp4`, `roket.mp4` Gazebo
  **kuşbakışı GUI** kayıtları — front-camera feed'i değil. İşaretler bu açıdan minik nokta halinde
  kaldığından model hiçbir şey bulamadı (`talos-yeni-test.mp4` üzerinde %0 tespit). Bu bir
  **görüntü-uyumsuzluğu**, model hatası değil. Benchmark bu yüzden rosbag'deki gerçek front-camera
  feed'i ile yapıldı.
- **Etiketli sim test seti yok.** Bu yüzden sim içi gerçek mAP/precision/recall yeniden hesaplanamadı;
  Bölüm 3'teki metrikler eğitim datasetine aittir. Sim içi doğruluğu net ölçmek için
  `/cart/front_camera/image_raw` üzerinden etiketli bir set toplanması gerekir.
- Düşük tespit oranı (%14.4) beklenen bir durum: karelerin çoğunda görüş alanında işaret yok.
- 26 sınıfın sadece 3'ü bu kayıtta test edildi. Diğer sınıflar (özellikle `dur`, `lamba_*`,
  `yaya_gecidi`) için kapsamlı bir senaryo kaydı gerekir.

---

## 6. Karar (`karar-node`) Bağlantısı

Levha bilgisi karar düğümüne `fixes/yolov8_ros_node_fixed.py` üzerinden ulaşıyor:

- Girdi: `/cart/front_camera/image_raw` (Image)
- Çıktı: `/trafik_levha` (String) — `karar-node`'un `levha_callback`'i tüketir
- Çıktı: `/yaya_gecidi` (String) — `karar-node`'un `yaya_callback`'i tüketir
- Format: `"<SINIF>,<mesafe>,<x_offset>"` (örn. `DUR,3.5,0.8`) ya da `none`
- Sınıf eşleştirmesi `SINIF_ESLESTIRME` ile yapılır (`dur`/`lamba_kirmizi` → `DUR`,
  `saga_mecburi_yon` → `SAG`, vb.); `karar.py` bunu `levha_callback` → karar mantığında kullanır.

Yeni model bu hattı **olduğu gibi** besliyor; node kodunda veya `karar.py`'de değişiklik gerekmedi.

---

## 7. Yapılan Değişiklikler

| Dosya | Değişiklik |
|---|---|
| `levha_tespiti/yolov8_ros/scripts/best.pt` | İndirilenlerdeki yolov10n model ile değiştirildi |
| `levha_tespiti/yolov8_ros/scripts/best.pt.yolov10s.bak` | Eski yolov10s model yedeği (yeni) |
| `docker-compose.yml` | `traffic-node`'a model bind-mount + `DISPLAY` env + X11 mount eklendi |
| `fixes/yolov8_ros_node_fixed.py` | `cv2.imshow("Levha Tespit")` penceresi eklendi (DISPLAY guard'lı, `run()` döngüsü — lane-follower ile aynı kalıp) |
| `README.md` | "Hangi Dosyayı Değiştirince Ne Olur" tablosuna `best.pt` satırı eklendi |

**Etkili olması için:** `docker compose up -d traffic-node` (rebuild gerekmez — bind-mount).

### Görselleştirici (`Levha Tespit` penceresi)

`lane-follower`'daki `cv2.imshow` kalıbının aynısı. `traffic-node` artık `DISPLAY` env'i ve
`/tmp/.X11-unix` mount'u alıyor; node `DISPLAY` set ise anotasyonlu kamera karesini (sınıf + kutu)
bir pencerede gösterir, değilse sessizce sadece topic yayınına devam eder. `baslat.sh` zaten
`xhost +local:docker` çağırıyor ve `traffic-node` profile'sız olduğu için her zaman ayağa kalkar —
ekstra adım gerekmez. Pencerede `q` ile çıkılır.
