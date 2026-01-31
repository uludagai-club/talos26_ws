# ROS Hedef Yöneticisi (Target Manager)

Bu Python tabanlı ROS düğümü, otonom bir aracın GeoJSON formatında tanımlanmış X/Y koordinatlarını (metre cinsinden) sırasıyla takip etmesini sağlayan merkezi yönetim sistemidir.

[Image of ROS node communication architecture diagram showing publisher and subscriber nodes]

## 🚀 Genel Bakış

Sistem, bir görev listesini okur ve koordinatları `/hedef` konusu üzerinden yayınlar. Araç navigasyon katmanı hedefe ulaştığında bir onay mesajı gönderir ve sistem otomatik olarak bir sonraki görevi tetikler.

## 🛠 Çalışma Mantığı

1. **Koordinat İşleme:** `geojson_data` içindeki noktalar başlangıçta bir kuyruğa alınır.
2. **Yayın (Publish):** Hedefler `/hedef` konusuna JSON formatında basılır. `latch=True` sayesinde yeni bağlanan düğümler son hedefi kaçırmaz.
3. **Onay Mekanizması (Subscribe):** Araç `/gorev_durumu` konusuna `VARILDI` mesajı gönderdiğinde bir sonraki hedefe geçilir.

## 📡 Haberleşme Protokolü

### Yayınlanan Veri (Topic: `/hedef`)
Mesajlar JSON formatındadır:

```json
{
    "type": "Point",
    "target_name": "gorev_1",
    "coordinates": [-5.0, -34.0]
}
```
Beklenen Onay (Topic: /gorev_durumu)
Mesaj Tipi: std_msgs/String

Beklenen Değer: VARILDI

# 📂 Kurulum ve Çalıştırma
Gereksinimler
İşletim Sistemi: Ubuntu (20.04 tavsiye edilir)

ROS Sürümü: Noetic / Melodic

Dil: Python 3.x

Bağımlılıklar: rospy, std_msgs

Çalıştırma Adımları
Çalıştırma İzni: Scriptin bulunduğu dizine gidin ve izin verin:

```Bash

chmod +x hedef_yoneticisi.py
ROS Master: Bir terminalde Master düğümünü başlatın:

Bash

roscore
Düğümü Başlatın: Yeni bir terminalde ana düğümü çalıştırın:

Bash

python3 hedef_yoneticisi.py
```