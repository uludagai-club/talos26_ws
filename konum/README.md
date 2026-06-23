# Araç Konum Yöneticisi (Talos)

Bu proje, Gazebo simülasyon ortamındaki araç için konum takibi sağlayan Python tabanlı kontrolcü kodunu içerir.

## Dosyalar
* **konum_yoneticisi.py:** Aracın konum verilerini işleyen ve yönlendiren ana Python kodu.
* **arac_hud.py:** Aracın konum verilerini (X, Y) ve yönelimini (Pusula/YAW) görselleştiren grafik arayüz (HUD).

## Gereksinimler
Bu kodun çalışması için aşağıdaki ortamların hazır olması gerekmektedir:
* ROS (Robot Operating System)
* Python 3
* Gazebo Simülasyonu

## Kurulum ve Kullanım

1. **Simülasyonu Başlatın:**
   Öncelikle Gazebo dünyasını ve aracı yükleyen launch dosyanızı çalıştırın.

2. **Kodu Çalıştırın:**
   Terminali açın, dosyanın bulunduğu dizine gelin ve şu komutu girin:

   ```bash
   python3 konum_yoneticisi.py
   ```

3. **HUD (Arayüz) Çalıştırın (Opsiyonel):**
   Aracın konum ve yönelimini anlık olarak grafiksel bir ekranda takip etmek için yeni bir terminal açıp dosyanın bulunduğu dizine gelin ve şu komutu girin:

   ```bash
   python3 arac_hud.py
   ```
