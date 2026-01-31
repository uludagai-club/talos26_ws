# Araç Konum Yöneticisi (Talos)

Bu proje, Gazebo simülasyon ortamındaki araç için konum takibi sağlayan Python tabanlı kontrolcü kodunu içerir.

## Dosyalar
* **konum_yoneticisi.py:** Aracın konum verilerini işleyen ve yönlendiren ana Python kodu.

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
