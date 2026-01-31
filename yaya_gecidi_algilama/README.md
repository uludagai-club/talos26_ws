# 🚸 Yaya Geçidi Algılama Modülü

Bu paket, **Turtlebot3 Waffle Pi** robotunun kamera verilerini kullanarak yaya geçitlerini tespit etmesini, çizgide durmasını ve belirli bir süre bekledikten sonra yoluna devam etmesini sağlar.

## ⚠️ Önemli Ön Hazırlık (Kamera İçin)
Robotun kamerasının düzgün çalışması için modelin **Waffle Pi** olarak seçilmesi şarttır. Kodu çalıştırmadan önce mutlaka şu komutu girin:

```bash
export TURTLEBOT3_MODEL=waffle_pi
Kodu çalıştırmak için şu komutu girin:
python3 yaya_state_machine.py
