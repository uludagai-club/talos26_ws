# TEKNOFEST Karar Algoritması

## 📁 Dosya Yapısı

```
decision_algorithm/
├── scripts/
│   ├── decision_node.py      # Ana karar algoritması
│   └── test_publisher.py     # Test simülatörü
└── config/
    └── decision_params.yaml  # Parametreler (opsiyonel)
```

## 🚀 Kullanım

### Karar Algoritmasını Çalıştır

```bash
# Mevcut ROS workspace'inizde
rosrun <paket_adı> decision_node.py

# Veya doğrudan
python3 decision_node.py
```

### Test Et

```bash
# Terminal 1: Karar algoritması
python3 decision_node.py

# Terminal 2: Test simülatörü
python3 test_publisher.py
# Menüden senaryo seç (1-7)
```

## 📊 Karar Algoritması Mantığı

### Öncelik Sırası
1. **ACİL DURUM** (< 2m) → `emergency_stop`
2. **YAYA GEÇİDİ** → Mesafeye göre
3. **TRAFİK LEVHALARI** → Levha tipine göre
4. **NORMAL SÜRÜŞ** → `forward,normal`

### Yaya Geçidi
```
> 10m    → Normal sürüş
10-3m    → Yavaşla (forward,slow)
< 3m     → Dur (stop) + 5s bekle
Sonra    → Yavaşça devam
```

### STOP Levhası
```
> 8m     → Normal sürüş
8-2.5m   → Yavaşla (forward,slow)
< 2.5m   → Dur (stop) + 3s bekle
Sonra    → Normal hızda devam
```

### Diğer Levhalar
- **GO:** İleri git
- **TURN_LEFT:** 2s sola dön
- **TURN_RIGHT:** 2s sağa dön

## 📨 Mesaj Formatları

### Görüntü İşleme → Karar Algoritması

**Trafik Levhası** (`/trafik_levha`):
```
"sign_type,distance,confidence"
Örnek: "stop,8.5,0.95"
```

**Yaya Geçidi** (`/yaya_gecidi`):
```
"detected,distance,confidence"
Örnek: "true,5.2,0.92"
```

### Karar Algoritması → CAN Bus

**Komutlar** (`/control_canbus`):
```
stop              # Dur
forward,normal    # Normal hızda ileri
forward,slow      # Yavaş hızda ileri
turn_left         # Sola dön
turn_right        # Sağa dön
emergency_stop    # Acil fren
```

## ⚙️ Parametreler

Kodun içinde tanımlı (isteğe bağlı `decision_params.yaml` kullanılabilir):

```python
# Mesafeler (metre)
crosswalk_slow_down: 10.0    # Yaya geçidi yavaşlama
crosswalk_stop: 3.0          # Yaya geçidi durma
traffic_sign_slow_down: 8.0  # STOP yavaşlama
traffic_sign_stop: 2.5       # STOP durma

# Süreler (saniye)
crosswalk_wait_time: 5.0     # Yaya geçidi bekleme
stop_sign_wait_time: 3.0     # STOP bekleme
turn_duration: 2.0           # Dönüş süresi

# Güven skorları
min_sign_confidence: 0.7     # Min levha güven
min_crosswalk_confidence: 0.6 # Min yaya geçidi güven
```

## 🧪 Test Senaryoları

1. Normal sürüş
2. Yaya geçidi (15m → 3m → dur → bekle)
3. STOP levhası (10m → 2.5m → dur → bekle)
4. Sola dönüş
5. Sağa dönüş
6. Acil durum (2m'de yaya geçidi)
7. Rastgele test

## 🔍 Debug

# Topic'leri izle
rostopic echo /trafik_levha
rostopic echo /yaya_gecidi
rostopic echo /control_canbus

# Topic listesi
rostopic list