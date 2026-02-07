# Mini Proje Karar Algoritması

**Modül:** `karar.py`  
**Sorumlu:** Enes Bilgin  
---

## 1. Çalışma Mantığı
Bu modül, görüntü işleme ekibinden gelen nesne verilerini alır, **araca göre bağıl (relative)** mesafeyi hesaplar ve motor kontrolcüsüne (Hilmi) nihai sürüş emrini iletir.

* **Matematik:** Gelen `x,y` verileri kullanılarak Öklid mesafesi hesaplanır: $\sqrt{x^2 + y^2}$
---

## 2. Haberleşme Protokolü
Sistemin çalışması için tüm birimlerin Burak'ın attığı roadmaps'e göre aşağıdaki veri formatlarına uyması gerekmektedir.

### 📥 Girdiler (Subscribers)
Görüntü işleme ekipleri veriyi **String** formatında basmalıdır.

| Topic | Kaynak | Format | Örnek Veri | Açıklama |
| :--- | :--- | :--- | :--- | :--- |
| `/yaya_gecidi` | Aybüke | `"x,y"` | `"12.5,2.0"` | Araçtan 12.5m ileride, 2m sağda. |
| `/trafik_levha` | Selenay | `"ISIM,x,y"` | `"DUR,3.5,0.0"` | Araçtan 3.5m ileride DUR levhası. |

### 📤 Çıktılar (Publishers)
Motor kontrolcüsü bu topic'i dinleyerek hareket etmelidir.

| Topic | Hedef | Format | Komutlar |
| :--- | :--- | :--- | :--- |
| `/karar` | Hilmi | `String` | `"dur"`, `"slow"`, `"normal"`, `"acildurus"`, `"sag"`, `"sol"` |

---

## 3. Karar Kuralları (Öncelik Sırası)

Modül, aşağıdaki hiyerarşiye göre karar verir. Üstteki şart sağlanırsa alttakiler ezilir.

1.  **ACİL DURUM:** Herhangi bir cisim **< 2.0m** ise → `acildurus`
2.  **YAYA:**
    * **< 4.0m** ise → `dur`
    * **< 12.0m** ise → `slow`
3.  **LEVHA:**
    * **DUR:** < 3.5m ise tam duruş (3 saniye bekleme).
    * **30/OKUL:** < 10.0m ise `slow`.
    * **SAG/SOL:** < 5.0m ise `sag` veya `sol`.
4.  **NORMAL:** Tehdit yoksa → `normal`

---

## 4. Parametre Ayarları
Mesafe eşikleri kodun içinde sabitlenmiştir, sahada gerekirse `karar.py` başındaki şu satırlar düzenlenebilir:

```python
MESAFE_ACIL_DURUS = 2.0     # Metre
MESAFE_YAYA_DUR   = 4.0     # Metre
MESAFE_YAYA_YAVAS = 12.0    # Metre
MESAFE_LEVHA_DUR  = 3.5     # Metre
```

## 5. Nasıl Çalıştırılır?

```bash
# 1. ROS Core'u Başlat
roscore

# 2. Karar Modülünü Başlat (Ayrı Terminal)
rosrun <paketadı> karar.py
veya
python3 karar.py
```

## 6. Manuel Test

```bash
# Senaryo 1: Yaya çok uzakta (Normal gitmeli)
rostopic pub /yaya_gecidi std_msgs/String "20.0,5.0"

# Senaryo 2: Önüne yaya atladı (Acil duruş yapmalı)
rostopic pub /yaya_gecidi std_msgs/String "1.5,0.0"

# Senaryo 3: DUR levhası gördü (3sn durup devam etmeli)
rostopic pub /trafik_levha std_msgs/String "DUR,3.0,1.0"
```

## 7. Docker Notları!!!
Bu modül Docker konteyneri olarak paketlenmiştir. ROS ağındaki diğer düğümlerle (Hilmi, Aybüke vb.) haberleşebilmesi için **Host Network** modunda çalıştırılması zorunludur.

Adım 1: İmajı Yükle

--> docker load -i karar_node.tar

Adım 2: Başlat (Kritik Adım) --net=host parametresi olmadan topic verileri akmaz.

--> docker run -it --net=host karar-node