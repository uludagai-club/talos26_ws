#ID: 0x100 | Data: 40 00 00 00 00 00 00 00 | Timestamp: 1766259805.564
#ID: 0x201 | Data: E8 03 00 00 00 00 00 00 | Timestamp: 1766259831.410

import can
import struct

class CANDecoder:
    """CAN mesajlarını anlamlı değerlere çeviren sınıf"""
    
    @staticmethod
    def decode_speed(data):
        """
        Hız verisini decode et
        Format: 2 byte, Little Endian, ölçek: 0.01, birim: km/h
        """
        if len(data) < 2:
            return 0.0
        
        # İlk 2 byte'ı Little Endian olarak oku
        raw_value = struct.unpack('<H', data[0:2])[0]
        
        # Ölçekle ve km/h'ye çevir
        speed_kmh = raw_value * 0.01
        return speed_kmh
    
    @staticmethod
    def decode_steering(data):
        """
        Direksiyon açısını decode et
        Format: 2 byte, Little Endian, ölçek: 0.1, birim: derece
        Offset: -500 (merkez sıfır için)
        """
        if len(data) < 2:
            return 0.0
        
        raw_value = struct.unpack('<H', data[0:2])[0]
        
        # Ölçekle ve offseti uygula
        angle_deg = (raw_value * 0.1) - 500.0
        return angle_deg

    @staticmethod
    def decode_gear(data):
        """
        Vites bilgisini decode et
        Format: Byte 2 (Index 2)
        0: No Command, 1: Neutral, 2: Forward, 3: Reverse
        """
        if len(data) < 3:
            return 0 # Varsayılan
        
        return data[2]

    @staticmethod
    def decode_real_speed(data):
        """
        GERÇEK Hız verisini decode et (ID: 0x301)
        Format: Byte 0-1, Little Endian, ölçek: 0.01, birim: km/h
        """
        if len(data) < 2:
            return 0.0
        return struct.unpack('<H', data[0:2])[0] * 0.01

    @staticmethod
    def decode_rpm(data):
        """
        Motor Devrini decode et (ID: 0x301)
        Format: Byte 2-3, Little Endian, birim: RPM
        """
        if len(data) < 4:
            return 0
        return struct.unpack('<H', data[2:4])[0]

    @staticmethod
    def decode_brake(data):
        """
        Fren değerini decode et (ID: 0x100)
        Format: Byte 3 (Index 3), Ölçek: 0.01 (yani 0-100 değer 0-1.0 olsun)
        """
        if len(data) < 4:
            return 0.0
        return data[3] * 0.01

    # =========================================================================
    # YENİ MESAJLAR - Genişletilmiş CAN Protokolü
    # =========================================================================

    @staticmethod
    def decode_battery_status(data):
        """
        Batarya Durumu decode et (ID: 0x303)
        Format:
            Byte 0-1: SoC (State of Charge) %, ölçek: 0.1
            Byte 2-3: Voltaj (V), ölçek: 0.1
            Byte 4-5: Akım (A), ölçek: 0.1, signed
            Byte 6: Sıcaklık (°C), offset: -40
        """
        if len(data) < 7:
            return {'soc': 0.0, 'voltage': 0.0, 'current': 0.0, 'temperature': 0}

        soc = struct.unpack('<H', data[0:2])[0] * 0.1
        voltage = struct.unpack('<H', data[2:4])[0] * 0.1
        current = struct.unpack('<h', data[4:6])[0] * 0.1  # Signed (şarj/deşarj)
        temperature = data[6] - 40

        return {
            'soc': min(100.0, soc),
            'voltage': voltage,
            'current': current,
            'temperature': temperature
        }

    @staticmethod
    def decode_error_codes(data):
        """
        Hata/Uyarı Kodları decode et (ID: 0x304)
        Format:
            Byte 0: Aktif hata sayısı
            Byte 1: Hata seviyesi (0=Yok, 1=Uyarı, 2=Hata, 3=Kritik)
            Byte 2-3: Ana hata kodu
            Byte 4-5: Alt hata kodu
            Byte 6: Sistem durumu (bitmask)
        """
        if len(data) < 7:
            return {'error_count': 0, 'level': 0, 'main_code': 0, 'sub_code': 0, 'status': 0}

        return {
            'error_count': data[0],
            'level': data[1],
            'main_code': struct.unpack('<H', data[2:4])[0],
            'sub_code': struct.unpack('<H', data[4:6])[0],
            'status': data[6]
        }

    @staticmethod
    def decode_park_brake(data):
        """
        Park Freni Durumu decode et (ID: 0x305)
        Format:
            Byte 0: Park freni durumu (0=Serbest, 1=Aktif, 2=Geçiş)
            Byte 1: Talep edilen durum
        """
        if len(data) < 2:
            return {'state': 0, 'requested': 0}

        return {
            'state': data[0],
            'requested': data[1]
        }

    @staticmethod
    def decode_speed_limit(data):
        """
        Hız Limiti decode et (ID: 0x103)
        Format:
            Byte 0-1: Maksimum hız limiti (km/h * 100)
            Byte 2: Limit aktif mi (0=Hayır, 1=Evet)
            Byte 3: Limit kaynağı (0=Manuel, 1=Otonom, 2=Güvenlik)
        """
        if len(data) < 4:
            return {'max_speed_kmh': 0.0, 'active': False, 'source': 0}

        max_speed = struct.unpack('<H', data[0:2])[0] * 0.01
        active = data[2] == 1
        source = data[3]

        return {
            'max_speed_kmh': max_speed,
            'active': active,
            'source': source
        }

    @staticmethod
    def decode_emergency_stop(data):
        """
        Acil Duruş decode et (ID: 0x001)
        Format:
            Byte 0: Acil duruş aktif (0=Normal, 1=Acil Duruş)
            Byte 1: Kaynak (0=Yazılım, 1=Düğme, 2=Uzaktan, 3=Engel)
        """
        if len(data) < 2:
            return {'active': False, 'source': 0}

        return {
            'active': data[0] == 1,
            'source': data[1]
        }

    # =========================================================================
    # ENCODE FONKSİYONLARI - CAN Mesajı Oluşturma
    # =========================================================================

    @staticmethod
    def encode_battery_status(soc, voltage, current, temperature):
        """
        Batarya Durumu encode et (ID: 0x303)
        """
        soc_raw = int(min(100.0, soc) * 10)
        voltage_raw = int(voltage * 10)
        current_raw = int(current * 10)
        temp_raw = int(temperature + 40)

        data = struct.pack('<HHhB', soc_raw, voltage_raw, current_raw, temp_raw)
        return data + bytes(8 - len(data))

    @staticmethod
    def encode_error_codes(error_count, level, main_code, sub_code, status):
        """
        Hata Kodları encode et (ID: 0x304)
        """
        data = bytes([error_count, level])
        data += struct.pack('<HH', main_code, sub_code)
        data += bytes([status, 0])
        return data

    @staticmethod
    def encode_park_brake_command(requested_state):
        """
        Park Freni Komutu encode et (ID: 0x102)
        requested_state: 0=Serbest, 1=Aktif
        """
        return bytes([requested_state]) + bytes(7)

    @staticmethod
    def encode_speed_limit(max_speed_kmh, active, source=0):
        """
        Hız Limiti Komutu encode et (ID: 0x103)
        """
        speed_raw = int(max_speed_kmh * 100)
        data = struct.pack('<H', speed_raw)
        data += bytes([1 if active else 0, source])
        return data + bytes(4)

    @staticmethod
    def encode_emergency_stop(active, source=0):
        """
        Acil Duruş Komutu encode et (ID: 0x001)
        """
        return bytes([1 if active else 0, source]) + bytes(6)


# =========================================================================
# CAN MESAJ ID TANIMLARI
# =========================================================================

class CANMessageID:
    """CAN Mesaj ID Sabitleri"""

    # Komut Mesajları (Araca Gönderilen)
    EMERGENCY_STOP = 0x001      # Acil duruş komutu
    THROTTLE_BRAKE_GEAR = 0x100 # Gaz/Fren/Vites komutu
    PARK_BRAKE_CMD = 0x102      # Park freni komutu
    SPEED_LIMIT_CMD = 0x103     # Hız limiti komutu
    STEERING = 0x201            # Direksiyon komutu

    # Geri Besleme Mesajları (Araçtan Okunan)
    SPEED_RPM = 0x301           # Hız ve RPM
    IMU_ACCEL = 0x302           # İvme verileri
    BATTERY_STATUS = 0x303      # Batarya durumu
    ERROR_CODES = 0x304         # Hata kodları
    PARK_BRAKE_STATUS = 0x305   # Park freni durumu


# Test et
if __name__ == "__main__":
    print("=" * 60)
    print("  CAN Decoder Test")
    print("=" * 60)

    # Örnek veri: 0x4000 = 16384 decimal
    test_speed = bytes([0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    print(f"Hız: {CANDecoder.decode_speed(test_speed):.2f} km/h")

    # Örnek veri: 0x03E8 = 1000 decimal -> (1000*0.1)-500 = -400 derece
    test_steering = bytes([0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    print(f"Direksiyon: {CANDecoder.decode_steering(test_steering):.1f}°")

    print("\n--- Yeni Mesaj Tipleri ---")

    # Batarya testi
    battery_data = CANDecoder.encode_battery_status(soc=75.5, voltage=48.2, current=-12.5, temperature=35)
    battery = CANDecoder.decode_battery_status(battery_data)
    print(f"Batarya: SoC={battery['soc']:.1f}%, V={battery['voltage']:.1f}V, I={battery['current']:.1f}A, T={battery['temperature']}°C")

    # Hız limiti testi
    limit_data = CANDecoder.encode_speed_limit(max_speed_kmh=5.0, active=True, source=1)
    limit = CANDecoder.decode_speed_limit(limit_data)
    print(f"Hız Limiti: {limit['max_speed_kmh']:.1f} km/h, Aktif={limit['active']}")

    # Acil duruş testi
    estop_data = CANDecoder.encode_emergency_stop(active=True, source=3)
    estop = CANDecoder.decode_emergency_stop(estop_data)
    print(f"Acil Duruş: Aktif={estop['active']}, Kaynak={estop['source']}")

    print("\n--- CAN Mesaj ID'leri ---")
    print(f"EMERGENCY_STOP: 0x{CANMessageID.EMERGENCY_STOP:03X}")
    print(f"THROTTLE_BRAKE_GEAR: 0x{CANMessageID.THROTTLE_BRAKE_GEAR:03X}")
    print(f"STEERING: 0x{CANMessageID.STEERING:03X}")
    print(f"BATTERY_STATUS: 0x{CANMessageID.BATTERY_STATUS:03X}")