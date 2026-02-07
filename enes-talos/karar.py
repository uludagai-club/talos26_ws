#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import String

# 1. Mesafe Eşikleri (Metre)
MESAFE_ACIL_DURUS = 2.0     # Bu mesafenin altında ne olursa olsun DUR.
MESAFE_YAYA_DUR   = 4.0     # Yayayı görünce güvenli durma mesafesi.
MESAFE_YAYA_YAVAS = 12.0    # Yayayı görünce gaz kesme mesafesi.
MESAFE_LEVHA_DUR  = 3.5     # DUR levhasında durulacak mesafe.
MESAFE_LEVHA_OKU  = 10.0    # Levhaların dikkate alınacağı maksimum menzil.

# 2. Zamanlayıcılar (Saniye)
SURE_DUR_LEVHASI_BEKLEME = 3.0  # Kurallar gereği tam duruş süresi.

class KararMekanizmasi:
    def __init__(self):
        # Node Başlatma
        rospy.init_node('karar_mekanizmasi_enes', anonymous=True)
        rospy.loginfo(">> KARAR MEKANİZMASI BAŞLATILDI (Logic Unit Active)")
        rospy.loginfo(">> Beklenen Veri Formatı: 'x,y' (Metre cinsinden bağıl konum)")

        # --- ABONELİKLER (INPUTS) ---
        # Görüntü işleme ekiplerinden gelen veriler
        rospy.Subscriber("/trafik_levha", String, self.levha_callback)
        rospy.Subscriber("/yaya_gecidi", String, self.yaya_callback)

        # --- YAYINCI (OUTPUT) ---
        # Hilmi'nin kontrolcüsüne giden nihai emir
        self.karar_pub = rospy.Publisher('/karar', String, queue_size=10)

        # --- DURUM DEĞİŞKENLERİ (STATE MEMORY) ---
        self.yaya_verisi = "none"
        self.levha_verisi = "none"
        
        # Dur Levhası Mantığı İçin Hafıza
        self.dur_levhasi_aktif = False
        self.durma_baslangic_zamani = None

        # Sistem Döngü Hızı (10 Hz = 100ms tepki süresi)
        self.rate = rospy.Rate(10)

    def levha_callback(self, msg):
        """Selenay/Aslı'dan gelen levha verisini günceller."""
        self.levha_verisi = msg.data

    def yaya_callback(self, msg):
        """Aybüke/Rabia'dan gelen yaya verisini günceller."""
        self.yaya_verisi = msg.data

    def mesafe_hesapla(self, x_str, y_str):
        """
        Gelen x,y string değerlerini float'a çevirip hipotenüsü (uzaklığı) hesaplar.
        Hata durumunda -1 döndürür.
        """
        try:
            x = float(x_str)
            y = float(y_str)
            # Öklid Mesafesi (Euclidean Distance): sqrt(x^2 + y^2)
            return math.hypot(x, y)
        except ValueError:
            return -1.0

    def mantik_yurut(self):
        """
        Ana beyin döngüsü. Öncelik sırasına göre kararı belirler.
        Priority: ACİL > YAYA > LEVHA > NORMAL
        """
        while not rospy.is_shutdown():
            nihai_karar = "normal" # Varsayılan: Her şey yolunda, devam et.
            
            # Debug için anlık durumu konsola basma (Opsiyonel kapatılabilir)
            # rospy.loginfo(f"Inputlar -> Yaya: {self.yaya_verisi} | Levha: {self.levha_verisi}")

            # =========================================================
            # 1. ANALİZ KATMANI: Verileri İşle ve Mesafeleri Bul
            # =========================================================
            
            yaya_mesafesi = -1
            levha_mesafesi = -1
            levha_ismi = "none"

            # Yaya Verisini İşle
            if self.yaya_verisi != "none":
                try:
                    # Format: "x,y" -> Örn: "10.5,2.0"
                    parcalar = self.yaya_verisi.split(',')
                    yaya_mesafesi = self.mesafe_hesapla(parcalar[0], parcalar[1])
                except:
                    rospy.logwarn("Veri Hatasi: Yaya verisi parse edilemedi!")

            # Levha Verisini İşle
            if self.levha_verisi != "none":
                try:
                    # Format: "ISIM,x,y" -> Örn: "DUR,15.0,3.0"
                    parcalar = self.levha_verisi.split(',')
                    levha_ismi = parcalar[0]
                    levha_mesafesi = self.mesafe_hesapla(parcalar[1], parcalar[2])
                except:
                    rospy.logwarn("Veri Hatasi: Levha verisi parse edilemedi!")

            # =========================================================
            # 2. KARAR KATMANI: Hiyerarşik Kontrol (Decision Tree)
            # =========================================================

            # --- A. ACİL DURUM KONTROLÜ (Safety First) ---
            if (yaya_mesafesi != -1 and yaya_mesafesi < MESAFE_ACIL_DURUS):
                nihai_karar = "acildurus"
                rospy.logerr(f"!!! ACİL DURUM !!! Yaya Çok Yakın: {yaya_mesafesi:.2f}m")

            # --- B. YAYA GÜVENLİK PROTOKOLÜ ---
            elif yaya_mesafesi != -1:
                if yaya_mesafesi < MESAFE_YAYA_DUR:
                    nihai_karar = "dur"
                    rospy.logwarn(f"Yaya Geçidi: Tam Durus. Mesafe: {yaya_mesafesi:.2f}m")
                elif yaya_mesafesi < MESAFE_YAYA_YAVAS:
                    nihai_karar = "slow"
                    rospy.loginfo(f"Yaya Geçidi: Yavaşlaniyor. Mesafe: {yaya_mesafesi:.2f}m")

            # --- C. TRAFİK LEVHA PROTOKOLÜ ---
            elif levha_mesafesi != -1 and levha_mesafesi < MESAFE_LEVHA_OKU:
                
                # C1. DUR LEVHASI MANTIĞI (Timerlı Yapı)
                if levha_ismi == "DUR":
                    if levha_mesafesi < MESAFE_LEVHA_DUR or self.dur_levhasi_aktif:
                        # Sayacı Başlat
                        if not self.dur_levhasi_aktif:
                            self.dur_levhasi_aktif = True
                            self.durma_baslangic_zamani = rospy.get_time()
                            rospy.loginfo("🛑 DUR LEVHASI: 3 Saniyelik bekleme başlatıldı.")

                        # Süreyi Kontrol Et
                        gecen_sure = rospy.get_time() - self.durma_baslangic_zamani
                        if gecen_sure < SURE_DUR_LEVHASI_BEKLEME:
                            nihai_karar = "dur"
                            rospy.loginfo(f"🛑 Bekleniyor... ({gecen_sure:.1f}/{SURE_DUR_LEVHASI_BEKLEME}s)")
                        else:
                            # Süre doldu, artık geçebiliriz
                            nihai_karar = "normal"
                            # Levha görüşten çıkana kadar tekrar tetiklenmemesi için 
                            # self.dur_levhasi_aktif'i hemen False yapmıyoruz, 
                            # levha "none" olunca veya mesafe artınca sıfırlanabilir.
                            # Şimdilik basitlik adına burada bırakıyoruz, araç ilerleyince levha arkada kalacak.
                            if levha_mesafesi > MESAFE_LEVHA_DUR + 2.0: # Biraz uzaklaşınca resetle
                                self.dur_levhasi_aktif = False

                    elif levha_mesafesi < MESAFE_LEVHA_OKU:
                        nihai_karar = "slow" # Levhaya yaklaşırken yavaşla

                # C2. HIZ SINIRI LEVHASI
                elif levha_ismi == "30" or levha_ismi == "OKUL":
                     nihai_karar = "slow"
                
                # C3. YÖN LEVHALARI
                elif levha_ismi == "SAG" and levha_mesafesi < 5.0:
                    nihai_karar = "sag"
                elif levha_ismi == "SOL" and levha_mesafesi < 5.0:
                    nihai_karar = "sol"

            # --- D. VARSAYILAN SÜRÜŞ ---
            else:
                # Tehlike yok, veri yok veya objeler uzakta.
                nihai_karar = "normal"
                # Dur levhası sayacını, levha görüşten çıktıysa resetle (Güvenlik)
                if self.levha_verisi == "none":
                    self.dur_levhasi_aktif = False

            # =========================================================
            # 3. AKSİYON KATMANI: Emri Yayınla
            # =========================================================
            self.karar_pub.publish(nihai_karar)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        beyin = KararMekanizmasi()
        beyin.mantik_yurut()
    except rospy.ROSInterruptException:
        pass