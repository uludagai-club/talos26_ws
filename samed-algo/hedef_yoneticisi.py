#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json

class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')

        # --- HARİTA VERİSİ ---
        self.geojson_data = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": "gorev_1", "description": "1. Durak"},
                    "geometry": {"type": "Point", "coordinates": [-5.0, -34.0]}
                },
                {
                    "type": "Feature",
                    "properties": {"name": "gorev_2", "description": "2. Durak"},
                    "geometry": {"type": "Point", "coordinates": [11.0, -25.0]}
                },
                {
                    "type": "Feature",
                    "properties": {"name": "gorev_3", "description": "3. Durak"},
                    "geometry": {"type": "Point", "coordinates": [20.0, -22.0]}
                },
                {
                    "type": "Feature",
                    "properties": {"name": "gorev_4", "description": "4. Durak"},
                    "geometry": {"type": "Point", "coordinates": [25.0, -6.0]}
                }
            ]
        }

        # --- DEĞİŞKENLER ---
        self.hedef_kuyrugu = [] 
        self.aktif_hedef_index = 0
        self.gorev_tamamlandi = False # Sürekli 'Bitti' yazdırmamak için bayrak

        # --- İLETİŞİM ---
        # latch=True: Yeni abone olanlar son mesajı hemen alır
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10, latch=True)
        rospy.Subscriber('/gorev_durumu', String, self.durum_callback)

        print("Hedef Sistemi Başlatıldı. (Periyodik Yayın Modu)")
        self.haritayi_isle()
        
        # --- ZAMANLAYICI (TIMER) ---
        # 1.0 saniyede bir self.timer_callback fonksiyonunu çağırır.
        print("Sistem hazır. Hedefler her saniye yayınlanacak.")
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def haritayi_isle(self):
        """ JSON verisini okur ve listeye atar """
        features = self.geojson_data["features"]
        
        print("-" * 30)
        print("ROTANIZ (X, Y Metre):")
        for f in features:
            isim = f["properties"]["name"]
            x = f["geometry"]["coordinates"][0]
            y = f["geometry"]["coordinates"][1]
            self.hedef_kuyrugu.append({"isim": isim, "x": x, "y": y})
            print(f" > {isim:<10}: X={x:.2f}m, Y={y:.2f}m")
        print("-" * 30)

    def timer_callback(self, event):
        """ Her saniye otomatik çağrılır """
        # Görevler bitmediyse yayınla
        if self.aktif_hedef_index < len(self.hedef_kuyrugu):
            self.hedef_gonder()
        else:
            # Görev bittiyse ve daha önce yazdırmadıysak
            if not self.gorev_tamamlandi:
                print("\n[BİTİŞ] TÜM GÖREVLER TAMAMLANDI! (Yayın durduruldu)")
                self.gorev_tamamlandi = True
            # Burada 'pass' geçiyoruz, artık yayın yapmıyoruz.

    def hedef_gonder(self):
        """ Aktif hedefi yayınlar """
        hedef = self.hedef_kuyrugu[self.aktif_hedef_index]
        
        msg_dict = {
            "type": "Point",
            "target_name": hedef["isim"],
            "coordinates": [hedef["x"], hedef["y"]]
        }
        
        json_str = json.dumps(msg_dict)
        self.pub_hedef.publish(json_str)
        # Sürekli aynı şeyi print edip konsolu doldurmasın diye sadece log (debug) amaçlı durabilir
        # Veya sadece publish edebiliriz. Aşağıdaki satır her saniye konsola yazar:
        print(f"[TIMER] Yayınlanıyor: {hedef['isim']} -> X:{hedef['x']}, Y:{hedef['y']}")

    def durum_callback(self, msg):
        """ Melih 'VARILDI' deyince indexi artırır """
        if msg.data == "VARILDI":
            # Halen gidilecek hedef var mı kontrol et
            if self.aktif_hedef_index < len(self.hedef_kuyrugu):
                biten = self.hedef_kuyrugu[self.aktif_hedef_index]['isim']
                print(f"\n[ONAY] {biten} hedefine varıldı! Sıradaki hedefe geçiliyor...\n")
                
                # Sadece indexi artırıyoruz.
                # Timer zaten her saniye çalıştığı için yeni hedefi otomatik gönderecek.
                self.aktif_hedef_index += 1

if __name__ == '__main__':
    try:
        HedefYoneticisi()
        rospy.spin() # ROS döngüsünü canlı tutar
    except rospy.ROSInterruptException:
        pass