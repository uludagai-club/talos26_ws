#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
import time

class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')

        # Koordinatları doğrudan METRE (X, Y) cinsinden yaziyoruz.
        # Simülasyonun başlangıç noktasını (0,0) kabul ediyoruz.
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

        # --- DEGISKENLER ---
        self.hedef_kuyrugu = [] 
        self.aktif_hedef_index = 0

        # --- ILETISIM ---
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10, latch=True)
        rospy.Subscriber('/gorev_durumu', String, self.durum_callback)

        print("Hedef Sistemi (Manuel X/Y Modu) Baslatildi.")
        
        self.haritayi_isle()
        
        time.sleep(2)
        print("Sistem hazir. İlk hedef gönderiliyor...")
        self.hedef_gonder()

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

    def hedef_gonder(self):
        """ Sıradaki hedefi gönderir """
        if self.aktif_hedef_index < len(self.hedef_kuyrugu):
            hedef = self.hedef_kuyrugu[self.aktif_hedef_index]
            
            # Melih'e giden mesaj
            msg_dict = {
                "type": "Point",
                "target_name": hedef["isim"],
                "coordinates": [hedef["x"], hedef["y"]]
            }
            
            self.pub_hedef.publish(json.dumps(msg_dict))
            print(f"[GÖREV] Gönderildi: {hedef['isim']} -> X:{hedef['x']}, Y:{hedef['y']}")
        else:
            print("\n[BİTİŞ] TÜM GÖREVLER TAMAMLANDI!")

    def durum_callback(self, msg):
        """ Melih 'VARILDI' deyince sonrakine geçer """
        if msg.data == "VARILDI":
            biten = self.hedef_kuyrugu[self.aktif_hedef_index]['isim']
            print(f"[ONAY] {biten} tamamlandı. Sonrakine geçiliyor...")
            
            self.aktif_hedef_index += 1
            time.sleep(1) # Kisa bekleme
            self.hedef_gonder()

if __name__ == '__main__':
    try:
        HedefYoneticisi()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass