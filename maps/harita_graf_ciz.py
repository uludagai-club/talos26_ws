#!/usr/bin/env python3
"""
Harita yapisini cikaran basit gorsellestirme scripti.

Occupancy haritasi (my_map.pgm + my_map.yaml) ile yol grafini
(final_graph.yaml) ayni dunya koordinat sisteminde ust uste bindirip
tek bir PGM (P5, gri-tonlamali) olarak kaydeder. ROS ve matplotlib
gerekmez, standalone calisir; grafi occupancy grid'in kendi cozunurlugunde
piksellere dogrudan "yakar".

Cikti PGM oldugu icin renk yoktur:
    - kenarlar orta gri (--kenar-gri, varsayilan 128)
    - dugumler siyah   (--dugum-gri, varsayilan 0)
arka plan occupancy grid'in orijinal gri tonlaridir (0=dolu, 255=bos).

Kullanim (maps/ klasoru icinden ya da herhangi bir yerden):
    python3 harita_graf_ciz.py
    python3 harita_graf_ciz.py --cikti /tmp/harita.pgm --goster

Girdi dosyalari ayni klasorde varsayilir; --harita / --graf ile degistirilebilir.
"""
import argparse
import os

import yaml
from PIL import Image, ImageDraw

BURASI = os.path.dirname(os.path.abspath(__file__))


def harita_yukle(harita_yaml):
    """my_map.yaml + .pgm dosyasini okuyup (goruntu, res, ox, oy) doner.

    goruntu: 'L' modunda PIL Image (0=dolu/siyah .. 255=bos/beyaz).
    res: metre/piksel; (ox, oy): sol-alt piksele denk gelen dunya origin'i.
    """
    with open(harita_yaml) as f:
        meta = yaml.safe_load(f)

    res = float(meta["resolution"])                 # metre/piksel
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    # .pgm yolu yaml'a gore goreli olabilir
    pgm_yolu = meta["image"]
    if not os.path.isabs(pgm_yolu):
        pgm_yolu = os.path.join(os.path.dirname(harita_yaml), pgm_yolu)

    img = Image.open(pgm_yolu).convert("L")         # gri-tonlama garanti
    return img, res, ox, oy


def graf_yukle(graf_yaml):
    """final_graph.yaml -> (id->(x,y) sozlugu, [(u,v)...] kenar listesi)."""
    with open(graf_yaml) as f:
        g = yaml.safe_load(f)
    dugum = {n["id"]: (n["x"], n["y"]) for n in g["nodes"]}
    kenar = g.get("edges", [])
    return dugum, kenar


def dunya_to_piksel(x, y, res, ox, oy, h):
    """Dunya (metre) -> piksel (kolon j, satir i).

    ROS harita kurali: sol-alt piksel = origin (ox, oy) ve goruntu ust
    satirdan asagi saklanir, yani satir 0 (ust) en buyuk y'ye denk gelir.
    """
    j = (x - ox) / res
    i = h - (y - oy) / res
    return (j, i)


def ciz(harita_yaml, graf_yaml, cikti, goster, kenar_gri, dugum_gri,
        kenar_kalinlik, dugum_yaricap):
    img, res, ox, oy = harita_yukle(harita_yaml)
    dugum, kenar = graf_yukle(graf_yaml)
    w, h = img.size
    draw = ImageDraw.Draw(img)

    # 1) Kenarlar (orta gri cizgiler)
    for u, v in kenar:
        if u in dugum and v in dugum:
            x0, y0 = dugum[u]
            x1, y1 = dugum[v]
            p0 = dunya_to_piksel(x0, y0, res, ox, oy, h)
            p1 = dunya_to_piksel(x1, y1, res, ox, oy, h)
            draw.line([p0, p1], fill=kenar_gri, width=kenar_kalinlik)

    # 2) Dugumler (siyah noktalar)
    r = dugum_yaricap
    for (x, y) in dugum.values():
        j, i = dunya_to_piksel(x, y, res, ox, oy, h)
        draw.ellipse([j - r, i - r, j + r, i + r], fill=dugum_gri)

    img.save(cikti)   # 'L' modu + .pgm uzantisi -> P5 (binary PGM)
    print(f"Kaydedildi: {cikti}  ({w}x{h} px, {len(dugum)} dugum, "
          f"{len(kenar)} kenar)")
    if goster:
        img.show()


def main():
    ap = argparse.ArgumentParser(description="Harita + graf -> PGM gorsellestirme")
    ap.add_argument("--harita", default=os.path.join(BURASI, "my_map.yaml"),
                    help="occupancy harita yaml (my_map.yaml)")
    ap.add_argument("--graf", default=os.path.join(BURASI, "final_graph.yaml"),
                    help="yol grafi yaml (final_graph.yaml)")
    ap.add_argument("--cikti", default=os.path.join(BURASI, "harita_graf.pgm"),
                    help="cikti PGM yolu")
    ap.add_argument("--goster", action="store_true", help="pencerede goster")
    ap.add_argument("--kenar-gri", type=int, default=128,
                    help="kenar cizgilerinin gri tonu (0-255)")
    ap.add_argument("--dugum-gri", type=int, default=0,
                    help="dugum noktalarinin gri tonu (0-255)")
    ap.add_argument("--kenar-kalinlik", type=int, default=1,
                    help="kenar cizgi kalinligi (piksel)")
    ap.add_argument("--dugum-yaricap", type=int, default=2,
                    help="dugum nokta yaricapi (piksel)")
    args = ap.parse_args()

    ciz(args.harita, args.graf, args.cikti, args.goster,
        args.kenar_gri, args.dugum_gri, args.kenar_kalinlik, args.dugum_yaricap)


if __name__ == "__main__":
    main()
