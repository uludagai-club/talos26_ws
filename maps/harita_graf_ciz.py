#!/usr/bin/env python3
"""
Harita yapisini cikaran basit gorsellestirme scripti.

Occupancy haritasi (my_map.pgm + my_map.yaml) ile yol grafini
(final_graph.yaml) ayni dunya koordinat sisteminde ust uste bindirip
tek bir PNG olarak kaydeder. ROS gerekmez, standalone calisir.

Kullanim (maps/ klasoru icinden ya da herhangi bir yerden):
    python3 harita_graf_ciz.py
    python3 harita_graf_ciz.py --cikti /tmp/harita.png --goster

Girdi dosyalari ayni klasorde varsayilir; --harita / --graf ile degistirilebilir.
"""
import argparse
import os

import yaml
import numpy as np
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt

BURASI = os.path.dirname(os.path.abspath(__file__))


def harita_yukle(harita_yaml):
    """my_map.yaml + .pgm dosyasini okuyup (goruntu, extent) doner.

    extent = [x_min, x_max, y_min, y_max] dunya (metre) koordinatinda;
    matplotlib imshow icin origin='upper' ile kullanilir.
    """
    with open(harita_yaml) as f:
        meta = yaml.safe_load(f)

    res = float(meta["resolution"])                 # metre/piksel
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    # .pgm yolu yaml'a gore goreli olabilir
    pgm_yolu = meta["image"]
    if not os.path.isabs(pgm_yolu):
        pgm_yolu = os.path.join(os.path.dirname(harita_yaml), pgm_yolu)

    img = np.asarray(Image.open(pgm_yolu))          # 0=dolu(siyah) .. 255=bos(beyaz)
    h, w = img.shape[:2]

    # ROS harita kuralı: sol-alt piksel = origin. origin='upper' ile
    # goruntu satir 0 (ust) y_max'a denk gelir.
    extent = [ox, ox + w * res, oy, oy + h * res]
    return img, extent


def graf_yukle(graf_yaml):
    """final_graph.yaml -> (id->(x,y) sozlugu, [(u,v)...] kenar listesi)."""
    with open(graf_yaml) as f:
        g = yaml.safe_load(f)
    dugum = {n["id"]: (n["x"], n["y"]) for n in g["nodes"]}
    kenar = g.get("edges", [])
    return dugum, kenar


def ciz(harita_yaml, graf_yaml, cikti, goster):
    img, extent = harita_yukle(harita_yaml)
    dugum, kenar = graf_yukle(graf_yaml)

    fig, ax = plt.subplots(figsize=(12, 12))

    # 1) Arka plan: occupancy grid (gri)
    ax.imshow(img, cmap="gray", origin="upper", extent=extent, zorder=0)

    # 2) Kenarlar (mavi cizgiler)
    for u, v in kenar:
        if u in dugum and v in dugum:
            x0, y0 = dugum[u]
            x1, y1 = dugum[v]
            ax.plot([x0, x1], [y0, y1], "-", color="#1f77b4",
                    linewidth=0.8, alpha=0.7, zorder=1)

    # 3) Dugumler (kirmizi noktalar)
    xs = [p[0] for p in dugum.values()]
    ys = [p[1] for p in dugum.values()]
    ax.scatter(xs, ys, s=6, c="red", zorder=2)

    ax.set_title(f"Harita + yol grafi  |  {len(dugum)} dugum, {len(kenar)} kenar")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.3)

    fig.tight_layout()
    fig.savefig(cikti, dpi=150)
    print(f"Kaydedildi: {cikti}  ({len(dugum)} dugum, {len(kenar)} kenar)")
    if goster:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Harita + graf gorsellestirme")
    ap.add_argument("--harita", default=os.path.join(BURASI, "my_map.yaml"),
                    help="occupancy harita yaml (my_map.yaml)")
    ap.add_argument("--graf", default=os.path.join(BURASI, "final_graph.yaml"),
                    help="yol grafi yaml (final_graph.yaml)")
    ap.add_argument("--cikti", default=os.path.join(BURASI, "harita_graf.png"),
                    help="cikti PNG yolu")
    ap.add_argument("--goster", action="store_true", help="pencerede goster")
    args = ap.parse_args()

    if not args.goster:
        matplotlib.use("Agg")   # basssiz ortamda (Docker/SSH) calissin
    ciz(args.harita, args.graf, args.cikti, args.goster)


if __name__ == "__main__":
    main()
