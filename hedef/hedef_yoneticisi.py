#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hedef_yoneticisi.py — Entry point (modüler hedef paketi için).

Eski monolitik hedef_yoneticisi.py yerine modüler dosyalardan
HedefYoneticisi'ni import edip çalıştırır.
"""

import sys
import os

# Modüllerin bulunabilmesi için bu dizini PYTHONPATH'e ekle
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

import rospy
from manager import HedefYoneticisi

if __name__ == '__main__':
    try:
        HedefYoneticisi().loop()
    except rospy.ROSInterruptException:
        pass