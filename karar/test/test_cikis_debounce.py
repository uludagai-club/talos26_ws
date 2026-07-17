#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cikis_debounce.CikisDebounce regresyon testi (P1 №6, E3-O2).

Çalıştır:
    cd talos26_ws/karar && python3 -m test.test_cikis_debounce
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from cikis_debounce import CikisDebounce


def _d(karar, reason="r"):
    return {"karar": karar, "reason": reason, "phase": "driving", "wait_remaining_s": 0.0}


def test_yukselen_aninda():
    """Yükselen karar (slow→dur→acildurus) ANINDA geçer — güvenlik gecikmez."""
    f = CikisDebounce(3)
    assert f.filtre(_d("slow"))["karar"] == "slow"
    assert f.filtre(_d("dur"))["karar"] == "dur"
    assert f.filtre(_d("acildurus"))["karar"] == "acildurus"
    print("  ✓ yükselen karar anında geçti")


def test_tek_tick_flicker_bastirilir():
    """E3 imzası: dur içinde tek-tick cruise flicker'ı yayına YANSIMAZ."""
    f = CikisDebounce(3)
    f.filtre(_d("dur"))
    out = f.filtre(_d("normal"))          # 1. alçalan tick — bastır
    assert out["karar"] == "dur", out
    out = f.filtre(_d("dur"))             # flicker bitti, dur geri
    assert out["karar"] == "dur"
    assert f.aday is None, "dur'a dönüş aday sayacını sıfırlamalı"
    print("  ✓ tek-tick flicker bastırıldı (dur kesintisiz yayınlandı)")


def test_istikrarli_alcalan_gecer():
    """Gerçek düşüş: K tick üst üste aynı alçalan karar → geçer."""
    f = CikisDebounce(3)
    f.filtre(_d("dur"))
    assert f.filtre(_d("normal"))["karar"] == "dur"    # 1
    assert f.filtre(_d("normal"))["karar"] == "dur"    # 2
    assert f.filtre(_d("normal"))["karar"] == "normal" # 3 → geçti
    print("  ✓ 3 tick istikrarlı alçalma geçti")


def test_alcalan_aday_degisirse_sifirlanir():
    """Alçalan aday değişirse (normal→slow) sayaç yeni adayla baştan başlar."""
    f = CikisDebounce(3)
    f.filtre(_d("dur"))
    f.filtre(_d("normal"))                              # aday=normal,1
    f.filtre(_d("slow"))                                # aday=slow,1
    assert f.filtre(_d("slow"))["karar"] == "dur"       # slow,2 → hâlâ dur
    assert f.filtre(_d("slow"))["karar"] == "slow"      # slow,3 → geçti
    print("  ✓ aday değişiminde sayaç sıfırlandı")


def test_esit_siddet_gecer():
    """Eşit şiddet (sol↔sag, aynı karar) anında geçer — manevra komutu tutulmaz."""
    f = CikisDebounce(3)
    f.filtre(_d("sol"))
    assert f.filtre(_d("sag"))["karar"] == "sag"
    # aynı karar yeni reason'la içerik tazeler
    out = f.filtre(_d("sag", reason="yeni"))
    assert out["reason"] == "yeni"
    print("  ✓ eşit şiddet anında geçti, içerik tazelendi")


def test_kapali_mod():
    """ticks=0 → filtre şeffaf (kapalı)."""
    f = CikisDebounce(0)
    f.filtre(_d("dur"))
    assert f.filtre(_d("normal"))["karar"] == "normal"
    print("  ✓ ticks=0 şeffaf")


def test_muhur_statik_dur_inisi():
    """P0 №3 inişi (acildurus→dur, alçalan): K tick sonra geçer — control'ün
    20 s'lik DUR-kaçış penceresi yanında 0.3 s gecikme ihmal edilebilir."""
    f = CikisDebounce(3)
    f.filtre(_d("acildurus", reason="emergency_latched"))
    for _ in range(2):
        assert f.filtre(_d("dur", reason="muhur_statik_dur"))["karar"] == "acildurus"
    out = f.filtre(_d("dur", reason="muhur_statik_dur"))
    assert out["karar"] == "dur" and out["reason"] == "muhur_statik_dur"
    print("  ✓ statik iniş 3 tick'te geçti (reason korunarak)")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fail = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            fail += 1
            print(f"  ✗ {t.__name__}: {e}")
    print("=" * 50)
    print("OK: hepsi geçti" if fail == 0 else f"FAIL: {fail}/{len(tests)}")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
