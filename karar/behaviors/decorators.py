"""Decorator'lar — sensör tazeliği, debounce.

py_trees.decorators üzerinden tek-child wrap'leri.
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status

from bb import Blackboard


class Debounce(py_trees.decorators.Decorator):
    """Child SUCCESS olduğunda, key adlı sayaç artar; N tick üst üste SUCCESS
    olursa SUCCESS döner. FAILURE olunca sayaç sıfırlanır.
    Aksi halde FAILURE döner — yani 'şu an karar üretme'.

    ÇIKIŞ-TUTMASI (hold_ticks > 0, 2026-07-22 karar-kararsızlığı fix):
    Giriş debounce'u (min_consecutive) SİMETRİK değil — bir kez angaje olduktan
    (SUCCESS eşiğine ulaştıktan) sonra child FAILURE olsa bile karar `hold_ticks`
    tick boyunca SUCCESS dönmeye DEVAM eder. Amaç: kısa algı boşlukları (ör. tünel
    duvarı / duba detektör titremesi, /obstacles/poses bir tick boş gelmesi) dalı
    anında düşürüp `normal`'a ittirmesin → karar `normal↔dur/slow` flip-flop'u
    kesilir. Her SUCCESS tutma penceresini tazeler; pencere biterse (hold=0) dal
    düşer ve yeniden angaje olmak için min_consecutive baştan gerekir.

    hold_ticks=0 (varsayılan) → eski davranış (bire-bir); geriye tam uyumlu.
    Güvenlik yönü: tutma yalnız 'engaged' state'i UZATIR (daha temkinli kararı
    tutar); dur/acildurus zaten ağaçta ÜST dallardan gelir, bu dekoratör onları
    geciktirmez.
    """

    def __init__(self, name: str, child, bb: Blackboard, key: str,
                 min_consecutive: int, hold_ticks: int = 0):
        super().__init__(name=name, child=child)
        self.bb = bb
        self.key = key
        self.min_consecutive = int(min_consecutive)
        self.hold_ticks = max(0, int(hold_ticks))
        self._hold_key = f"{key}__hold"   # kalan tutma tick'i (bb.state.debounce'ta)

    def update(self):
        deb = self.bb.state.debounce
        child_status = self.decorated.status
        if child_status == Status.SUCCESS:
            deb[self.key] = deb.get(self.key, 0) + 1
            if deb[self.key] >= self.min_consecutive:
                deb[self._hold_key] = self.hold_ticks   # tutma penceresini arm/tazele
                return Status.SUCCESS
            return Status.FAILURE
        else:
            # Child FAILURE → giriş sayacını sıfırla, ama tutma penceresi varsa
            # angaje kararı kısa boşlukta koru (histerezis).
            deb[self.key] = 0
            hold = deb.get(self._hold_key, 0)
            if hold > 0:
                deb[self._hold_key] = hold - 1
                return Status.SUCCESS
            return Status.FAILURE
