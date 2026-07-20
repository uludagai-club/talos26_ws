# -*- coding: utf-8 -*-
"""Yayın-öncesi ASİMETRİK çıkış-debounce (P1 №6, inceleme 2026-07-16 E3-O2).

BT hafızasız selector olduğundan her tick kararı sıfırdan üretir; hiçbir dalda
çıkış-debounce yoktu → tek-tick 'bayat flicker' inişleri (E3: mesaj periyodu ≈
tazelik eşiği çakışması, bölütlerin 2/3'ü) doğrudan /karar'a yansıyor, control
fren/gaz chatter'ına çeviriyordu (10 s'de 46 değişime kadar).

Kural (asimetrik — güvenlik yönü ANINDA):
  - Yükselen veya eşit şiddet (örn. slow→dur, normal→acildurus): ANINDA geçer.
  - Alçalan şiddet (örn. dur→normal): ancak K tick üst üste aynı alçalan karar
    üretilirse geçer; o zamana dek önceki karar yayınlanmaya devam eder.

Uygulama nüansı (rapor E3-O2 ⚠): filtre bb.last_decision'ı YAYIN ÖNCESİ
mutasyonlamalı — karar_bt_node tick döngüsünde tree.tick() ile
bridge.publish_decision() arasında çağrılır; böylece RerouteManager, CSV ve
trace da ham churn'ü değil debounced kararı görür.
"""
from __future__ import annotations

# Karar şiddet sırası: yükseklik = güvenlik önceliği. sol/sag manevra komutları
# slow ile dur arasında (manevra kesilmesin ama dur/acil her zaman bassın).
KARAR_SIDDET = {
    "normal": 0,
    "slow": 1,
    "sol": 2,
    "sag": 2,
    "dur": 3,
    "acildurus": 4,
}


class CikisDebounce:
    """Karar dict'i filtresi. `filtre(decision)` yayınlanacak dict'i döndürür."""

    def __init__(self, ticks: int):
        self.ticks = int(ticks)
        self.held = None       # yayında tutulan son karar dict'i
        self.aday = None       # alçalan aday karar (string)
        self.aday_count = 0

    def filtre(self, decision: dict) -> dict:
        if self.ticks <= 0 or self.held is None:
            self.held = dict(decision)
            return decision

        yeni = decision.get("karar", "normal")
        eski = self.held.get("karar", "normal")

        if KARAR_SIDDET.get(yeni, 0) >= KARAR_SIDDET.get(eski, 0):
            # Eşit/yükselen → anında geç (aynı karar da içerik/reason tazeler)
            self.held = dict(decision)
            self.aday = None
            self.aday_count = 0
            return decision

        # Alçalan → K tick aynı alçalan kararda istikrar iste
        if yeni == self.aday:
            self.aday_count += 1
        else:
            self.aday = yeni
            self.aday_count = 1

        if self.aday_count >= self.ticks:
            self.held = dict(decision)
            self.aday = None
            self.aday_count = 0
            return decision

        # Henüz istikrar yok → önceki kararı tutmaya devam et
        return dict(self.held)
