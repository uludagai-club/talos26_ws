#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Canlı parametre izleyici — docker restart GEREKTİRMEDEN parametre değişikliği.

Nasıl çalışır
-------------
- Her servis dosyası parametre VARSAYILANLARINI kendi üst bloğunda tutar
  (hedef_yoneticisi.py stili). Bu modül onları EZMEZ; yalnız override eder.
- Merkez dosya: config/canli_params.yaml (host'ta düzenlenir, container'lara
  read-only mount edilir). Bir satırın yorumunu kaldırıp değer yazınca ~1 sn
  içinde ilgili servise uygulanır; satırı tekrar yoruma alınca kod içindeki
  varsayılana GERİ DÖNER.
- Dosya bozuksa (YAML hatası) eski değerler korunur ve uyarı loglanır —
  izleyici hiçbir koşulda node'u düşürmez.

Kullanım (servis dosyasının üst parametre bloğundan hemen sonra):

    try:
        from talos_common.canli_params import canli_parametre_izle
        _canli = canli_parametre_izle("engel", globals())
    except Exception as _e:
        _canli = None
        print(f"[canli_params] izleyici yok, statik parametreler: {_e}", flush=True)

`degisiklik_cb` ile türetilmiş değerler (örn. MAX_SPEED_MS) yeniden hesaplanır
veya canlı nesnelere (PID kazançları) uygulanır. `sinirlar={"AD": (min, max)}`
güvenlik-kritik parametreleri kelepçeler.

rospy'ye BAĞIMLI DEĞİLDİR: host node'ları (ground_filter) ve testler de kullanır.
"""

import os
import threading


def _config_adaylari(dosya):
    """Sırayla denenecek yaml yolları. İlk VAR OLAN kazanır; hiçbiri yoksa
    ilk aday izlenmeye devam eder (dosya sonradan oluşturulabilir)."""
    adaylar = []
    if dosya:
        adaylar.append(dosya)
    env = os.environ.get("TALOS_CANLI_PARAMS")
    if env:
        adaylar.append(env)
    # talos_common/.. = container'da /app, host'ta talos26_ws kökü
    kok = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    adaylar.append(os.path.join(kok, "config", "canli_params.yaml"))
    return adaylar


class CanliParamsIzleyici:
    """Tek servisin parametre bölümünü izler ve modül globals'ına uygular."""

    def __init__(self, servis, hedef_globals, degisiklik_cb=None,
                 sinirlar=None, dosya=None, period_s=1.0):
        self.servis = servis
        self._g = hedef_globals
        self._cbs = [degisiklik_cb] if degisiklik_cb else []
        self._sinirlar = dict(sinirlar or {})
        self._adaylar = _config_adaylari(dosya)
        self._period = max(0.05, float(period_s))
        self._orijinaller = {}      # override edilen anahtarların kod-içi varsayılanları
        self._aktif_override = set()
        self._bilinmeyen_uyarildi = set()
        self._son_imza = None       # (yol, mtime_ns, boyut)
        self._son_hata = None
        self._dur = threading.Event()
        self._thread = None

    # -- dış API ---------------------------------------------------------

    def baslat(self):
        self._kontrol_et()  # başlangıç override'ları senkron uygula
        self._thread = threading.Thread(
            target=self._dongu, name=f"canli-params-{self.servis}", daemon=True)
        self._thread.start()
        return self

    def durdur(self):
        self._dur.set()

    def degisiklik_ekle(self, cb):
        """Sonradan callback kaydı (örn. sınıf örneği kurulduktan sonra)."""
        if cb:
            self._cbs.append(cb)

    # -- iç mekanik -------------------------------------------------------

    def _log(self, mesaj):
        print(f"[canli_params][{self.servis}] {mesaj}", flush=True)

    def _dongu(self):
        while not self._dur.wait(self._period):
            try:
                self._kontrol_et()
            except Exception as e:  # izleyici asla node'u düşürmez
                self._hata_logla(f"beklenmeyen hata: {e}")

    def _hata_logla(self, mesaj):
        if mesaj != self._son_hata:
            self._log(f"UYARI: {mesaj} — eski değerler korunuyor")
            self._son_hata = mesaj

    def _yol_sec(self):
        for yol in self._adaylar:
            if os.path.isfile(yol):
                return yol
        return None

    def _kontrol_et(self):
        yol = self._yol_sec()
        if yol is None:
            self._son_imza = None
            return
        try:
            st = os.stat(yol)
            imza = (yol, st.st_mtime_ns, st.st_size)
        except OSError:
            return
        if imza == self._son_imza:
            return
        try:
            import yaml
            with open(yol, "r") as f:
                veri = yaml.safe_load(f) or {}
        except Exception as e:
            self._hata_logla(f"{yol} okunamadı/bozuk: {e}")
            return
        self._son_imza = imza
        self._son_hata = None
        bolum = veri.get(self.servis)
        if bolum is None:
            bolum = {}
        if not isinstance(bolum, dict):
            self._hata_logla(f"'{self.servis}' bölümü sözlük değil ({type(bolum).__name__})")
            return
        self._uygula(bolum, yol)

    def _uygula(self, bolum, yol):
        degisenler = {}

        # 1) Dosyadan kaldırılan (yoruma alınan) override'lar → varsayılana dön
        for ad in sorted(self._aktif_override - set(bolum.keys())):
            eski = self._g.get(ad)
            varsayilan = self._orijinaller[ad]
            self._g[ad] = varsayilan
            self._aktif_override.discard(ad)
            degisenler[ad] = varsayilan
            self._log(f"{ad}: {eski!r} → {varsayilan!r} (override kalktı, varsayılana döndü)")

        # 2) Dosyadaki override'ları uygula
        for ad, yeni in bolum.items():
            if ad not in self._g:
                if ad not in self._bilinmeyen_uyarildi:
                    self._log(f"UYARI: bilinmeyen parametre '{ad}' — yok sayıldı "
                              f"(kod üst bloğundaki adla birebir aynı olmalı)")
                    self._bilinmeyen_uyarildi.add(ad)
                continue
            varsayilan = self._orijinaller.get(ad, self._g[ad])
            yeni = self._donustur(ad, yeni, varsayilan)
            if yeni is _GECERSIZ:
                continue
            yeni = self._kelepcele(ad, yeni)
            mevcut = self._g[ad]
            if ad not in self._aktif_override:
                self._orijinaller[ad] = mevcut
                self._aktif_override.add(ad)
            if not self._esit(mevcut, yeni):
                self._g[ad] = yeni
                degisenler[ad] = yeni
                self._log(f"{ad}: {mevcut!r} → {yeni!r}  ({os.path.basename(yol)})")

        # 3) Türetilmiş değer/canlı nesne callback'leri
        if degisenler:
            for cb in self._cbs:
                try:
                    cb(dict(degisenler))
                except Exception as e:
                    self._log(f"UYARI: degisiklik_cb hatası: {e}")

    @staticmethod
    def _esit(a, b):
        try:
            karsilastirma = (a == b)
            # numpy dizileri eleman-bazlı döner → tekile indir
            if hasattr(karsilastirma, "all"):
                return bool(karsilastirma.all())
            return bool(karsilastirma)
        except Exception:
            return False

    def _donustur(self, ad, yeni, varsayilan):
        """YAML değerini kod-içi varsayılanın tipine güvenle çevir."""
        try:
            if isinstance(varsayilan, bool):
                if isinstance(yeni, bool):
                    return yeni
                self._log(f"UYARI: {ad} bool bekler, {type(yeni).__name__} geldi — yok sayıldı")
                return _GECERSIZ
            if isinstance(varsayilan, float):
                if isinstance(yeni, (int, float)) and not isinstance(yeni, bool):
                    return float(yeni)
                self._log(f"UYARI: {ad} sayı bekler, {yeni!r} geldi — yok sayıldı")
                return _GECERSIZ
            if isinstance(varsayilan, int):
                if isinstance(yeni, bool):
                    self._log(f"UYARI: {ad} tamsayı bekler, bool geldi — yok sayıldı")
                    return _GECERSIZ
                if isinstance(yeni, int):
                    return yeni
                if isinstance(yeni, float) and float(yeni).is_integer():
                    return int(yeni)
                self._log(f"UYARI: {ad} tamsayı bekler, {yeni!r} geldi — yok sayıldı")
                return _GECERSIZ
            if isinstance(varsayilan, str):
                return str(yeni)
            # numpy dizisi (örn. HSV eşikleri) → aynı dtype ile diziye çevir
            if hasattr(varsayilan, "dtype") and isinstance(yeni, (list, tuple)):
                import numpy as np
                return np.asarray(yeni, dtype=varsayilan.dtype)
            return yeni  # dict/list/None: olduğu gibi
        except Exception as e:
            self._log(f"UYARI: {ad} dönüştürülemedi ({e}) — yok sayıldı")
            return _GECERSIZ

    def _kelepcele(self, ad, yeni):
        sinir = self._sinirlar.get(ad)
        if sinir is None or not isinstance(yeni, (int, float)) or isinstance(yeni, bool):
            return yeni
        alt, ust = sinir
        if yeni < alt or yeni > ust:
            kirpik = min(max(yeni, alt), ust)
            self._log(f"UYARI: {ad}={yeni!r} güvenlik sınırı [{alt}, {ust}] dışında → {kirpik!r} kullanılıyor")
            return type(yeni)(kirpik) if isinstance(yeni, float) else kirpik
        return yeni


class _Gecersiz:
    __slots__ = ()


_GECERSIZ = _Gecersiz()


def canli_parametre_izle(servis, hedef_globals, degisiklik_cb=None,
                         sinirlar=None, dosya=None, period_s=1.0):
    """İzleyiciyi kur + başlat. HİÇBİR KOŞULDA exception fırlatmaz;
    kurulamazsa None döner (servis statik parametrelerle çalışmaya devam eder)."""
    try:
        izleyici = CanliParamsIzleyici(
            servis, hedef_globals, degisiklik_cb=degisiklik_cb,
            sinirlar=sinirlar, dosya=dosya, period_s=period_s)
        return izleyici.baslat()
    except Exception as e:
        print(f"[canli_params][{servis}] izleyici başlatılamadı: {e} — "
              f"statik parametrelerle devam", flush=True)
        return None
