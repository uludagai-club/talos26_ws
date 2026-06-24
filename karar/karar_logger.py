#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""karar_logger.py — karar mekanizması (Behavior Tree) için kalıcı tanı logu.

NEDEN AYRI BİR LOGGER?
    `hedef_logger.py` ile BİREBİR aynı tasarım/biçimde, karar tarafının da
    sahada teşhis edilebilmesi için. Dün hedef için yaptığımız sistemin ikizi:
    aynı dizin ağacı (logs/<RUN_ID>/<bileşen>/), aynı events.jsonl şeması
    ({"type","t","t_iso","ros_t",...}), aynı throttle'lı pose/trace izi.
    Böylece `hedef/events.jsonl` ile `karar/events.jsonl` yan yana, aynı `jq`
    filtreleriyle ve aynı zaman ekseninde (ros_t = sim saati) okunur.

    NOT: karar_bt zaten ortak `TalosLogger` ile düz `karar.csv` telemetri yazıyor;
    bu modül ONU DEĞİŞTİRMEZ, yanına TİPLİ olay + zengin karar izi ekler
    (karar değişimi, engel kaçış yönü + çapraz-çarpım gerekçesi, sollama
    başlangıç/dönüş, Ackermann hesabı). İkisi aynı RUN_ID ağacında birlikte yaşar.

KALICILIK
    Loglar konteyner içinde TALOS_LOG_ROOT (varsayılan /app/logs) altına yazılır.
    docker-compose bunu host'taki ./logs dizinine bind-mount eder (karar-node:
    `./logs:/app/logs`). Her çalışma RUN_ID (baslat.sh UTC damgası) alt dizinine
    ayrılır → logs/<RUN_ID>/karar/.

ÇIKTI (her RUN_ID için iki dosya)
    trace.csv      — yüksek frekanslı karar izi (varsayılan 5 Hz'e kısılır)
    events.jsonl   — JSON Lines; her satır bir olay (karar_change / engel_kacis /
                     sollama_basla / sollama_donus / ackermann / init).

TASARIM
    Bu modül karar düğümünü ASLA çökertmemeli — her yazma try/except ile sarılı,
    hata olursa sessizce yutulur (stderr'e tek satır uyarı). karar_bt_node bu
    modülü import edemese de çalışmaya devam eder (orada da try/except var).

ÖRNEK FİLTRELEME (host'ta ./logs/<RUN_ID>/karar/ içinde)
    # Sadece engel kaçış yönü kararlarını gör:
    grep '"type": "engel_kacis"' events.jsonl | jq .
    # Yol-bilinçli (rota verisiyle) seçilen taraflar:
    jq -c 'select(.type=="engel_kacis" and .kaynak=="rota")' events.jsonl
    # Sollama başlat/dönüş zaman çizelgesi:
    jq -c 'select(.type|test("sollama"))' events.jsonl
"""

import os
import sys
import json
import csv
import math
import time
import threading

# rospy opsiyonel: container içinde mevcut → ros_t (sim-time) ekleriz.
# Standalone testte yoksa ros_t alanı None olur, logger yine çalışır.
try:
    import rospy as _rospy
except Exception:  # noqa: BLE001
    _rospy = None


class KararLogger:
    def __init__(self, run_id=None, root=None, trace_hz=5.0):
        """trace_hz: trace.csv yazım üst frekansı. <=0 verilirse kısıtlama
        KAPALI (her çağrıda yazar) — sadece test/hata ayıklama için."""
        self.ok = False
        self._lock = threading.Lock()
        self._trace_min_dt = (1.0 / trace_hz) if trace_hz and trace_hz > 0 else 0.0
        self._last_trace_t = 0.0
        self._trace_writer = None
        self._trace_file = None
        self._events_file = None

        run_id = run_id or os.environ.get("RUN_ID", "dev")
        root = root or os.environ.get("TALOS_LOG_ROOT", "/app/logs")

        # Birincil hedef yazılamazsa /tmp'ye düş — log uğruna node çökmesin.
        for base in (root, "/tmp/karar_logs"):
            try:
                run_dir = os.path.join(base, run_id, "karar")
                os.makedirs(run_dir, exist_ok=True)
                self._open(run_dir)
                self.ok = True
                self._dir = run_dir
                # init olayı: ok=True olduktan SONRA yazılmalı (log_event ok kontrol eder)
                self.log_event("init", run_id=run_id, pid=os.getpid())
                sys.stderr.write(f"[karar_logger] Log dizini: {run_dir}\n")
                break
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"[karar_logger] {base} açılamadı: {e}\n")
                continue

    # ------------------------------------------------------------------ #
    def _open(self, run_dir):
        self._events_file = open(
            os.path.join(run_dir, "events.jsonl"), "a", buffering=1, encoding="utf-8"
        )
        trace_path = os.path.join(run_dir, "trace.csv")
        write_header = not os.path.exists(trace_path) or os.path.getsize(trace_path) == 0
        self._trace_file = open(trace_path, "a", buffering=1, newline="", encoding="utf-8")
        self._trace_writer = csv.writer(self._trace_file)
        if write_header:
            self._trace_writer.writerow([
                "t_unix", "t_iso", "ros_t",
                "x", "y", "yaw_deg", "speed_kmh",
                "karar", "reason", "phase",
                "engel_present", "d_center", "d_left", "d_right", "angle_deg",
                "kacis_yon", "overtake_active",
                "hedef_x", "hedef_y",
            ])

    # ------------------------------------------------------------------ #
    @staticmethod
    def _iso(t):
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))

    @staticmethod
    def _ros_t():
        # use_sim_time=true iken sim saatini verir → rosbag/candump ve hedef logu
        # ile hizalanır. Node init olmadan / clock yokken patlamasın diye korumalı.
        if _rospy is None:
            return None
        try:
            return round(_rospy.Time.now().to_sec(), 3)
        except Exception:  # noqa: BLE001
            return None

    def log_event(self, etype, **fields):
        """events.jsonl'e tek satır JSON olay yaz (her zaman, kısma yok)."""
        if not self.ok or self._events_file is None:
            return
        try:
            t = time.time()
            rec = {"type": etype, "t": round(t, 3), "t_iso": self._iso(t),
                   "ros_t": self._ros_t()}
            rec.update(fields)
            line = json.dumps(rec, ensure_ascii=False, default=self._jsonable)
            with self._lock:
                self._events_file.write(line + "\n")
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"[karar_logger] log_event hata: {e}\n")

    def trace_due(self):
        """Throttle hint: bir sonraki trace örneği yazılmaya hazır mı?
        Lock'suz okuma (kasıtlı) — yalnızca pahalı snapshot'ı gereksiz yapmamak
        için kapı; gerçek throttle kararı log_trace içinde lock altında verilir."""
        return self.ok and (time.time() - self._last_trace_t) >= self._trace_min_dt

    def log_trace(self, *, x, y, yaw, speed_kmh, karar, reason, phase,
                  engel_present, d_center, d_left, d_right, angle_deg,
                  kacis_yon=None, overtake_active=False,
                  hedef_x=None, hedef_y=None):
        """Karar izini trace.csv'ye yaz — trace_hz'e göre kısılır."""
        if not self.ok or self._trace_writer is None:
            return
        try:
            t = time.time()
            yaw_deg = math.degrees(yaw) if yaw is not None else None
            with self._lock:
                # Throttle kararı + _last_trace_t güncellemesi lock altında →
                # oku-kontrol-yaz atomik (çift yazım yok)
                if (t - self._last_trace_t) < self._trace_min_dt:
                    return
                self._last_trace_t = t
                self._trace_writer.writerow([
                    round(t, 3), self._iso(t), self._ros_t(),
                    self._r(x), self._r(y), self._r(yaw_deg, 2), self._r(speed_kmh, 2),
                    karar, reason, phase,
                    1 if engel_present else 0,
                    self._r(self._fin(d_center)), self._r(self._fin(d_left)),
                    self._r(self._fin(d_right)), self._r(angle_deg, 2),
                    kacis_yon or "", 1 if overtake_active else 0,
                    self._r(hedef_x), self._r(hedef_y),
                ])
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"[karar_logger] log_trace hata: {e}\n")

    # --- Karara özel kolay yazıcılar (events.jsonl) ------------------- #
    def log_karar_change(self, prev, new, reason, **extra):
        """Karar değiştiğinde tek olay — hedef'in 'wp_gecis'ine paralel."""
        self.log_event("karar_change", prev=prev, new=new, reason=reason, **extra)

    def log_kacis(self, taraf, kaynak, **extra):
        """Engelden kaçış yönü seçimi. kaynak='rota' (yol-bilinçli çapraz-çarpım)
        ya da 'yan_sektor' (rota yokken en açık taraf). extra: cross, obstacle
        dünya konumu, rota başlığı, yanal offset, vb."""
        self.log_event("engel_kacis", taraf=taraf, kaynak=kaynak, **extra)

    def log_overtake(self, faz, **extra):
        """(DEPRECATED) Sollama yaşam döngüsü — eski overtake.py içindi."""
        self.log_event(f"sollama_{faz}", **extra)

    def log_reroute(self, faz, **extra):
        """Cone reroute yaşam döngüsü (§16): faz='blok'|'serbest'|'zaman_asimi'.
        extra: cone_dunya, yaricap_m, neden, vb."""
        self.log_event(f"reroute_{faz}", **extra)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _jsonable(o):
        try:
            return list(o)
        except Exception:
            return str(o)

    @staticmethod
    def _r(v, nd=2):
        if isinstance(v, (int, float)) and math.isfinite(v):
            return round(v, nd)
        return None

    @staticmethod
    def _fin(v):
        """inf → None (CSV'de boş); sonlu değer aynen."""
        if v is None:
            return None
        return v if math.isfinite(v) else None

    def close(self):
        # Her dosyayı AYRI try ile kapat: biri hata verirse diğeri yine kapansın.
        for f in (self._events_file, self._trace_file):
            try:
                if f:
                    f.close()
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"[karar_logger] close hata: {e}\n")
