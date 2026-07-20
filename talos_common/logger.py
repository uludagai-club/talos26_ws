"""TalosLogger — KTR §9.14.2 + new plan §2.9 birleştirilmiş ortak logger.

Servisler arası import için tek nokta. Üç tip kayıt:
    metric(**fields)  -> per-component CSV (telemetri)
    event(level, msg) -> per-component .log + global events_<run>.jsonl
    health(**fields)  -> system/health.csv (1 Hz)

Zaman damgaları KTR §9.14.3 uyarınca: ts_wall_iso + ts_ros_sec + ts_mono_ns.
Disk fail-over KTR §8.15.3: birincil yol erişilemezse /tmp/talos_logs/ kullan.
"""

from __future__ import annotations

import csv
import json
import os
import sys
import time
import threading
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
import logging

try:
    import rospy  # noqa: F401
    _HAS_ROSPY = True
except Exception:
    _HAS_ROSPY = False


# KTR §9.14.2 — L2 RotatingFileHandler boyutları
_LOG_MAX_BYTES = 50 * 1024 * 1024
_LOG_BACKUP_COUNT = 5
_FALLBACK_DIR = "/tmp/talos_logs"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _ros_now_sec() -> float:
    if _HAS_ROSPY:
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            return 0.0
    return 0.0


def _mono_ns() -> int:
    return time.monotonic_ns()


def _resolve_run_id() -> str:
    rid = os.environ.get("RUN_ID")
    if rid:
        return rid
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _resolve_root() -> str:
    """Birincil log kökü; erişilemezse /tmp/talos_logs."""
    root = os.environ.get("TALOS_LOG_ROOT", "/app/logs")
    try:
        os.makedirs(root, exist_ok=True)
        # write probe
        probe = os.path.join(root, ".write_probe")
        with open(probe, "w") as f:
            f.write("ok")
        os.remove(probe)
        return root
    except OSError:
        os.makedirs(_FALLBACK_DIR, exist_ok=True)
        return _FALLBACK_DIR


class TalosLogger:
    """Per-component logger.

    Args:
        component: "karar", "engel", "konum", "control", "can_bridge", ...
        schema: opsiyonel CSV alan listesi. Verilmezse metric() ilk çağrısı
                fields anahtarlarından otomatik header üretir.
        run_id: yoksa env RUN_ID veya UTC zaman damgasından üretilir.
    """

    _EVENT_LEVELS = {"INFO", "WARN", "ERR", "DEBUG"}
    _global_lock = threading.Lock()

    def __init__(self, component: str, schema=None, run_id=None):
        self.component = component
        self.run_id = run_id or _resolve_run_id()
        self.schema = list(schema) if schema else None

        root = _resolve_root()
        self.run_dir = os.path.join(root, self.run_id)
        self.comp_dir = os.path.join(self.run_dir, component)
        os.makedirs(self.comp_dir, exist_ok=True)

        # CSV (metric)
        self.csv_path = os.path.join(self.comp_dir, f"{component}.csv")
        self._csv_lock = threading.Lock()
        self._csv_initialized = os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0

        # Olay log (event) — RotatingFileHandler
        self.log_path = os.path.join(self.comp_dir, f"{component}.log")
        self._py_logger = logging.getLogger(f"talos.{component}")
        self._py_logger.setLevel(logging.DEBUG)
        if not any(getattr(h, "_talos_owned", False) for h in self._py_logger.handlers):
            handler = RotatingFileHandler(
                self.log_path,
                maxBytes=_LOG_MAX_BYTES,
                backupCount=_LOG_BACKUP_COUNT,
            )
            handler.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
            handler._talos_owned = True
            self._py_logger.addHandler(handler)

        # L5 — global events JSONL
        self.events_path = os.path.join(self.run_dir, f"events_{self.run_id}.jsonl")
        self._events_lock = TalosLogger._global_lock

        # Health CSV (paylaşımlı)
        sys_dir = os.path.join(self.run_dir, "system")
        os.makedirs(sys_dir, exist_ok=True)
        self.health_path = os.path.join(sys_dir, "health.csv")
        self._health_lock = TalosLogger._global_lock
        self._health_header = None  # M3: header init/ilk yazımda cache'lenir
        self._health_initialized = os.path.exists(self.health_path) and os.path.getsize(self.health_path) > 0
        if self._health_initialized:
            try:
                with open(self.health_path, "r", newline="") as f:
                    self._health_header = next(csv.reader(f), None)
            except OSError:
                self._health_header = None

        self._start_time = time.time()

    # ------------------------------------------------------------------
    # metric — telemetri CSV satırı
    # ------------------------------------------------------------------
    def metric(self, **fields):
        """Bir metrik satırı yaz. ts_* alanları otomatik eklenir."""
        row = {
            "ts_wall_iso": _utc_iso(),
            "ts_ros_sec": f"{_ros_now_sec():.6f}",
            "ts_mono_ns": _mono_ns(),
        }
        row.update(fields)

        with self._csv_lock:
            if not self._csv_initialized:
                if self.schema:
                    header = ["ts_wall_iso", "ts_ros_sec", "ts_mono_ns"] + [
                        f for f in self.schema if f not in ("ts_wall_iso", "ts_ros_sec", "ts_mono_ns")
                    ]
                else:
                    header = list(row.keys())
                self.schema = header
                with open(self.csv_path, "w", newline="") as f:
                    csv.writer(f).writerow(header)
                self._csv_initialized = True
            elif self.schema and "ts_wall_iso" not in self.schema:
                # Var olan dosyaya devam (container restart, aynı RUN_ID): header'ı
                # dosyadan yükle; yoksa restart sonrası satırlar ts_* kolonsuz
                # yazılıyordu (health.csv ile aynı kalıp).
                try:
                    with open(self.csv_path, "r", newline="") as f:
                        hdr = next(csv.reader(f), None)
                except OSError:
                    hdr = None
                self.schema = hdr if hdr else (
                    ["ts_wall_iso", "ts_ros_sec", "ts_mono_ns"] + list(self.schema))

            ordered = [str(row.get(k, "")) for k in self.schema]
            try:
                with open(self.csv_path, "a", newline="") as f:
                    csv.writer(f).writerow(ordered)
            except OSError:
                pass

    # ------------------------------------------------------------------
    # event — insan-okur log + JSONL
    # ------------------------------------------------------------------
    def event(self, level: str, msg: str, **extra):
        level = (level or "INFO").upper()
        if level not in self._EVENT_LEVELS:
            level = "INFO"

        py_level = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARN": logging.WARNING,
            "ERR": logging.ERROR,
        }[level]
        self._py_logger.log(py_level, msg)

        record = {
            "ts_wall_iso": _utc_iso(),
            "ts_ros_sec": _ros_now_sec(),
            "ts_mono_ns": _mono_ns(),
            "component": self.component,
            "level": level,
            "msg": msg,
        }
        if extra:
            record.update(extra)

        line = json.dumps(record, ensure_ascii=False, default=str)
        with self._events_lock:
            try:
                with open(self.events_path, "a") as f:
                    f.write(line + "\n")
            except OSError:
                pass

    # ------------------------------------------------------------------
    # health — system/health.csv 1 Hz satırı
    # ------------------------------------------------------------------
    def health(self, **fields):
        row = {
            "ts_wall_iso": _utc_iso(),
            "component": self.component,
            "uptime_s": f"{time.time() - self._start_time:.1f}",
        }
        row.update(fields)

        with self._health_lock:
            if not self._health_initialized:
                self._health_header = list(row.keys())
                try:
                    with open(self.health_path, "w", newline="") as f:
                        csv.writer(f).writerow(self._health_header)
                except OSError:
                    pass
                self._health_initialized = True

            header = self._health_header or list(row.keys())
            ordered = [str(row.get(k, "")) for k in header]
            try:
                with open(self.health_path, "a", newline="") as f:
                    csv.writer(f).writerow(ordered)
            except OSError:
                pass

    # ------------------------------------------------------------------
    # health worker — 1 Hz background thread (N3)
    # ------------------------------------------------------------------
    def start_health_loop(self, interval_s: float = 1.0, **static_fields):
        """Arka plan thread'i 1 Hz health() satırı yazar.

        Args:
            interval_s: yazım periyodu (saniye)
            **static_fields: her satıra eklenen sabit alanlar (örn. node_name)
        """
        if getattr(self, "_health_thread", None) is not None:
            return
        self._health_stop = threading.Event()

        def _loop():
            while not self._health_stop.is_set():
                try:
                    self.health(**static_fields)
                except Exception:
                    pass
                self._health_stop.wait(interval_s)

        t = threading.Thread(target=_loop, name=f"talos-health-{self.component}", daemon=True)
        t.start()
        self._health_thread = t

    def stop_health_loop(self):
        if hasattr(self, "_health_stop"):
            self._health_stop.set()

    # ------------------------------------------------------------------
    # convenience — eski Logger.log()/csv() ile köprü
    # ------------------------------------------------------------------
    def log(self, msg: str):
        self.event("INFO", msg)
