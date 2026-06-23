#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hedef_logger.py — hedef_yoneticisi (D* rota yöneticisi) için kalıcı tanı logu.

AMAÇ
    Rotanın bazen neden "uzaktan" çizildiğini (start düğümünün araçtan
    kopuk seçilmesini) sahada teşhis edebilmek. Aracın konumu, yaw'ı, hangi
    WP'de olduğu, rotanın ne zaman/neden yeniden hesaplandığı ve seçilen
    start düğümünün araca uzaklığı kaydedilir.

KALICILIK
    Loglar konteyner içinde TALOS_LOG_ROOT (varsayılan /app/logs) altına
    yazılır. docker-compose bunu host'taki ./hedef/logs dizinine bind-mount
    eder; böylece konteyner kapanınca/silinince loglar host diskinde kalır.
    Her çalışma RUN_ID (baslat.sh'nin UTC damgası) alt dizinine ayrılır.

ÇIKTI (her RUN_ID için iki dosya)
    pose.csv       — yüksek frekanslı konum izi (varsayılan 5 Hz'e kısılır)
    events.jsonl   — JSON Lines; her satır bir olay (recalc / wp_gecis /
                     sapma / gorev_tamam / init). `jq` veya grep ile filtrelenir.

TASARIM
    Bu modül planner'ı ASLA çökertmemeli — her yazma try/except ile sarılı,
    hata olursa sessizce yutulur (stderr'e tek satır uyarı). hedef_yoneticisi
    bu modülü import edemezse de çalışmaya devam eder (orada try/except var).

ÖRNEK FİLTRELEME (host'ta ./hedef/logs/<RUN_ID>/ içinde)
    # Sadece rota yeniden hesaplama olaylarını gör:
    grep '"type": "recalc"' events.jsonl | jq .
    # start düğümü araçtan 8m'den uzak seçilen recalc'ler (uzaktan çizme!):
    jq -c 'select(.type=="recalc" and .dist_robot_start > 8)' events.jsonl
    # Belirli bir göreve giderken çizilen rotalar:
    jq -c 'select(.type=="recalc" and .task_name=="park")' events.jsonl
"""

import os
import sys
import json
import csv
import math
import time
import threading

# rospy opsiyonel: container içinde mevcut → ros_t (sim-time) ekleriz.
# Standalone testte yoksa ros_t alanı atlanır, logger yine çalışır.
try:
    import rospy as _rospy
except Exception:  # noqa: BLE001
    _rospy = None


class HedefLogger:
    def __init__(self, run_id=None, root=None, pose_hz=5.0):
        """pose_hz: pose.csv yazım üst frekansı. <=0 verilirse kısıtlama
        KAPALI (her çağrıda yazar) — sadece test/hata ayıklama için."""
        self.ok = False
        self._lock = threading.Lock()
        self._pose_min_dt = (1.0 / pose_hz) if pose_hz and pose_hz > 0 else 0.0
        self._last_pose_t = 0.0
        self._pose_writer = None
        self._pose_file = None
        self._events_file = None

        run_id = run_id or os.environ.get("RUN_ID", "dev")
        root = root or os.environ.get("TALOS_LOG_ROOT", "/app/logs")

        # Birincil hedef yazılamazsa /tmp'ye düş — log uğruna node çökmesin.
        for base in (root, "/tmp/hedef_logs"):
            try:
                run_dir = os.path.join(base, run_id, "hedef")
                os.makedirs(run_dir, exist_ok=True)
                self._open(run_dir)
                self.ok = True
                self._dir = run_dir
                # init olayı: ok=True olduktan SONRA yazılmalı (log_event ok kontrol eder)
                self.log_event("init", run_id=run_id, pid=os.getpid())
                sys.stderr.write(f"[hedef_logger] Log dizini: {run_dir}\n")
                break
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"[hedef_logger] {base} açılamadı: {e}\n")
                continue

    # ------------------------------------------------------------------ #
    def _open(self, run_dir):
        self._events_file = open(
            os.path.join(run_dir, "events.jsonl"), "a", buffering=1, encoding="utf-8"
        )
        pose_path = os.path.join(run_dir, "pose.csv")
        write_header = not os.path.exists(pose_path) or os.path.getsize(pose_path) == 0
        self._pose_file = open(pose_path, "a", buffering=1, newline="", encoding="utf-8")
        self._pose_writer = csv.writer(self._pose_file)
        if write_header:
            self._pose_writer.writerow([
                "t_unix", "t_iso", "ros_t", "x", "y", "yaw_rad", "yaw_deg",
                "task_idx", "wp_idx", "n_path", "dist_to_wp", "dist_to_goal",
            ])

    # ------------------------------------------------------------------ #
    @staticmethod
    def _iso(t):
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))

    @staticmethod
    def _ros_t():
        # use_sim_time=true iken sim saatini verir → rosbag/candump ile hizalanır.
        # Node init olmadan / clock yokken patlamasın diye korumalı.
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
            sys.stderr.write(f"[hedef_logger] log_event hata: {e}\n")

    def pose_due(self):
        """Throttle hint: bir sonraki pose örneği yazılmaya hazır mı?
        Lock'suz okuma (kasıtlı) — yalnızca pahalı mesafe hesaplarını gereksiz
        yapmamak için kapı; gerçek throttle kararı log_pose içinde lock altında
        tekrar verilir, yani yarış olsa bile en fazla bir örnek kayar/atlanır."""
        return self.ok and (time.time() - self._last_pose_t) >= self._pose_min_dt

    def log_pose(self, x, y, yaw, task_idx, wp_idx, n_path,
                 dist_to_wp=None, dist_to_goal=None):
        """Konum izini pose.csv'ye yaz — pose_hz'e göre kısılır."""
        if not self.ok or self._pose_writer is None:
            return
        try:
            t = time.time()
            yaw_deg = math.degrees(yaw) if yaw is not None else None
            with self._lock:
                # Throttle kararı + _last_pose_t güncellemesi lock altında →
                # oku-kontrol-yaz atomik (recalc içinden tekrar girişte çift yazım yok)
                if (t - self._last_pose_t) < self._pose_min_dt:
                    return
                self._last_pose_t = t
                self._pose_writer.writerow([
                    round(t, 3), self._iso(t), self._ros_t(),
                    self._r(x), self._r(y),
                    self._r(yaw, 4), self._r(yaw_deg, 2),
                    task_idx, wp_idx, n_path,
                    self._r(dist_to_wp), self._r(dist_to_goal),
                ])
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"[hedef_logger] log_pose hata: {e}\n")

    def log_recalc(self, reason, rx, ry, yaw, front, start_node, goal_node,
                   task_idx, task_name, path, path_changed=None):
        """
        Rota yeniden hesaplama olayını kaydet — tanının kalbi.
        dist_robot_start: rotanın başladığı düğümün araca uzaklığı (büyükse
        "uzaktan çizim"). dist_start_goal: start→goal düz mesafe; path_len_m ile
        kıyaslanınca kısa/dolambaçlı rota anlaşılır. path_changed: bir önceki
        rotadan farklı mı (False ise boşa recalc / oscillation işareti).
        Not: start_node her zaman path[0]'dır; ayrı path_start alanı tutulmaz.
        """
        try:
            d_robot_start = self._d((rx, ry), start_node)
            d_front_start = self._d(front, start_node) if front else None
            d_start_goal = self._d(start_node, goal_node)
            n_wp = len(path) if path else 0
            path_len = self._path_len(path) if path else 0.0
            self.log_event(
                "recalc",
                reason=reason,
                robot=[self._r(rx), self._r(ry)],
                yaw_deg=self._r(math.degrees(yaw), 2) if yaw is not None else None,
                front=[self._r(front[0]), self._r(front[1])] if front else None,
                start_node=[self._r(start_node[0]), self._r(start_node[1])],
                dist_robot_start=self._r(d_robot_start),
                dist_front_start=self._r(d_front_start),
                goal_node=[self._r(goal_node[0]), self._r(goal_node[1])],
                dist_start_goal=self._r(d_start_goal),
                task_idx=task_idx,
                task_name=task_name,
                path_found=bool(path),
                path_changed=path_changed,
                n_wp=n_wp,
                path_len_m=self._r(path_len),
            )
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"[hedef_logger] log_recalc hata: {e}\n")

    # ------------------------------------------------------------------ #
    @staticmethod
    def _jsonable(o):
        try:
            return list(o)
        except Exception:
            return str(o)

    @staticmethod
    def _r(v, nd=2):
        return round(v, nd) if isinstance(v, (int, float)) else None

    @staticmethod
    def _d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _path_len(path):
        return sum(
            math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            for i in range(1, len(path))
        )

    def close(self):
        try:
            if self._events_file:
                self._events_file.close()
            if self._pose_file:
                self._pose_file.close()
        except Exception:
            pass
