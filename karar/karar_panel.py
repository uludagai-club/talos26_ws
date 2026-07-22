#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TALOS karar panosu — anlık kararı ve onu doğuran girdileri canlı gösterir.

Amaç: ekranda gördüğün başlık = kontrol birimine gönderdiğimiz string. Karar
node'u iki şey yayınlıyor, pano ikisini birlikte okur:

  * /karar (std_msgs/String) — kontrole giden HAM string ("normal"/"slow"/"dur"/
    "acildurus"/"sag"/"sol"). Pano başlığı BUNU basar (her tick, ~10 Hz → anlık).
  * /karar_bt/snapshot (std_msgs/String, JSON) — kararın gerekçesi + o kararı
    doğuran girdiler (engel d_arc/açı, yaya, levha, hız). ~2 Hz.

Salt-görsel bir izleme aracıdır; hiçbir topic'e yazmaz, karar davranışını
etkilemez. can_visualizer.py ile aynı renk sözlüğünü kullanır (tutarlılık).

Çalıştırma:
    python3 karar_panel.py            # canlı pencere
    KARAR_PANEL_HZ=5 python3 karar_panel.py
"""
from __future__ import annotations

import json
import os
import threading
import time

import rospy
from std_msgs.msg import String

import matplotlib
matplotlib.use(os.environ.get("MPLBACKEND", "TkAgg"))
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch


# can_visualizer.py ile bire-bir aynı sözlük (görsel tutarlılık).
KARAR_COLORS = {
    "normal":    {"bg": "#1e3d1e", "text": "#7fdb7f", "label": "NORMAL"},
    "slow":      {"bg": "#3d2e1e", "text": "#ffb74d", "label": "YAVAS"},
    "dur":       {"bg": "#3d1e1e", "text": "#ff6e6e", "label": "DUR"},
    "acildurus": {"bg": "#5d0000", "text": "#ff1744", "label": "ACIL DURUS"},
    "sag":       {"bg": "#1e2e3d", "text": "#64b5f6", "label": "SAG"},
    "sol":       {"bg": "#1e2e3d", "text": "#64b5f6", "label": "SOL"},
}
_UNKNOWN = {"bg": "#2a2a2a", "text": "#9e9e9e", "label": "?"}

STALE_S = 1.5   # /karar bu süre sessizse "BAYAT" uyarısı


class _Store:
    """Callback'lerin yazdığı, animasyonun okuduğu thread-safe durum."""

    def __init__(self):
        self.lock = threading.Lock()
        self.karar = "normal"          # /karar HAM string (kontrole giden)
        self.karar_t = 0.0             # son /karar geliş zamanı (bayatlık)
        self.reason = ""
        self.phase = ""
        self.wait_s = 0.0
        self.snap = {}                 # son snapshot bb sözlüğü
        self.snap_t = 0.0

    def on_karar(self, msg: String):
        with self.lock:
            self.karar = (msg.data or "").strip().lower()
            self.karar_t = time.time()

    def on_snapshot(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        bb = payload.get("bb", {}) or {}
        dec = bb.get("decision", {}) or {}
        with self.lock:
            self.snap = bb
            self.snap_t = time.time()
            self.reason = str(dec.get("reason", ""))
            self.phase = str(dec.get("phase", ""))
            try:
                self.wait_s = float(dec.get("wait_remaining_s", 0.0))
            except (ValueError, TypeError):
                self.wait_s = 0.0

    def read(self) -> dict:
        with self.lock:
            return {
                "karar": self.karar,
                "karar_age": (time.time() - self.karar_t) if self.karar_t else 1e9,
                "reason": self.reason,
                "phase": self.phase,
                "wait_s": self.wait_s,
                "snap": dict(self.snap),
                "snap_age": (time.time() - self.snap_t) if self.snap_t else 1e9,
            }


def _fmt_m(v) -> str:
    """Snapshot mesafeleri: -1.0 = yok/∞ (bb._fin), sonlu = metre."""
    try:
        v = float(v)
    except (ValueError, TypeError):
        return "—"
    return "—" if v < 0 else f"{v:.1f}m"


def _input_lines(snap: dict) -> list[tuple[str, str]]:
    """Kararı doğuran girdileri (etiket, değer) satırlarına indirger."""
    if not snap:
        return [("veri", "snapshot bekleniyor…")]

    eng = snap.get("engel", {}) or {}
    yaya = snap.get("yaya", {}) or {}
    lev = snap.get("levha", {}) or {}

    eng_present = eng.get("present", False)
    eng_val = (
        f"VAR  d_arc={_fmt_m(eng.get('d_arc'))}  "
        f"d_ctr={_fmt_m(eng.get('d_center'))}  açı={eng.get('angle_deg', 0.0):+.0f}°  "
        f"{eng.get('source', '?')} n{eng.get('count', 0)} m{eng.get('mem', 0)}"
        if eng_present else "yok"
    )

    yaya_val = (f"VAR  d={_fmt_m(yaya.get('d'))}" if yaya.get("present")
                else "yok")

    lev_isim = lev.get("isim", "NONE")
    lev_val = ("yok" if lev_isim in ("NONE", None)
               else f"{lev_isim}  d={_fmt_m(lev.get('d'))}")

    speed = snap.get("speed_kmh", 0.0)
    st = snap.get("state", {}) or {}

    return [
        ("engel", eng_val),
        ("yaya", yaya_val),
        ("levha", lev_val),
        ("hiz", f"{speed:.1f} km/h"),
        ("durum", f"emergency_latched={st.get('emergency_latched', False)}  "
                  f"stop_sign={st.get('stop_sign_phase', '-')}"),
    ]


def main():
    rospy.init_node("karar_panel", anonymous=True, disable_signals=True)
    store = _Store()
    rospy.Subscriber("/karar", String, store.on_karar, queue_size=10)
    rospy.Subscriber("/karar_bt/snapshot", String, store.on_snapshot, queue_size=2)
    rospy.loginfo("[karar_panel] /karar + /karar_bt/snapshot dinleniyor")

    plt.rcParams["toolbar"] = "None"
    fig = plt.figure(figsize=(7.6, 6.4), facecolor="#141414")
    fig.canvas.manager.set_window_title("TALOS — Karar Panosu")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    # Başlık kutusu (kontrole giden string)
    head_box = FancyBboxPatch(
        (0.06, 0.60), 0.88, 0.30,
        boxstyle="round,pad=0.02,rounding_size=0.03",
        linewidth=0, facecolor="#1e3d1e", transform=ax.transAxes,
    )
    ax.add_patch(head_box)

    ax.text(0.5, 0.925, "KONTROLE GİDEN KARAR   (/karar)",
            transform=ax.transAxes, ha="center", va="center",
            color="#8a8a8a", fontsize=9, family="monospace")
    txt_head = ax.text(0.5, 0.755, "NORMAL", transform=ax.transAxes,
                       ha="center", va="center", color="#7fdb7f",
                       fontsize=40, fontweight="bold", family="monospace")
    txt_raw = ax.text(0.5, 0.635, "'normal'", transform=ax.transAxes,
                      ha="center", va="center", color="#cccccc",
                      fontsize=13, family="monospace")

    # Gerekçe / faz satırı
    txt_reason = ax.text(0.06, 0.545, "", transform=ax.transAxes,
                         ha="left", va="center", color="#e0e0e0",
                         fontsize=11, family="monospace")

    ax.plot([0.06, 0.94], [0.50, 0.50], color="#333333", lw=1,
            transform=ax.transAxes)
    ax.text(0.06, 0.475, "KARARI DOĞURAN GİRDİLER", transform=ax.transAxes,
            ha="left", va="center", color="#8a8a8a", fontsize=9,
            family="monospace")

    # Girdi satırları (5 sabit satır; içerik animasyonda güncellenir)
    n_rows = 5
    row_labels = []
    row_values = []
    y0 = 0.42
    dy = 0.075
    for i in range(n_rows):
        y = y0 - i * dy
        lbl = ax.text(0.08, y, "", transform=ax.transAxes, ha="left",
                      va="center", color="#7aa7d0", fontsize=11,
                      family="monospace", fontweight="bold")
        val = ax.text(0.23, y, "", transform=ax.transAxes, ha="left",
                      va="center", color="#dddddd", fontsize=10,
                      family="monospace")
        row_labels.append(lbl)
        row_values.append(val)

    txt_foot = ax.text(0.5, 0.03, "", transform=ax.transAxes, ha="center",
                       va="center", color="#666666", fontsize=8,
                       family="monospace")

    def update(_frame):
        s = store.read()
        karar = s["karar"]
        cfg = KARAR_COLORS.get(karar, _UNKNOWN)
        stale = s["karar_age"] > STALE_S

        # Başlık — bayatsa gri + uyarı
        if stale:
            head_box.set_facecolor("#2a2a2a")
            txt_head.set_text("BAYAT")
            txt_head.set_color("#ffb74d")
            txt_raw.set_text(f"/karar {s['karar_age']:.1f}s sessiz")
            txt_raw.set_color("#ffb74d")
        else:
            head_box.set_facecolor(cfg["bg"])
            txt_head.set_text(cfg["label"])
            txt_head.set_color(cfg["text"])
            txt_raw.set_text(f"'{karar}'")
            txt_raw.set_color("#cccccc")

        reason = s["reason"] or "—"
        phase = s["phase"] or "—"
        wait = s["wait_s"]
        rline = f"sebep: {reason}   faz: {phase}"
        if wait > 0.05:
            rline += f"   bekleme: {wait:.1f}s"
        txt_reason.set_text(rline)

        rows = _input_lines(s["snap"])
        for i in range(n_rows):
            if i < len(rows):
                lbl, val = rows[i]
                row_labels[i].set_text(lbl)
                row_values[i].set_text(val)
            else:
                row_labels[i].set_text("")
                row_values[i].set_text("")

        snap_age = s["snap_age"]
        snap_note = ("snapshot yok" if snap_age > 1e8
                     else f"snapshot {snap_age:.1f}s önce")
        txt_foot.set_text(snap_note + "   ·   salt-görsel · karar'a yazmaz")

        return [head_box, txt_head, txt_raw, txt_reason, txt_foot,
                *row_labels, *row_values]

    hz = float(os.environ.get("KARAR_PANEL_HZ", "5"))
    interval_ms = max(50, int(1000.0 / max(1.0, hz)))
    _anim = animation.FuncAnimation(
        fig, update, interval=interval_ms, blit=False,
        cache_frame_data=False)

    # ROS shutdown → pencereyi kapat (disable_signals=True olduğu için Ctrl-C mpl'e düşer)
    def _on_close(_evt):
        rospy.signal_shutdown("panel kapatıldı")
    fig.canvas.mpl_connect("close_event", _on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("panel çıkış")


if __name__ == "__main__":
    main()
