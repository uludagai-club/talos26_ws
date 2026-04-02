#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TALOS Waypoint Editoru
Gazebo pist haritasi uzerinde interaktif waypoint olusturma araci.

Kullanim:
    python3 waypoint_editor.py
    python3 waypoint_editor.py --track data/track_layout.jpg
    python3 waypoint_editor.py --load waypoints.json
    python3 waypoint_editor.py --track data/track_layout.jpg --lidar-yaml ../maps/my_map.yaml
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from PIL import Image
import json
import os
import argparse
import copy
from datetime import datetime


# =============================================================================
# VARSAYILAN YAPILANDIRMA
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_TRACK_IMAGE = os.path.join(SCRIPT_DIR, 'data', 'track_layout.jpg')
DEFAULT_CALIBRATION_FILE = os.path.join(SCRIPT_DIR, 'track_calibration.json')
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')

# Varsayilan kalibrasyon - pist goruntusunun Gazebo koordinatlarindaki sinirlari
# Senaryo2 modeli DAE mesh bariyerlerinden hesaplandi
DEFAULT_CALIBRATION = {
    "x_min": -25.34,
    "x_max": 40.59,
    "y_min": -37.83,
    "y_max": 15.87
}

# Bilinen referans noktalari (Gazebo koordinatlari)
VEHICLE_SPAWN = (-14.935761, -34.031181)

REFERENCE_WAYPOINTS = [
    (-4.7047, -34.308881),
    (-1.8232, -31.086682),
    (8.8342, -34.313881),
    (11.225352, -16.357474),
    (11.225352, -7.227474),
    (15.524211, -4.3727474),
    (22.027806, -3.2479100),
    (23.522607, -17.535281),
]


# =============================================================================
# WAYPOINT
# =============================================================================

class Waypoint:
    def __init__(self, x, y, name="", stop=False, speed=None):
        self.x = x
        self.y = y
        self.name = name
        self.stop = stop
        self.speed = speed

    def to_dict(self):
        d = {"x": round(self.x, 6), "y": round(self.y, 6)}
        if self.name:
            d["name"] = self.name
        if self.stop:
            d["stop"] = True
        if self.speed is not None:
            d["speed"] = self.speed
        return d

    @staticmethod
    def from_dict(d):
        return Waypoint(
            d["x"], d["y"],
            name=d.get("name", ""),
            stop=d.get("stop", False),
            speed=d.get("speed", None)
        )


# =============================================================================
# EDITOR
# =============================================================================

class WaypointEditor:

    def __init__(self, track_image_path=None, lidar_yaml_path=None,
                 load_file=None, output_dir=None):
        self.waypoints = []
        self.undo_stack = []
        self.redo_stack = []
        self.selected_idx = None
        self.dragging = False
        self.drag_idx = None
        self.mode = 'add'
        self.output_dir = output_dir or DEFAULT_OUTPUT_DIR

        # Kalibrasyon
        self.calibrating = False
        self.calib = dict(DEFAULT_CALIBRATION)
        self._load_calibration()

        # Goruntuler
        self.track_image = None
        self.lidar_image = None
        self.show_lidar = False
        self.show_track = True
        self.show_grid_lines = True
        self.show_refs = True

        # Lidar harita bilgileri
        self.lidar_resolution = 0.05
        self.lidar_origin = (-50.0, -50.0)
        self.lidar_size = (0, 0)

        self._load_track_image(track_image_path or DEFAULT_TRACK_IMAGE)
        if lidar_yaml_path:
            self._load_lidar_map(lidar_yaml_path)

        if load_file:
            self._load_waypoints(load_file)

        self._setup_ui()

    # ----- Goruntu Yukleme -----

    def _load_track_image(self, path):
        if os.path.exists(path):
            img = Image.open(path).convert('RGBA')
            self.track_image = np.array(img)
            print(f"Pist goruntusu yuklendi: {img.size[0]}x{img.size[1]} ({path})")
        else:
            print(f"Pist goruntusu bulunamadi: {path}")

    def _load_lidar_map(self, yaml_path):
        if not os.path.exists(yaml_path):
            print(f"Lidar haritasi bulunamadi: {yaml_path}")
            return
        import yaml
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self.lidar_resolution = data.get('resolution', 0.05)
        origin = data.get('origin', [-50.0, -50.0, 0.0])
        self.lidar_origin = (origin[0], origin[1])
        img_file = data.get('image', '')
        if not os.path.isabs(img_file):
            img_file = os.path.join(os.path.dirname(yaml_path), img_file)
        if os.path.exists(img_file):
            img = Image.open(img_file)
            self.lidar_image = np.array(img)
            self.lidar_size = (img.width, img.height)
            print(f"Lidar haritasi yuklendi: {img.width}x{img.height}")

    # ----- Kalibrasyon -----

    def _load_calibration(self):
        if os.path.exists(DEFAULT_CALIBRATION_FILE):
            with open(DEFAULT_CALIBRATION_FILE, 'r') as f:
                saved = json.load(f)
            self.calib.update(saved)
            print(f"Kalibrasyon yuklendi: {DEFAULT_CALIBRATION_FILE}")

    def _save_calibration(self):
        with open(DEFAULT_CALIBRATION_FILE, 'w') as f:
            json.dump(self.calib, f, indent=2)
        print(f"Kalibrasyon kaydedildi: X[{self.calib['x_min']:.1f}, {self.calib['x_max']:.1f}] "
              f"Y[{self.calib['y_min']:.1f}, {self.calib['y_max']:.1f}]")

    def _track_extent(self):
        return [self.calib['x_min'], self.calib['x_max'],
                self.calib['y_min'], self.calib['y_max']]

    # ----- Arayuz -----

    def _setup_ui(self):
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.canvas.manager.set_window_title('TALOS Waypoint Editoru')
        self.fig.patch.set_facecolor('#2b2b2b')

        self.ax = self.fig.add_axes([0.05, 0.12, 0.68, 0.83])
        self.ax.set_facecolor('#1a1a1a')

        self.info_ax = self.fig.add_axes([0.76, 0.12, 0.22, 0.83])
        self.info_ax.axis('off')
        self.info_ax.set_facecolor('#2b2b2b')

        self._draw_backgrounds()

        self.ax.set_xlabel('X (metre)', color='white')
        self.ax.set_ylabel('Y (metre)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.set_aspect('equal')

        margin = 5
        self.ax.set_xlim(self.calib['x_min'] - margin, self.calib['x_max'] + margin)
        self.ax.set_ylim(self.calib['y_min'] - margin, self.calib['y_max'] + margin)

        btn_specs = [
            (0.05, 'EKLE [1]', 'lightgreen', lambda e: self._set_mode('add')),
            (0.14, 'SEC [2]', 'lightyellow', lambda e: self._set_mode('select')),
            (0.23, 'SIL [3]', 'lightcoral', lambda e: self._set_mode('delete')),
            (0.33, 'Geri Al', 'lightgray', lambda e: self._undo()),
            (0.42, 'Ileri Al', 'lightgray', lambda e: self._redo()),
            (0.52, 'Temizle', '#ffddaa', lambda e: self._on_clear(e)),
            (0.62, 'Kaydet', 'lightblue', lambda e: self._on_save(e)),
            (0.72, 'Kalibr [C]', '#ddbbff', lambda e: self._toggle_calibration()),
            (0.82, 'Ref [R]', '#bbddff', lambda e: self._toggle_refs()),
        ]
        self.buttons = []
        for x, label, color, callback in btn_specs:
            ax_btn = self.fig.add_axes([x, 0.03, 0.08, 0.04])
            btn = Button(ax_btn, label, color=color)
            btn.on_clicked(callback)
            self.buttons.append(btn)

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)

        self._update_title()
        self._redraw()

    def _draw_backgrounds(self):
        to_remove = [a for a in self.ax.get_children() if hasattr(a, '_background')]
        for a in to_remove:
            a.remove()

        if self.show_lidar and self.lidar_image is not None:
            ox, oy = self.lidar_origin
            w, h = self.lidar_size
            res = self.lidar_resolution
            extent = [ox, ox + w * res, oy, oy + h * res]
            im = self.ax.imshow(self.lidar_image, cmap='gray', origin='lower',
                               extent=extent, alpha=0.3, zorder=0)
            im._background = True

        if self.show_track and self.track_image is not None:
            im = self.ax.imshow(self.track_image, origin='upper',
                               extent=self._track_extent(), alpha=0.85,
                               zorder=1, interpolation='bilinear')
            im._background = True

    def _update_title(self):
        mode_names = {'add': 'EKLEME', 'select': 'SEC/TASI', 'delete': 'SILME'}
        mode_str = mode_names.get(self.mode, self.mode)
        if self.calibrating:
            self.ax.set_title(
                'KALIBRASYON MODU | Oklar: kaydir | +/-: olcekle | Ctrl+S: kaydet',
                color='yellow', fontsize=10, fontweight='bold')
        else:
            self.ax.set_title(
                f'Mod: {mode_str} | Scroll: zoom | C: kalibrasyon',
                color='white', fontsize=10)

    # ----- Mod -----

    def _set_mode(self, mode):
        self.mode = mode
        self.calibrating = False
        self._update_title()
        self.fig.canvas.draw_idle()

    def _toggle_calibration(self):
        self.calibrating = not self.calibrating
        self._update_title()
        self.fig.canvas.draw_idle()

    def _toggle_refs(self):
        self.show_refs = not self.show_refs
        self._redraw()

    # ----- Undo/Redo -----

    def _save_state(self):
        self.undo_stack.append([copy.deepcopy(wp) for wp in self.waypoints])
        self.redo_stack.clear()
        if len(self.undo_stack) > 50:
            self.undo_stack.pop(0)

    def _undo(self):
        if not self.undo_stack:
            return
        self.redo_stack.append([copy.deepcopy(wp) for wp in self.waypoints])
        self.waypoints = self.undo_stack.pop()
        self.selected_idx = None
        self._redraw()

    def _redo(self):
        if not self.redo_stack:
            return
        self.undo_stack.append([copy.deepcopy(wp) for wp in self.waypoints])
        self.waypoints = self.redo_stack.pop()
        self.selected_idx = None
        self._redraw()

    # ----- Mouse -----

    def _find_nearest(self, x, y):
        if not self.waypoints:
            return None
        xlim = self.ax.get_xlim()
        threshold = (xlim[1] - xlim[0]) * 0.025
        min_dist = float('inf')
        min_idx = None
        for i, wp in enumerate(self.waypoints):
            dist = np.sqrt((wp.x - x) ** 2 + (wp.y - y) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx if min_dist <= max(threshold, 1.5) else None

    def _on_click(self, event):
        if event.inaxes != self.ax or self.calibrating:
            return
        x, y = event.xdata, event.ydata

        if event.button == 1:
            if self.mode == 'add':
                self._save_state()
                wp = Waypoint(x, y)
                if self.selected_idx is not None and self.selected_idx < len(self.waypoints):
                    self.waypoints.insert(self.selected_idx + 1, wp)
                    self.selected_idx += 1
                else:
                    self.waypoints.append(wp)
                    self.selected_idx = len(self.waypoints) - 1
                self._redraw()
            elif self.mode == 'select':
                idx = self._find_nearest(x, y)
                if idx is not None:
                    self.selected_idx = idx
                    self.dragging = True
                    self.drag_idx = idx
                    self._save_state()
                else:
                    self.selected_idx = None
                self._redraw()
            elif self.mode == 'delete':
                idx = self._find_nearest(x, y)
                if idx is not None:
                    self._save_state()
                    self.waypoints.pop(idx)
                    self.selected_idx = None
                    self._redraw()

        elif event.button == 3:
            idx = self._find_nearest(x, y)
            if idx is not None:
                self._save_state()
                self.waypoints.pop(idx)
                self.selected_idx = None
                self._redraw()

        elif event.button == 2:
            idx = self._find_nearest(x, y)
            if idx is not None:
                self.selected_idx = idx
                self.dragging = True
                self.drag_idx = idx
                self._save_state()
                self._redraw()

    def _on_release(self, event):
        self.dragging = False
        self.drag_idx = None

    def _on_motion(self, event):
        if not self.dragging or self.drag_idx is None or event.inaxes != self.ax:
            return
        self.waypoints[self.drag_idx].x = event.xdata
        self.waypoints[self.drag_idx].y = event.ydata
        self._redraw()

    def _on_scroll(self, event):
        if event.inaxes != self.ax:
            return
        factor = 0.8 if event.button == 'up' else 1.25
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        xc, yc = event.xdata, event.ydata
        self.ax.set_xlim(xc - (xc - xlim[0]) * factor, xc + (xlim[1] - xc) * factor)
        self.ax.set_ylim(yc - (yc - ylim[0]) * factor, yc + (ylim[1] - yc) * factor)
        self.fig.canvas.draw_idle()

    # ----- Keyboard -----

    def _on_key(self, event):
        if event.key == 'ctrl+z':
            self._undo()
        elif event.key == 'ctrl+y':
            self._redo()
        elif event.key == 'ctrl+s':
            if self.calibrating:
                self._save_calibration()
            else:
                self._on_save(None)
        elif event.key in ('delete', 'backspace'):
            if self.selected_idx is not None and self.selected_idx < len(self.waypoints):
                self._save_state()
                self.waypoints.pop(self.selected_idx)
                self.selected_idx = None
                self._redraw()
        elif event.key == '1':
            self._set_mode('add')
        elif event.key == '2':
            self._set_mode('select')
        elif event.key == '3':
            self._set_mode('delete')
        elif event.key == 'c':
            self._toggle_calibration()
        elif event.key == 'r':
            self._toggle_refs()
        elif event.key == 'l':
            self.show_lidar = not self.show_lidar
            self._draw_backgrounds()
            self._redraw()
        elif event.key == 't':
            self.show_track = not self.show_track
            self._draw_backgrounds()
            self._redraw()
        elif event.key == 'g':
            self.show_grid_lines = not self.show_grid_lines
            self.ax.grid(self.show_grid_lines, alpha=0.3, color='gray')
            self.fig.canvas.draw_idle()
        elif event.key == 's' and not self.calibrating:
            if self.selected_idx is not None:
                self.waypoints[self.selected_idx].stop = not self.waypoints[self.selected_idx].stop
                self._redraw()
        elif self.calibrating:
            self._handle_calibration_key(event.key)

    def _handle_calibration_key(self, key):
        x_span = self.calib['x_max'] - self.calib['x_min']
        y_span = self.calib['y_max'] - self.calib['y_min']
        step = 0.5

        if key == 'right':
            self.calib['x_min'] += step
            self.calib['x_max'] += step
        elif key == 'left':
            self.calib['x_min'] -= step
            self.calib['x_max'] -= step
        elif key == 'up':
            self.calib['y_min'] += step
            self.calib['y_max'] += step
        elif key == 'down':
            self.calib['y_min'] -= step
            self.calib['y_max'] -= step
        elif key in ('+', '='):
            self.calib['x_min'] -= step
            self.calib['x_max'] += step
            self.calib['y_min'] -= step * (y_span / x_span)
            self.calib['y_max'] += step * (y_span / x_span)
        elif key in ('-', '_'):
            self.calib['x_min'] += step
            self.calib['x_max'] -= step
            self.calib['y_min'] += step * (y_span / x_span)
            self.calib['y_max'] -= step * (y_span / x_span)
        elif key == 'shift+right':
            self.calib['x_max'] += step
        elif key == 'shift+left':
            self.calib['x_max'] -= step
        elif key == 'shift+up':
            self.calib['y_max'] += step
        elif key == 'shift+down':
            self.calib['y_max'] -= step
        else:
            return

        self._draw_backgrounds()
        self._redraw()
        print(f"  Kalibrasyon: X[{self.calib['x_min']:.1f}, {self.calib['x_max']:.1f}] "
              f"Y[{self.calib['y_min']:.1f}, {self.calib['y_max']:.1f}] "
              f"({x_span:.1f}m x {y_span:.1f}m)")

    # ----- Cizim -----

    def _redraw(self):
        to_remove = [a for a in self.ax.get_children() if hasattr(a, '_wp_marker')]
        for a in to_remove:
            a.remove()

        self.ax.grid(self.show_grid_lines, alpha=0.2, color='gray')

        if self.show_refs:
            self._draw_references()
        if self.waypoints:
            self._draw_waypoints()

        self._update_info()
        self.fig.canvas.draw_idle()

    def _draw_references(self):
        sx, sy = VEHICLE_SPAWN
        m = self.ax.plot(sx, sy, 's', color='cyan', markersize=10,
                        markeredgecolor='white', markeredgewidth=1.5, zorder=4)
        m[0]._wp_marker = True
        t = self.ax.annotate('Arac Baslangic', (sx, sy),
                            textcoords="offset points", xytext=(10, -5),
                            fontsize=7, color='cyan', fontweight='bold', zorder=7)
        t._wp_marker = True

        for i, (ex, ey) in enumerate(REFERENCE_WAYPOINTS):
            m = self.ax.plot(ex, ey, 'x', color='gray', markersize=6,
                            markeredgewidth=1.5, alpha=0.5, zorder=3)
            m[0]._wp_marker = True
            t = self.ax.annotate(str(i + 1), (ex, ey),
                                textcoords="offset points", xytext=(5, 5),
                                fontsize=6, color='gray', alpha=0.5, zorder=3)
            t._wp_marker = True

    def _draw_waypoints(self):
        xs = [wp.x for wp in self.waypoints]
        ys = [wp.y for wp in self.waypoints]

        if len(self.waypoints) > 1:
            line, = self.ax.plot(xs, ys, '-', color='#00aaff', linewidth=2, alpha=0.6, zorder=8)
            line._wp_marker = True
            for i in range(len(self.waypoints) - 1):
                dx = xs[i + 1] - xs[i]
                dy = ys[i + 1] - ys[i]
                dist = np.sqrt(dx**2 + dy**2)
                if dist > 0.5:
                    mx = (xs[i] + xs[i + 1]) / 2
                    my = (ys[i] + ys[i + 1]) / 2
                    arr = self.ax.annotate('',
                        xy=(mx + dx * 0.08, my + dy * 0.08),
                        xytext=(mx - dx * 0.08, my - dy * 0.08),
                        arrowprops=dict(arrowstyle='->', color='#00aaff', lw=2, alpha=0.8),
                        zorder=9)
                    arr._wp_marker = True

        for i, wp in enumerate(self.waypoints):
            if i == self.selected_idx:
                color, size, edge, ew = '#00ff00', 140, 'yellow', 3
            elif i == 0:
                color, size, edge, ew = '#00cc00', 110, 'white', 2
            elif i == len(self.waypoints) - 1:
                color, size, edge, ew = '#ff3333', 110, 'white', 2
            elif wp.stop:
                color, size, edge, ew = '#ff8800', 100, 'white', 2
            else:
                color, size, edge, ew = '#0088ff', 80, 'white', 1.5

            sc = self.ax.scatter(wp.x, wp.y, c=color, s=size,
                               edgecolors=edge, linewidths=ew, zorder=10, marker='o')
            sc._wp_marker = True

            t = self.ax.annotate(str(i + 1), (wp.x, wp.y),
                                textcoords="offset points", xytext=(8, 8),
                                fontsize=9, fontweight='bold', color='white',
                                bbox=dict(boxstyle='round,pad=0.2',
                                         facecolor='black', alpha=0.8),
                                zorder=11)
            t._wp_marker = True

            if wp.stop:
                t2 = self.ax.annotate('DUR', (wp.x, wp.y),
                                     textcoords="offset points", xytext=(8, -14),
                                     fontsize=7, color='#ff4444', fontweight='bold', zorder=11)
                t2._wp_marker = True

    def _update_info(self):
        self.info_ax.clear()
        self.info_ax.axis('off')

        total_dist = sum(
            np.sqrt((self.waypoints[i].x - self.waypoints[i-1].x)**2 +
                    (self.waypoints[i].y - self.waypoints[i-1].y)**2)
            for i in range(1, len(self.waypoints))
        )

        lines = [
            "WAYPOINT LiSTESi",
            "=" * 24,
            f"Toplam: {len(self.waypoints)}",
            f"Mesafe: {total_dist:.1f} m",
            "-" * 24,
        ]
        for i, wp in enumerate(self.waypoints):
            sel = ">" if i == self.selected_idx else " "
            stop = " [D]" if wp.stop else ""
            lines.append(f"{sel}{i+1}: ({wp.x:.1f}, {wp.y:.1f}){stop}")

        lines.extend(["", "-" * 24, "",
                      "KISAYOLLAR:",
                      "1/2/3: Ekle/Sec/Sil",
                      "s: Durak isle",
                      "Del: Seciliyi sil",
                      "Ctrl+Z/Y: Geri/Ileri",
                      "Ctrl+S: Kaydet",
                      "Scroll: Zoom",
                      "",
                      "GORUNUM:",
                      "C: Kalibrasyon modu",
                      "R: Referans noktalari",
                      "T: Pist goruntusu",
                      "L: Lidar haritasi",
                      "G: Grid acma/kapama"])

        if self.calibrating:
            lines.extend(["", "--- KALIBRASYON ---",
                         "Oklar: Kaydir",
                         "+/-: Olcekle",
                         "Shift+Ok: Tek eksen",
                         "Ctrl+S: Kal. kaydet"])

        text = "\n".join(lines)
        self.info_ax.text(0.05, 0.98, text, transform=self.info_ax.transAxes,
                         fontsize=7.5, verticalalignment='top', fontfamily='monospace',
                         color='white',
                         bbox=dict(boxstyle='round', facecolor='#333333', alpha=0.9))

    # ----- Kaydet / Yukle -----

    def _on_clear(self, event):
        if self.waypoints:
            self._save_state()
            self.waypoints.clear()
            self.selected_idx = None
            self._redraw()

    def _on_save(self, event):
        if not self.waypoints:
            print("Kaydedilecek waypoint yok!")
            return
        self._export_all()

    def _load_waypoints(self, path):
        if not os.path.exists(path):
            print(f"Dosya bulunamadi: {path}")
            return
        with open(path, 'r') as f:
            data = json.load(f)
        self.waypoints.clear()
        if isinstance(data, dict) and "waypoints" in data:
            for d in data["waypoints"]:
                self.waypoints.append(Waypoint.from_dict(d))
        elif isinstance(data, list):
            for item in data:
                if isinstance(item, dict):
                    self.waypoints.append(Waypoint.from_dict(item))
                elif isinstance(item, list) and len(item) >= 2:
                    self.waypoints.append(Waypoint(item[0], item[1]))
        print(f"{len(self.waypoints)} waypoint yuklendi: {path}")

    def _export_all(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        total_dist = sum(
            np.sqrt((self.waypoints[i].x - self.waypoints[i-1].x)**2 +
                    (self.waypoints[i].y - self.waypoints[i-1].y)**2)
            for i in range(1, len(self.waypoints))
        )

        # JSON
        json_path = os.path.join(self.output_dir, 'waypoints.json')
        json_data = {
            "metadata": {
                "created": timestamp,
                "count": len(self.waypoints),
                "total_distance_m": round(total_dist, 1),
                "coordinate_system": "gazebo_xy_meters"
            },
            "waypoints": [wp.to_dict() for wp in self.waypoints]
        }
        with open(json_path, 'w') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)

        # Python
        py_path = os.path.join(self.output_dir, 'waypoints_export.py')
        with open(py_path, 'w') as f:
            f.write("#!/usr/bin/env python3\n")
            f.write(f"# Otomatik olusturuldu: {timestamp}\n")
            f.write(f"# Toplam mesafe: {total_dist:.1f} m\n\n")
            f.write("DEFAULT_WAYPOINTS = [\n")
            for wp in self.waypoints:
                comment = "  # DURAK" if wp.stop else ""
                f.write(f"    ({wp.x:.6f}, {wp.y:.6f}),{comment}\n")
            f.write("]\n\n")
            stops = [i for i, wp in enumerate(self.waypoints) if wp.stop]
            if stops:
                f.write(f"STOP_INDICES = {stops}\n\n")
            wp_str = " ".join(f"{wp.x:.4f},{wp.y:.4f}" for wp in self.waypoints)
            f.write(f'# --waypoints "{wp_str}"\n')

        # CSV
        csv_path = os.path.join(self.output_dir, 'waypoints.csv')
        with open(csv_path, 'w') as f:
            f.write("index,x,y,name,stop,speed\n")
            for i, wp in enumerate(self.waypoints):
                f.write(f"{i+1},{wp.x:.6f},{wp.y:.6f},{wp.name},{wp.stop},{wp.speed or ''}\n")

        # Yedek
        backup_dir = os.path.join(self.output_dir, 'backups')
        os.makedirs(backup_dir, exist_ok=True)
        with open(os.path.join(backup_dir, f'waypoints_{timestamp}.json'), 'w') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)

        wp_str = " ".join(f"{wp.x:.4f},{wp.y:.4f}" for wp in self.waypoints)
        print(f"\n{'='*50}")
        print(f"  {len(self.waypoints)} waypoint kaydedildi ({total_dist:.1f}m)")
        print(f"  JSON  : {json_path}")
        print(f"  Python: {py_path}")
        print(f"  CSV   : {csv_path}")
        print(f"\n  --waypoints \"{wp_str}\"")
        print(f"{'='*50}")

    def run(self):
        print("=" * 50)
        print("  TALOS Waypoint Editoru")
        print("=" * 50)
        print("  Sol tik: ekle | Sag tik: sil | Orta tik: tasi")
        print("  1/2/3: Ekle/Sec/Sil | C: Kalibrasyon | R: Ref")
        print("  Ctrl+S: Kaydet | Ctrl+Z/Y: Geri/Ileri al")
        print("=" * 50)
        plt.show()


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='TALOS Waypoint Editoru - Gazebo pist haritasi uzerinde interaktif waypoint olusturma',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ornekler:
  python3 waypoint_editor.py
  python3 waypoint_editor.py --track data/track_layout.jpg
  python3 waypoint_editor.py --load output/waypoints.json
  python3 waypoint_editor.py --track data/track_layout.jpg --lidar-yaml ../maps/my_map.yaml
        """
    )
    parser.add_argument('--track', default=None,
                       help='Pist layout goruntusu yolu (varsayilan: data/track_layout.jpg)')
    parser.add_argument('--lidar-yaml', default=None,
                       help='Lidar haritasi YAML dosyasi (opsiyonel)')
    parser.add_argument('--load', default=None,
                       help='Yuklenecek waypoint JSON dosyasi')
    parser.add_argument('--output', default=None,
                       help='Cikti dizini (varsayilan: output/)')
    args = parser.parse_args()

    editor = WaypointEditor(
        track_image_path=args.track,
        lidar_yaml_path=args.lidar_yaml,
        load_file=args.load,
        output_dir=args.output
    )
    editor.run()


if __name__ == '__main__':
    main()
