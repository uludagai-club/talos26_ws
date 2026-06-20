#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import threading
from geometry_msgs.msg import Pose2D

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QPoint
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPolygon

# Thread-safe ROS veri alıcısı ve Qt sinyal köprüsü
class RosBridge(QObject):
    data_received = pyqtSignal(float, float, float) # x, y, yaw
    signal_lost = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.last_msg_time = 0.0
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(100) # 100ms'de bir kontrol et

    def start_subscriber(self):
        rospy.Subscriber('/konum', Pose2D, self.odom_callback)

    def odom_callback(self, msg):
        self.last_msg_time = rospy.get_time()
        
        # Pose2D doğrudan x, y ve theta (yaw) içerir
        x = msg.x
        y = msg.y
        yaw = msg.theta
        
        self.data_received.emit(x, y, yaw)

    def check_connection(self):
        # 1.0 saniyeden uzun süredir veri gelmediyse bağlantı kaybı sinyali gönder
        if rospy.get_time() - self.last_msg_time > 1.0:
            self.signal_lost.emit()


# Özel Pusula / Yön Göstergesi Widget'ı
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0 # Radyan cinsinden
        self.setMinimumSize(120, 120)

    def set_yaw(self, yaw):
        self.yaw = yaw
        self.update() # Yeniden çizimi tetikle

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        side = min(width, height)
        
        # Merkez noktası ve yarıçap
        cx = width / 2.0
        cy = height / 2.0
        r = (side / 2.0) - 10
        
        # Dış halkayı çiz
        pen = QPen(QColor(0, 229, 255, 100), 2)
        painter.setPen(pen)
        painter.drawEllipse(int(cx - r), int(cy - r), int(2*r), int(2*r))
        
        # Derece çizgilerini çiz
        pen_tick = QPen(QColor(0, 229, 255, 150), 1)
        painter.setPen(pen_tick)
        for i in range(12):
            angle = i * 30
            rad = math.radians(angle)
            x1 = cx + (r - 5) * math.cos(rad)
            y1 = cy + (r - 5) * math.sin(rad)
            x2 = cx + r * math.cos(rad)
            y2 = cy + r * math.sin(rad)
            painter.drawLine(QPoint(int(x1), int(y1)), QPoint(int(x2), int(y2)))

        # Yön harflerini çiz (N, E, S, W)
        painter.setFont(QFont("Ubuntu", 8, QFont.Bold))
        painter.setPen(QColor(255, 255, 255, 200))
        
        # Kuzey (North) - Yukarı
        painter.drawText(int(cx - 5), int(cy - r + 15), "N")
        # Doğu (East) - Sağ
        painter.drawText(int(cx + r - 15), int(cy + 4), "E")
        # Güney (South) - Aşağı
        painter.drawText(int(cx - 5), int(cy + r - 5), "S")
        # Batı (West) - Sol
        painter.drawText(int(cx - r + 5), int(cy + 4), "W")

        # Yaw yönündeki yön oku (aracın gittiği yönü gösterir)
        painter.translate(cx, cy)
        # Gazebo'da 0 derece Doğu'dur, Saat yönünün tersidir. Pusulayı buna göre döndürelim
        # Qt'de derece saat yönündedir, bu yüzden -yaw kullanılır ve 90 derece çıkarılır (Kuzeyi yukarı yapmak için)
        yaw_deg = math.degrees(self.yaw)
        painter.rotate(-yaw_deg - 90)
        
        # Ok çizimi
        arrow = QPolygon([
            QPoint(0, -int(r - 8)),
            QPoint(-8, -int(r - 25)),
            QPoint(0, -int(r - 20)),
            QPoint(8, -int(r - 25))
        ])
        
        painter.setBrush(QColor(0, 229, 255, 220))
        painter.setPen(QPen(QColor(0, 229, 255), 1.5))
        painter.drawPolygon(arrow)


# HUD Ana Penceresi
class HudWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        # Pencere Özellikleri: Çerçevesiz, Saydam ve Her Zaman Üstte
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.SubWindow)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        
        self.init_ui()
        self.resize(320, 210) # Hız kaldırıldığı için dikey boyut küçültüldü
        self.center_to_top_right()
        
        # Blinking LED state
        self.led_state = True
        self.is_connected = False
        
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.blink_led)
        self.blink_timer.start(500)

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Ana panel (Glassmorphic stil)
        self.panel = QWidget(self)
        self.panel.setObjectName("MainPanel")
        self.panel.setStyleSheet("""
            QWidget#MainPanel {
                background-color: rgba(10, 17, 30, 0.85);
                border: 2px solid rgba(0, 229, 255, 0.4);
                border-radius: 12px;
            }
        """)
        
        panel_layout = QVBoxLayout(self.panel)
        panel_layout.setContentsMargins(10, 10, 10, 10)
        
        # Başlık ve LED Durum göstergesi
        header_layout = QHBoxLayout()
        self.title_label = QLabel("TALOS NAVIGATION HUD")
        self.title_label.setFont(QFont("Ubuntu", 10, QFont.Bold))
        self.title_label.setStyleSheet("color: #00e5ff;")
        
        self.led_label = QLabel()
        self.led_label.setFixedSize(12, 12)
        self.set_led_color("red")
        
        header_layout.addWidget(self.title_label)
        header_layout.addStretch()
        header_layout.addWidget(self.led_label)
        panel_layout.addLayout(header_layout)
        
        # Orta Kısım: Pusula ve Veriler yan yana
        middle_layout = QHBoxLayout()
        
        # Pusula Widget'ı
        self.compass = CompassWidget()
        middle_layout.addWidget(self.compass)
        
        # Konum Verileri Layout'u
        data_layout = QVBoxLayout()
        data_layout.setSpacing(8)
        
        self.x_label = QLabel("X: --.--- m")
        self.y_label = QLabel("Y: --.--- m")
        self.yaw_label = QLabel("YAW: ---.-°")
        
        # Yazı stilleri
        for label in [self.x_label, self.y_label, self.yaw_label]:
            label.setFont(QFont("Ubuntu Mono", 11, QFont.Bold))
            label.setStyleSheet("color: #ffffff;")
        
        data_layout.addWidget(self.x_label)
        data_layout.addWidget(self.y_label)
        data_layout.addWidget(self.yaw_label)
        data_layout.addStretch()
        
        middle_layout.addLayout(data_layout)
        panel_layout.addLayout(middle_layout)
        
        # Alt Bilgi Satırı
        footer = QLabel("topic: /konum | rate: 50 Hz")
        footer.setFont(QFont("Ubuntu", 8))
        footer.setStyleSheet("color: rgba(255, 255, 255, 0.4);")
        panel_layout.addWidget(footer)
        
        layout.addWidget(self.panel)
        self.setLayout(layout)

    def center_to_top_right(self):
        # Ekranın sağ üst köşesine konumlandır (20px boşluk bırakarak)
        screen = QApplication.primaryScreen().geometry()
        x = screen.width() - self.width() - 20
        y = 40 # Gazebo üst menü barının altına denk gelmesi için
        self.move(x, y)

    def set_led_color(self, color):
        if color == "green":
            self.led_label.setStyleSheet("background-color: #39ff14; border-radius: 6px; border: 1px solid #ffffff;")
        elif color == "red":
            self.led_label.setStyleSheet("background-color: #ff073a; border-radius: 6px; border: 1px solid #ffffff;")
        elif color == "orange":
            self.led_label.setStyleSheet("background-color: #ff9d00; border-radius: 6px; border: 1px solid #ffffff;")

    def update_data(self, x, y, yaw):
        self.is_connected = True
        self.x_label.setText(f"X: {x:8.3f} m")
        self.y_label.setText(f"Y: {y:8.3f} m")
        
        # Derece cinsinden yaw açısını 0-360 arasına normalleştir
        yaw_deg = math.degrees(yaw) % 360
        self.yaw_label.setText(f"YAW: {yaw_deg:5.1f}°")
        
        # Pusulayı güncelle
        self.compass.set_yaw(yaw)
        self.panel.setStyleSheet("QWidget#MainPanel { background-color: rgba(10, 17, 30, 0.85); border: 2px solid rgba(0, 229, 255, 0.7); border-radius: 12px; }")

    def handle_signal_loss(self):
        self.is_connected = False
        self.x_label.setText("X: --.--- m")
        self.y_label.setText("Y: --.--- m")
        self.yaw_label.setText("YAW: ---.-°")
        self.panel.setStyleSheet("QWidget#MainPanel { background-color: rgba(30, 10, 10, 0.85); border: 2px solid rgba(255, 7, 58, 0.7); border-radius: 12px; }")

    def blink_led(self):
        self.led_state = not self.led_state
        if self.is_connected:
            self.set_led_color("green" if self.led_state else "orange")
        else:
            self.set_led_color("red" if self.led_state else "orange")


def main():
    # ROS Düğümünü başlat
    rospy.init_node('hud_overlay_node', anonymous=True)
    
    app = QApplication(sys.argv)
    hud = HudWindow()
    hud.show()
    
    # Qt & ROS Köprüsünü kur
    bridge = RosBridge()
    bridge.data_received.connect(hud.update_data)
    bridge.signal_lost.connect(hud.handle_signal_loss)
    
    # ROS dinleyicisini bir arka plan thread'inde çalıştır
    ros_thread = threading.Thread(target=bridge.start_subscriber)
    ros_thread.daemon = True
    ros_thread.start()
    
    # PyQt5 döngüsünü ROS kilitlenmelerini önleyecek şekilde spin et
    # rospy.is_shutdown kontrolü ile Qt uygulamasını kapatabilmek için QTimer kullanalım
    timer = QTimer()
    timer.timeout.connect(lambda: QApplication.quit() if rospy.is_shutdown() else None)
    timer.start(100)
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
