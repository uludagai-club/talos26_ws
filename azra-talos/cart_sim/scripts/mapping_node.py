#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Open3D Tabanlı 3D Haritalama Modülü - MappingNode (REFACTORED)
==============================================================

Refactoring Özeti:
------------------
1. **TF Broadcasting**: Odom mesajı artık 'world' -> 'base_link' TF'i olarak yayınlanıyor.
2. **Strict Sync**: LiDAR ve Kamera senkronize, Odometry ayrıldı.
3. **Exact Time Lookup**: LiDAR timestamp'i ile tam zamanlı TF sorgusu yapılıyor (interpolasyonlu).
4. **Keyframe Mapping**: Sadece belirli miktar hareket edince harita güncelleniyor (Ghosting fix).

"""

import rospy
import numpy as np
import open3d as o3d
import threading
import struct
import os
import math
from datetime import datetime
from typing import Optional, Tuple
from collections import deque

# ROS Mesaj Tipleri
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
import sensor_msgs.point_cloud2 as pc2

# TF2
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_euler, euler_from_quaternion

# Sensör senkronizasyonu
import message_filters

# Görüntü dönüşümü
from cv_bridge import CvBridge


# ==============================================================================
# YAPILANDIRMA PARAMETRELERİ
# ==============================================================================
class MappingConfig:
    """Haritalama parametreleri için yapılandırma sınıfı."""
    
    # ROS Topic'leri
    LIDAR_TOPIC = "/cart/center_laser/scan"
    CAMERA_TOPIC = "/cart/front_camera/image_raw"
    IMU_TOPIC = "/imu"
    ODOM_TOPIC = "/base_pose_ground_truth"
    
    # Çıkış Topic'leri
    GLOBAL_MAP_TOPIC = "/global_map"
    
    # TF Frame'leri
    LIDAR_FRAME = "velodyne"
    CAMERA_FRAME = "zed2_left_camera_optical_frame"
    WORLD_FRAME = "world"
    BASE_FRAME = "base_link" # Genellikle base_link veya chassis
    
    # TF Broadcasting Ayarı
    PUBLISH_ODOM_TF = True  # Eğer True ise, MappingNode Odom->TF yayını yapar
    
    # Keyframe Mapping (HAYALET GÖRÜNTÜ ÖNLEME)
    # Haritaya sadece bu miktar hareket edince yeni frame ekle
    KEYFRAME_MIN_DIST = 0.5    # 0.5 metre
    KEYFRAME_MIN_ANGLE = 10.0   # 10 derece
    
    # Senkronizasyon (LiDAR ve Kamera)
    SYNC_SLOP = 0.5  # 500ms (Gevşek eşleşme)
    SYNC_QUEUE_SIZE = 10
    
    # Voxel Downsampling
    VOXEL_SIZE = 0.05  # 5cm
    DOWNSAMPLE_EVERY_N_FRAMES = 20
    
    # Statistical Outlier Removal
    OUTLIER_REMOVAL_ENABLED = True
    OUTLIER_NB_NEIGHBORS = 20
    OUTLIER_STD_RATIO = 2.0
    
    # Z-Filter
    Z_FILTER_ENABLED = True
    Z_FILTER_MIN = -1.0
    Z_FILTER_MAX = 10.0
    
    # TF Timeout
    TF_WAIT_TIMEOUT = 1.0
    
    # Canlı Yayın Hz
    PUBLISH_RATE_HZ = 1.0
    
    # Harita Kaydetme
    OUTPUT_PATH = "/home/kerem/talos-maps/"
    
    # LiDAR Filtreleme
    LIDAR_MIN_RANGE = 0.5
    LIDAR_MAX_RANGE = 100.0
    
    # Kamera Intrinsic
    CAMERA_WIDTH = 1280
    CAMERA_HEIGHT = 720
    CAMERA_FX = 522.0
    CAMERA_FY = 522.0
    CAMERA_CX = 640.0
    CAMERA_CY = 360.0


# ==============================================================================
# LIDAR DESKEWING (Sadeleştirilmiş)
# ==============================================================================
class LidarDeskewer:
    """IMU verisi kullanarak LiDAR tarama bozulmasını düzelten sınıf."""
    def __init__(self, scan_duration: float = 0.1):
        self.scan_duration = scan_duration
        self.imu_buffer = deque(maxlen=100)
        self._lock = threading.Lock()
    
    def add_imu_measurement(self, imu_msg: Imu):
        with self._lock:
            self.imu_buffer.append({
                'stamp': imu_msg.header.stamp.to_sec(),
                'angular_velocity': np.array([
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z
                ])
            })
            
    def get_angular_velocity_at_time(self, timestamp: float) -> np.ndarray:
        with self._lock:
            if len(self.imu_buffer) < 2:
                return np.zeros(3)
            buffer_list = list(self.imu_buffer)
            for i in range(len(buffer_list) - 1):
                t1 = buffer_list[i]['stamp']
                t2 = buffer_list[i + 1]['stamp']
                if t1 <= timestamp <= t2:
                    alpha = (timestamp - t1) / (t2 - t1) if (t2 - t1) > 0 else 0
                    w1 = buffer_list[i]['angular_velocity']
                    w2 = buffer_list[i + 1]['angular_velocity']
                    return w1 + alpha * (w2 - w1)
            return buffer_list[-1]['angular_velocity']


# ==============================================================================
# RENK PROJEKSİYONU
# ==============================================================================
class ColorProjector:
    def __init__(self, config: MappingConfig):
        self.config = config
        self.cv_bridge = CvBridge()
        self.K = np.array([
            [config.CAMERA_FX, 0, config.CAMERA_CX],
            [0, config.CAMERA_FY, config.CAMERA_CY],
            [0, 0, 1]
        ], dtype=np.float64)
        self.tf_buffer = None
        
    def set_tf_buffer(self, tf_buffer: tf2_ros.Buffer):
        self.tf_buffer = tf_buffer
    
    def get_lidar_to_camera_transform(self, timestamp: rospy.Time) -> Optional[np.ndarray]:
        if self.tf_buffer is None: return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.config.CAMERA_FRAME,
                self.config.LIDAR_FRAME,
                timestamp,
                rospy.Duration(0.1)
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
            rot_matrix[0, 3] = t.x
            rot_matrix[1, 3] = t.y
            rot_matrix[2, 3] = t.z
            return rot_matrix
        except Exception:
            return None
    
    def project_colors(self, points: np.ndarray, image_msg: Image, timestamp: rospy.Time) -> np.ndarray:
        n_points = len(points)
        colors = np.ones((n_points, 3), dtype=np.uint8) * 128
        if n_points == 0: return colors
        
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception: return colors
        
        T_lidar_to_cam = self.get_lidar_to_camera_transform(timestamp)
        if T_lidar_to_cam is None: return colors
        
        points_homo = np.hstack([points, np.ones((n_points, 1))])
        points_cam = (T_lidar_to_cam @ points_homo.T).T[:, :3]
        valid_mask = points_cam[:, 2] > 0.1
        
        u = (self.K[0, 0] * points_cam[:, 0] / points_cam[:, 2] + self.K[0, 2]).astype(int)
        v = (self.K[1, 1] * points_cam[:, 1] / points_cam[:, 2] + self.K[1, 2]).astype(int)
        
        height, width = image.shape[:2]
        in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        valid = valid_mask & in_bounds
        
        valid_indices = np.where(valid)[0]
        # Vectorized color assignment possible, but loop is safer for bounds
        # Advanced: image[v[valid_indices], u[valid_indices]]
        # For safety/readability keeping loop or using advanced indexing carefully
        bgr = image[v[valid_indices], u[valid_indices]]
        colors[valid_indices] = bgr[:, [2, 1, 0]] # BGR -> RGB
        
        return colors


# ==============================================================================
# ANA HARİTALAMA DÜĞÜMÜ
# ==============================================================================
class MappingNode:
    def __init__(self):
        rospy.init_node('mapping_node', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("MappingNode Başlatılıyor (REFACTORED)...")
        rospy.loginfo("=" * 60)
        
        self.config = MappingConfig()
        
        # 1. TF2 Components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Opsiyonel: TF Broadcaster (Odom -> TF)
        if self.config.PUBLISH_ODOM_TF:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            rospy.loginfo(f"TF Broadcaster AKTİF: {self.config.WORLD_FRAME} -> {self.config.BASE_FRAME}")
        else:
            self.tf_broadcaster = None
            rospy.loginfo("TF Broadcaster PASİF (Harici kaynak bekleniyor)")

        # 2. Helper Classes
        self.deskewer = LidarDeskewer()
        self.color_projector = ColorProjector(self.config)
        self.color_projector.set_tf_buffer(self.tf_buffer)
        
        # 3. State Variables
        self.global_map = o3d.geometry.PointCloud()
        self.global_map_lock = threading.Lock()
        self.frame_count = 0
        
        # Kamera State (Latest-Only)
        self.latest_image_msg = None
        self.camera_lock = threading.Lock()
        
        # Keyframe State
        self.last_keyframe_pose = None # 4x4 matris
        self.last_keyframe_time = rospy.Time(0)
        
        # 4. Publishers & Subscribers
        self.map_pub = rospy.Publisher(self.config.GLOBAL_MAP_TOPIC, PointCloud2, queue_size=1)
        self.save_map_srv = rospy.Service('~save_map', Empty, self.save_map_callback)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0/self.config.PUBLISH_RATE_HZ), self.publish_global_map_callback)
        
        self._setup_subscribers()
        
        rospy.loginfo("MappingNode Hazır!")

    def _setup_subscribers(self):
        # IMU (Deskewing için)
        self.imu_sub = rospy.Subscriber(self.config.IMU_TOPIC, Imu, self.imu_callback, queue_size=100)
        
        # Odometry (Ayrı abone, senkronizasyona girmez)
        self.odom_sub = rospy.Subscriber(self.config.ODOM_TOPIC, Odometry, self.odom_callback, queue_size=10)
        
        # Kamera (Latest Only)
        self.camera_sub = rospy.Subscriber(
            self.config.CAMERA_TOPIC, 
            Image, 
            self.camera_callback, 
            queue_size=1
        )
        
        # LiDAR (Ana Trigger - Freshest Data Only)
        # queue_size=1 ve buff_size=2**24 çok önemli:
        # Eğer işlem hatırı 100ms sürüyorsa ve veri 10Hz geliyorsa, kuyruk birikmemeli.
        # Eski veriyi atıp hep en yeniyi işlemeliyiz.
        self.lidar_sub = rospy.Subscriber(
            self.config.LIDAR_TOPIC, 
            PointCloud2, 
            self.lidar_callback, 
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo(f"Subscribers Ready. REAL-TIME MODE (Queue=1). Sync Slop={self.config.SYNC_SLOP}s")

    def camera_callback(self, msg: Image):
        """Kameradan gelen SON görüntüyü sakla."""
        with self.camera_lock:
            self.latest_image_msg = msg

    def get_closest_image(self, timestamp: rospy.Time) -> Optional[Image]:
        """En son gelen görüntüyü kontrol et, zaman farkı azsa döndür."""
        target_time = timestamp.to_sec()
        
        with self.camera_lock:
            if self.latest_image_msg is None:
                return None
            
            img_time = self.latest_image_msg.header.stamp.to_sec()
            diff = abs(img_time - target_time)
            
            # Eğer zaman farkı tolerans içindeyse (0.2s veya config)
            if diff < 0.2:  # User 0.2s istedi, config.SYNC_SLOP da kullanılabilir
                return self.latest_image_msg
            else:
                # Görüntü çok eski veya çok yeni
                return None

    def imu_callback(self, msg: Imu):
        # IMU verisi geldikçe deskewer tamponuna ekle
        self.deskewer.add_imu_measurement(msg)

    def odom_callback(self, msg: Odometry):
        """
        Odometry verisini alır ve eğer yapılandırıldıysa TF olarak yayınlar.
        """
        if rospy.is_shutdown():
            return
            
        if self.tf_broadcaster:
            # Odom mesajından 'world' -> 'base_link' TF'i oluştur
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.config.WORLD_FRAME # Genellikle 'world' veya 'odom'
            t.child_frame_id = self.config.BASE_FRAME   # 'base_link'
            
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)

    def is_keyframe(self, current_pose_matrix: np.ndarray) -> bool:
        """
        Mevcut pozun bir Keyframe olup olmadığını kontrol et (yeterince hareket var mı?).
        """
        if self.last_keyframe_pose is None:
            return True # İlk frame her zaman keyframe
            
        # Göreceli dönüşümü hesapla: T_rel = inv(T_last) * T_curr
        T_rel = np.linalg.inv(self.last_keyframe_pose) @ current_pose_matrix
        
        # Translation farkı
        dx = T_rel[0, 3]
        dy = T_rel[1, 3]
        dz = T_rel[2, 3]
        dist = np.sqrt(dx*dx + dy*dy + dz*dz)
        
        if dist > self.config.KEYFRAME_MIN_DIST:
            return True
            
        # Rotation farkı (Trace üzerinden yaklaşık açı hesabı)
        # tr(R) = 1 + 2*cos(theta) -> theta = arccos((tr(R)-1)/2)
        trace = np.trace(T_rel[:3, :3])
        val = (trace - 1.0) / 2.0
        # Clipping for numerical stability
        val = max(min(val, 1.0), -1.0)
        angle_rad = np.arccos(val)
        angle_deg = np.degrees(angle_rad)
        
        if angle_deg > self.config.KEYFRAME_MIN_ANGLE:
            return True
            
        return False

    def lidar_callback(self, lidar_msg: PointCloud2):
        """
        Ana İşleme Döngüsü (LiDAR Driven)
        1. Raw Log yaz.
        2. TF al.
        3. Keyframe check.
        4. Kamera eşleştir (Manual Sync).
        5. Haritaya ekle.
        """
        # [Debug] Raw Trigger Log
        rospy.loginfo_throttle(5, f"[RAW] LiDAR received at {lidar_msg.header.stamp.to_sec():.2f}")
        
        if rospy.is_shutdown():
            return

        try:
            lidar_time = lidar_msg.header.stamp
            
            # 1. TF LOOKUP
            if not self.tf_buffer.can_transform(self.config.WORLD_FRAME, self.config.LIDAR_FRAME, lidar_time, rospy.Duration(0.2)):
                return 

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config.WORLD_FRAME,
                    self.config.LIDAR_FRAME,
                    lidar_time,
                    rospy.Duration(self.config.TF_WAIT_TIMEOUT)
                )
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                return
            
            # 4x4 Matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            current_pose = quaternion_matrix([q.x, q.y, q.z, q.w])
            current_pose[0, 3] = t.x
            current_pose[1, 3] = t.y
            current_pose[2, 3] = t.z
            
            # 2. KEYFRAME CHECK
            if not self.is_keyframe(current_pose):
                return
            
            # 3. VERİ İŞLEME
            points = self._pointcloud2_to_numpy(lidar_msg)
            if len(points) == 0: return
            
            # Filtreleme
            dists = np.linalg.norm(points, axis=1)
            mask = (dists > self.config.LIDAR_MIN_RANGE) & (dists < self.config.LIDAR_MAX_RANGE)
            points = points[mask]
            if len(points) == 0: return

            # 4. KAMERA SENKRONİZASYONU (MANUEL FALLBACK)
            camera_msg = self.get_closest_image(lidar_time)
            
            if camera_msg:
                # Eşleşme bulundu, renkli üret
                colors = self.color_projector.project_colors(points, camera_msg, lidar_time)
            else:
                # Eşleşme yok, gri devam et (FALLBACK)
                colors = np.ones((len(points), 3), dtype=np.uint8) * 128
                # Debug log: Neden eşleşmedi?
                if self.latest_image_msg:
                    diff = abs(self.latest_image_msg.header.stamp.to_sec() - lidar_time.to_sec())
                    rospy.logdebug_throttle(1, f"Cam Sync Fail. Diff: {diff:.3f}s (Max: 0.2s)")
                else:
                    rospy.logdebug_throttle(1, "Cam Sync Fail. No Image.")
            
            # Global Frame Transform
            points_homo = np.hstack([points, np.ones((len(points), 1))])
            points_global = (current_pose @ points_homo.T).T[:, :3]
            
            # Z-Filter
            if self.config.Z_FILTER_ENABLED:
                z_mask = (points_global[:, 2] > self.config.Z_FILTER_MIN) & \
                         (points_global[:, 2] < self.config.Z_FILTER_MAX)
                points_global = points_global[z_mask]
                colors = colors[z_mask]
            
            if len(points_global) == 0: return

            # 5. HARİTAYA EKLEME
            pcd_new = o3d.geometry.PointCloud()
            pcd_new.points = o3d.utility.Vector3dVector(points_global)
            pcd_new.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
            
            # Local Downsample
            pcd_new = pcd_new.voxel_down_sample(voxel_size=self.config.VOXEL_SIZE)
            
            with self.global_map_lock:
                self.global_map += pcd_new
                
                self.frame_count += 1
                self.last_keyframe_pose = current_pose
                
                # Periyodik Bakım
                if self.frame_count % self.config.DOWNSAMPLE_EVERY_N_FRAMES == 0:
                    self.global_map = self.global_map.voxel_down_sample(self.config.VOXEL_SIZE)
                         
            rospy.loginfo_throttle(2, f"Keyframe Eklendi ({'Renkli' if camera_msg else 'Gri'}). Map Size: {len(self.global_map.points)} pts")

        except Exception as e:
            rospy.logerr(f"Hata: {e}")

    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        # Hızlı veri okuma
        # Not: struct.unpack iterasyonu Python'da yavaş olabilir. 
        # PC2 read_points veya numpy frombuffer daha iyidir ama struct yapısı karmaşıksa bu kalabilir.
        # Basitleştirilmiş versiyon:
        
        # Generator'dan listeye
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen), dtype=np.float32)
        return points

    def publish_global_map_callback(self, event):
        if rospy.is_shutdown():
            return

        with self.global_map_lock:
            n_points = len(self.global_map.points)
            if n_points == 0: return
            
            # Periyodik Log (Her 5 saniyede bir)
            rospy.loginfo_throttle(5, f"[Status] Current Map Size: {n_points} points")
            
            # Downsample edilmiş halini yayınla (Bandwidth tasarrufu)
            # pcd_small = self.global_map.voxel_down_sample(0.1) 
            msg = self._o3d_to_pointcloud2(self.global_map)
            self.map_pub.publish(msg)

    def _o3d_to_pointcloud2(self, pcd):
        points = np.asarray(pcd.points)
        if not pcd.has_colors():
             colors = np.ones((len(points), 3)) * 0.5
        else:
             colors = np.asarray(pcd.colors)
        
        # Header
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.config.WORLD_FRAME
        
        # Veriyi pc2 create_cloud ile oluşturmak daha güvenli ve hızlı
        # [(x,y,z,r,g,b), ...] formatını hazırla (Renk paketleme gerekebilir)
        # Basitlik için sadece XYZ gönderelim veya renkli yapmak için detaylı paketleme gerekir
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
             # RGB eklemek detaylı struct pack gerektirir, şimdilik XYZ
        ]
        
        # Renkleri float'a pack edelim
        # RGB -> unpack('f', pack('BBBB', b, g, r, 0))[0]
        pc_data = []
        for i in range(len(points)):
            r = int(colors[i][0] * 255)
            g = int(colors[i][1] * 255)
            b = int(colors[i][2] * 255)
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]
            pc_data.append([points[i][0], points[i][1], points[i][2], rgb])
            
        fields_rgb = fields + [PointField('rgb', 12, PointField.FLOAT32, 1)]
        return pc2.create_cloud(header, fields_rgb, pc_data)

    def _safe_log(self, msg, is_error=False):
        """Shutdown-safe logging."""
        if rospy.is_shutdown():
            print(f"[MappingNode] {msg}")
        else:
            if is_error:
                rospy.logerr(msg)
            else:
                rospy.loginfo(msg)

    def save_map_callback(self, req):
        self.save_map()
        return EmptyResponse()

    def save_map(self):
        """
        Global haritayı güvenli şekilde kaydet (Shutdown-Proof).
        """
        # 1. Stop Incoming Data to prevent new callbacks
        try:
            if hasattr(self, 'lidar_sub'): self.lidar_sub.unregister()
            if hasattr(self, 'camera_sub'): self.camera_sub.unregister()
            if hasattr(self, 'odom_sub'): self.odom_sub.unregister()
        except Exception:
            pass

        # 2. Determine Path
        try:
            # Try to get param, fallback to config if rospy is dead
            save_dir = rospy.get_param('~map_save_path', self.config.OUTPUT_PATH)
        except Exception:
            save_dir = self.config.OUTPUT_PATH
            
        try:
            os.makedirs(save_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"talos_map_{timestamp}"
            pcd_path = os.path.join(save_dir, f"{filename}.pcd")
            
            self._safe_log(f"Shutdown Save Triggered... ({len(self.global_map.points)} points)")
            
            with self.global_map_lock:
                 if len(self.global_map.points) > 0:
                     o3d.io.write_point_cloud(pcd_path, self.global_map)
                     self._safe_log(f"SUCCESS: Map saved to {pcd_path}")
                 else:
                     self._safe_log("Warning: Map is empty, skipping save.")
                     
        except Exception as e:
            self._safe_log(f"Map Save Failed: {e}", is_error=True)

    # def run(self): REMOVED - Logic moved to __main__


if __name__ == '__main__':
    node = None
    try:
        node = MappingNode()
        
        # Manual Spin to allow Try-Finally
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n(!) SCRIPT ENDING - FORCING MAP SAVE...")
        if node is not None:
            node.save_map()
