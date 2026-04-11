#!/usr/bin/env python3
"""
Debug script: Point cloud'un bir frame'ini alıp analiz eder.
"""
import rospy
import sys
import os
sys.path.append(os.path.expanduser("~/.local/lib/python3.8/site-packages"))

import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

current_pose = None

def pose_callback(msg):
    global current_pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_pose = (x, y, z, roll, pitch, yaw)
    print(f"\n[POSE] x={x:.2f}, y={y:.2f}, z={z:.2f}")
    print(f"[POSE] roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°, yaw={np.degrees(yaw):.1f}°")

def pc_callback(msg):
    print(f"\n[PC] Frame: {msg.header.frame_id}")
    print(f"[PC] Size: {msg.width}x{msg.height} = {msg.width * msg.height} points")
    
    # İlk 10 noktayı oku
    x_offset = next(f.offset for f in msg.fields if f.name == 'x')
    y_offset = next(f.offset for f in msg.fields if f.name == 'y')
    z_offset = next(f.offset for f in msg.fields if f.name == 'z')
    
    point_step = msg.point_step
    data = msg.data
    fmt = '<'
    
    points = []
    for i in range(min(1000, msg.width * msg.height)):
        offset = i * point_step
        try:
            x = struct.unpack(fmt + 'f', data[offset + x_offset:offset + x_offset + 4])[0]
            y = struct.unpack(fmt + 'f', data[offset + y_offset:offset + y_offset + 4])[0]
            z = struct.unpack(fmt + 'f', data[offset + z_offset:offset + z_offset + 4])[0]
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
        except:
            continue
    
    if points:
        pts = np.array(points)
        print(f"\n[STATS] Gecerli nokta: {len(pts)}")
        print(f"[STATS] X: min={pts[:,0].min():.2f}, max={pts[:,0].max():.2f}, mean={pts[:,0].mean():.2f}")
        print(f"[STATS] Y: min={pts[:,1].min():.2f}, max={pts[:,1].max():.2f}, mean={pts[:,1].mean():.2f}")
        print(f"[STATS] Z: min={pts[:,2].min():.2f}, max={pts[:,2].max():.2f}, mean={pts[:,2].mean():.2f}")
        
        # Mesafe analizi
        distances = np.sqrt(pts[:,0]**2 + pts[:,1]**2 + pts[:,2]**2)
        print(f"[STATS] Mesafe: min={distances.min():.2f}, max={distances.max():.2f}, mean={distances.mean():.2f}")
        
        # İlk 5 nokta
        print("\n[SAMPLE] Ilk 5 nokta (x, y, z):")
        for i, p in enumerate(pts[:5]):
            print(f"  {i}: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    
    rospy.signal_shutdown("Done")

if __name__ == '__main__':
    rospy.init_node('debug_pc', anonymous=True)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, pose_callback)
    rospy.sleep(1)  # Pose bekle
    rospy.Subscriber('/zed2/point_cloud/cloud_registered', PointCloud2, pc_callback)
    print("Waiting for data...")
    rospy.spin()
