#!/usr/bin/env python3
import math
import struct
import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

# ════════════════════════════════════════════════════════════════════════
#   AYARLANABİLİR PARAMETRELER — hepsi burada
#   (canlı: config/canli_params.yaml 'pointcloud:' — restart'sız uygulanır;
#    rosparam ~ override'ları başlangıçta hâlâ geçerlidir)
# ════════════════════════════════════════════════════════════════════════
CLOUD_TOPIC              = "/cart/center_laser/scan"  # (RESTART) abonelik başlangıçta kurulur
CLUSTER_DIST_THRESHOLD_M = 1.0    # m - kümeleme komşuluk eşiği
MIN_CLUSTER_SIZE         = 10     # nokta - küme alt sınırı
MAX_DISTANCE_M           = 12.0   # m - bu menzilin ötesi atılır
MIN_DISTANCE_M           = 0.3    # m - bundan yakını atılır (öz-yansıma)
MIN_HEIGHT_M             = -1.0   # m - z alt eşiği
MAX_HEIGHT_M             = 2.0    # m - z üst eşiği

try:
    from talos_common.canli_params import canli_parametre_izle
    _canli_izleyici = canli_parametre_izle("pointcloud", globals())
except Exception as _canli_e:
    _canli_izleyici = None
    print(f"[pointcloud_obstacle] canli_params yok, statik parametreler: {_canli_e}", flush=True)


class PointCloudObstaclePublisher:
    def __init__(self):
        rospy.init_node("pointcloud_obstacle_publisher")

        # rosparam (~) verilmişse başlangıçta üst bloğu ezer; verilmemişse
        # üst blok (veya canli_params.yaml override'ı) geçerli kalır.
        for _ad, _param in [("CLOUD_TOPIC", "~cloud_topic"),
                            ("CLUSTER_DIST_THRESHOLD_M", "~cluster_dist_threshold"),
                            ("MIN_CLUSTER_SIZE", "~min_cluster_size"),
                            ("MAX_DISTANCE_M", "~max_distance"),
                            ("MIN_DISTANCE_M", "~min_distance"),
                            ("MIN_HEIGHT_M", "~min_height"),
                            ("MAX_HEIGHT_M", "~max_height")]:
            globals()[_ad] = rospy.get_param(_param, globals()[_ad])

        self.pose_pub = rospy.Publisher("/obstacle_positions", PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=10)

        rospy.Subscriber(CLOUD_TOPIC, PointCloud2, self.cloud_callback)

    def cloud_callback(self, msg):
        points = []

        # ROS Noetic pc2.read_points() has a buffer boundary bug with Velodyne data.
        # Use numpy directly to read XYZ fields safely.
        try:
            # Find field offsets for x, y, z
            field_map = {f.name: f for f in msg.fields}
            if not all(k in field_map for k in ('x', 'y', 'z')):
                return
            fmt = msg.point_step
            ox = field_map['x'].offset
            oy = field_map['y'].offset
            oz = field_map['z'].offset
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            n_points = len(raw) // fmt
            if n_points == 0:
                return
            raw = raw[:n_points * fmt].reshape(n_points, fmt)
            xs = np.frombuffer(raw[:, ox:ox+4].copy().tobytes(), dtype=np.float32)
            ys = np.frombuffer(raw[:, oy:oy+4].copy().tobytes(), dtype=np.float32)
            zs = np.frombuffer(raw[:, oz:oz+4].copy().tobytes(), dtype=np.float32)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"PointCloud read error: {e}")
            return

        for x, y, z in zip(xs, ys, zs):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            dist = math.hypot(x, y)
            if dist < MIN_DISTANCE_M or dist > MAX_DISTANCE_M:
                continue
            if z < MIN_HEIGHT_M or z > MAX_HEIGHT_M:
                continue
            points.append((x, y))

        clusters = self.cluster_points(points)

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = msg.header.frame_id

        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0.3)

            marker_array.markers.append(marker)

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)

    def cluster_points(self, points):
        if not points:
            return []

        points = sorted(points, key=lambda p: math.atan2(p[1], p[0]))

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev = points[i - 1]
            curr = points[i]
            dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])

            if dist < CLUSTER_DIST_THRESHOLD_M:
                current_cluster.append(curr)
            else:
                if len(current_cluster) >= MIN_CLUSTER_SIZE:
                    clusters.append(current_cluster)
                current_cluster = [curr]

        if len(current_cluster) >= MIN_CLUSTER_SIZE:
            clusters.append(current_cluster)

        return clusters


if __name__ == "__main__":
    try:
        PointCloudObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
