#!/usr/bin/env python3
import os
import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# YAML'daki kenarları kullan (varsa). Yoksa eski hardcoded CUSTOM_EDGES fallback.
USE_YAML_EDGES = os.environ.get("USE_YAML_EDGES", "1") != "0"

CUSTOM_EDGES = [
    (0,1),(1,2),(1,13),(2,3),(3,4),(4,12),(4,5),(5,6),(6,7),(7,11),(7,8),(8,9),
    (9,10),(13,22),(12,19),(11,16),(10,14),(14,15),(14,32),(15,16),(16,17),
    (16,27),(17,18),(18,19),(20,21),(21,22),(22,23),(19,24),(24,26),(26,25),
    (23,53),(53,54),(54,55),(55,56),(56,57),(57,58),(58,59),(59,60),(27,28),
    (28,29),(29,36),(36,37),(23,39),(39,38),(39,40),(40,52),(52,51),(51,50),
    (50,49),(49,41),(49,48),(48,47),(47,46),(46,42),(42,36),(32,31),(31,30),
    (30,33),(33,34),(30,34),(34,35),(34,43),(19,20),(24,25),(35,36),(43,44),
    (44,45),(45,46),(25,38),(38,41),(41,37),(37,25),(25,41),(38,37)
]

def publish_waypoints():
    rospy.init_node('waypoint_publisher')
    
    # Topic ismi: /waypoint
    marker_pub = rospy.Publisher('/waypoint', MarkerArray, queue_size=10, latch=True)
    rate = rospy.Rate(1)
    
    # Docker içindeki tam yol
    graph_path = '/app/final_graph.yaml'
    
    try:
        with open(graph_path, "r") as file:
            data = yaml.safe_load(file)
    except Exception as e:
        rospy.logerr(f"DOSYA OKUNAMADI: {e}")
        return

    # Edge kaynagi: yaml'da 'edges' anahtari varsa onu kullan, yoksa hardcoded
    yaml_edges = []
    if USE_YAML_EDGES and 'edges' in data and data['edges']:
        for e in data['edges']:
            if isinstance(e, (list, tuple)) and len(e) == 2:
                yaml_edges.append((int(e[0]), int(e[1])))
    active_edges = yaml_edges if yaml_edges else CUSTOM_EDGES
    src = "YAML" if yaml_edges else "CUSTOM_EDGES"
    rospy.loginfo("Waypoint yayini basladi (edge kaynagi: %s, %d edge)...",
                  src, len(active_edges))

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        node_coords = {}

        # --- Düğümler ---
        if 'nodes' in data:
            points_marker = Marker()
            points_marker.header.frame_id = "map"
            points_marker.header.stamp = rospy.Time.now()
            points_marker.ns = "nodes"
            points_marker.id = 0
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.pose.orientation.w = 1.0
            points_marker.scale.x = 0.4
            points_marker.scale.y = 0.4
            points_marker.scale.z = 0.4
            points_marker.color.r = 1.0
            points_marker.color.a = 1.0

            for node in data['nodes']:
                p = Point()
                p.x = node['x']
                p.y = node['y']
                points_marker.points.append(p)
                node_coords[node['id']] = (node['x'], node['y'])
            marker_array.markers.append(points_marker)

        # --- Kenarlar (CUSTOM_EDGES kullanılıyor) ---
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "edges"
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.1
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        for u, v in active_edges:
            # Düğümlerin YAML'dan okunan koordinatları arasında olup olmadığını kontrol et
            if u in node_coords and v in node_coords:
                p1 = Point(node_coords[u][0], node_coords[u][1], 0)
                p2 = Point(node_coords[v][0], node_coords[v][1], 0)
                line_marker.points.append(p1)
                line_marker.points.append(p2)
            else:
                # Eğer eksik bir düğüm varsa log basıp görebilirsin (opsiyonel)
                pass
                
        marker_array.markers.append(line_marker)

        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass