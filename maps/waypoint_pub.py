#!/usr/bin/env python3
import os
import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# NOT: Eski 60-node hardcoded CUSTOM_EDGES kaldirildi. Graf artik hedef_yoneticisi'nin
# build_track_graph() ciktisindan uretilir (maps/gen_track_graph.py -> final_graph.yaml,
# 644 node + edges). Hem node hem edge dosyadan okunur — guncel grafla daima senkron.

def publish_waypoints():
    rospy.init_node('waypoint_publisher')

    # Topic ismi: /waypoint
    marker_pub = rospy.Publisher('/waypoint', MarkerArray, queue_size=10, latch=True)
    rate = rospy.Rate(1)

    # Container'da /app/final_graph.yaml (maps/final_graph.yaml mount edilir);
    # yerelde script yanindaki final_graph.yaml.
    here = os.path.dirname(os.path.abspath(__file__))
    graph_path = next((p for p in ('/app/final_graph.yaml',
                                   os.path.join(here, 'final_graph.yaml'))
                       if os.path.exists(p)), '/app/final_graph.yaml')

    try:
        with open(graph_path, "r") as file:
            data = yaml.safe_load(file)
    except Exception as e:
        rospy.logerr(f"DOSYA OKUNAMADI: {e}")
        return

    rospy.loginfo(f"Waypoint yayini basladi — {len(data.get('nodes', []))} node, "
                  f"{len(data.get('edges', []))} edge ({graph_path}).")

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

        for u, v in data.get('edges', []):
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