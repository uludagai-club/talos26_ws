#!/usr/bin/env python3
import rospy
import yaml
import os
from visualization_msgs.msg import Marker, MarkerArray

class WaypointVisualizer:
    def __init__(self):
        rospy.init_node('waypoint_visualizer')
        
        self.filename = rospy.get_param('~filename', 'waypoints.yaml')
        self.marker_pub = rospy.Publisher('/visualized_waypoints', MarkerArray, queue_size=10, latch=True)
        
        rospy.loginfo(f"Loading waypoints from: {self.filename}")
        self.points = self.load_waypoints()
        
        if self.points:
            self.publish_markers()
            rospy.loginfo(f"Published {len(self.points)} waypoints.")
        else:
            rospy.logwarn("No waypoints found or file empty.")

    def load_waypoints(self):
        try:
            if not os.path.exists(self.filename):
                rospy.logwarn(f"File not found: {self.filename}")
                return []
                
            with open(self.filename, 'r') as f:
                return yaml.safe_load(f) or []
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
            return []

    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, pt in enumerate(self.points):
            # Sphere Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = pt.get('id', i)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pt['x']
            marker.pose.position.y = pt['y']
            marker.pose.position.z = pt.get('z', 0.0)
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Color (Green for loaded waypoints)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Text Marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "waypoints_ids"
            text_marker.id = pt.get('id', i) + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.text = str(pt.get('id', i))
            
            text_marker.pose.position.x = pt['x']
            text_marker.pose.position.y = pt['y']
            text_marker.pose.position.z = pt.get('z', 0.0) + 0.5
            text_marker.scale.z = 0.4
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        viz = WaypointVisualizer()
        viz.run()
    except rospy.ROSInterruptException:
        pass
