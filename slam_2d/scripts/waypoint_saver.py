#!/usr/bin/env python3
import rospy
import yaml
import os
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

class WaypointSaver:
    def __init__(self):
        rospy.init_node('waypoint_saver')
        
        # Get parameter for file path, default to 'waypoints.yaml' in the local directory
        self.filename = rospy.get_param('~filename', 'waypoints.yaml')
        self.points = []
        
        # Threshold to detect if we want to delete a point (in meters)
        self.delete_threshold = 1.0
        
        # Subscribe to RViz 'Publish Point' tool
        rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)
        
        # Publisher for visualizing recorded points immediately
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)
        
        rospy.loginfo("Waypoint Saver initialized.")
        rospy.loginfo(f"ACTION: Click empty space to ADD.")
        rospy.loginfo(f"ACTION: Click near a point (<{self.delete_threshold}m) to DELETE.")
        rospy.loginfo(f"Waypoints will be saved to: {os.path.abspath(self.filename)}")

    def click_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        # Check if we are close to any existing point
        closest_idx = -1
        min_dist = float('inf')
        
        for i, pt in enumerate(self.points):
            dist = ((pt['x'] - x)**2 + (pt['y'] - y)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx != -1 and min_dist < self.delete_threshold:
            # DELETE existing point
            removed_pt = self.points.pop(closest_idx)
            rospy.loginfo(f"Removed Waypoint (was at x={removed_pt['x']:.2f}, y={removed_pt['y']:.2f})")
            # Re-index remaining points ? Optional, but good for ID consistency if needed
            # For now keeping simple list, IDs will be index based on save
        else:
            # ADD new point
            waypoint = {'id': len(self.points), 'x': x, 'y': y, 'z': z}
            self.points.append(waypoint)
            rospy.loginfo(f"Added Waypoint: x={x:.2f}, y={y:.2f}")
        
        # Save and Visualize
        self.save_to_file()
        self.publish_markers()

    def save_to_file(self):
        # Update IDs before saving to ensure sequential 0,1,2...
        for i, pt in enumerate(self.points):
            pt['id'] = i
            
        try:
            with open(self.filename, 'w') as f:
                yaml.dump(self.points, f, default_flow_style=False)
            rospy.loginfo(f"File updated. Total waypoints: {len(self.points)}")
        except Exception as e:
            rospy.logerr(f"Failed to save waypoints: {e}")

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # DELETEALL marker to clear previous state
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, pt in enumerate(self.points):
            # Sphere Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "recorded_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pt['x']
            marker.pose.position.y = pt['y']
            marker.pose.position.z = pt['z']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            
            # Text Marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "recorded_waypoints_text"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.text = str(i)
            
            text_marker.pose.position.x = pt['x']
            text_marker.pose.position.y = pt['y']
            text_marker.pose.position.z = pt['z'] + 0.5
            text_marker.scale.z = 0.5
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
        saver = WaypointSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
