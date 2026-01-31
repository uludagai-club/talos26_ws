#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Point
import cv2

class YOLOv8Node:
    def __init__(self):
        rospy.init_node('yolov8_node', anonymous=True)
        rospy.loginfo("Node initialized")

        self.bridge = CvBridge()

        try:
            rospy.loginfo("Loading YOLO model...")
            self.model = YOLO("/home/selenay/traffic_ws/src/yolov8_ros/scripts/best.pt")
            rospy.loginfo("Model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Model loading failed: {e}")
            rospy.signal_shutdown("Model load error")

        self.pub = rospy.Publisher('/yolov8/image_annotated', Image, queue_size=1)
        self.pos_pub = rospy.Publisher('/traffic_sign/position', Point, queue_size=10)
        self.sub = rospy.Subscriber('/cart/front_camera/image_raw', Image, self.image_callback)

        rospy.loginfo("Subscriber and Publishers initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(cv_image)

            h, w, _ = cv_image.shape

            # Eğer levha bulunmuşsa
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0]

                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2

                    # Normalize edilmiş koordinatlar
                    x_norm = (cx - w/2) / (w/2)      # [-1, +1]
                    y_norm = (h - cy) / h           # [0, 1]

                    pos = Point()
                    pos.x = float(x_norm)
                    pos.y = float(y_norm)
                    pos.z = 0.0

                    self.pos_pub.publish(pos)

            # YOLO çizimli görüntü
            annotated_frame = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.pub.publish(out_msg)

        except Exception as e:
            rospy.logerr(f"YOLO node error: {e}")

if __name__ == '__main__':
    YOLOv8Node()
    rospy.spin()
