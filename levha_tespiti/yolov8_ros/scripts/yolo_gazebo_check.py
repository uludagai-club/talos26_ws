#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloGazeboVisual:
    def __init__(self):
        rospy.init_node("yolo_gazebo_visual")

        # 🔴 YOLO modelini yükle
        self.model = YOLO("best.pt")

        # 🔴 Confidence düşük tutuyoruz (küçük objeleri kaçırmamak için)
        self.conf = 0.40

        self.bridge = CvBridge()

        # 🔴 Gazebo kamera topic
        self.sub = rospy.Subscriber(
            "/cart/front_camera/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("YOLO Gazebo Visual Node started for /cart/front_camera/image_raw")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"CV Bridge error: {e}")
            return

        # 🔴 YOLO tahmini
        results = self.model(frame, conf=self.conf)

        # 🔴 Terminale yazdır
        detected_objects = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf_score = float(box.conf[0])
                detected_objects.append(f"{self.model.names[cls]} ({conf_score:.2f})")
        if detected_objects:
            rospy.loginfo(f"Detected: {', '.join(detected_objects)}")

        # 🔴 Görsel üstüne kutular çiz
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO GAZEBO VISUAL", annotated_frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    node = YoloGazeboVisual()
    rospy.spin()
