#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime


class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower_node', anonymous=True)
        self.vehicle_center_pub = rospy.Publisher('/vehicle_center', Float32, queue_size=10)
        self.road_center_pub = rospy.Publisher('/road_center', Float32, queue_size=10)

        
        self.offset_pub = rospy.Publisher('/lane_offset', Float32, queue_size=10)


        # ==== MODEL ====
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, '..', 'models', 'best.pt')
        self.model = YOLO(model_path)

        # ==== ROS ====
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/line', Float32, queue_size=10)

        rospy.Subscriber(
            "/cart/front_camera/image_raw/compressed",
            CompressedImage,
            self.callback
        )

        # ==== PARAM ====
        self.Kp = 0.12
        self.frame_to_show = None
        self.log = open("lane_log.txt", "w")

        rospy.loginfo("Lane follower hazır.")

    def callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = frame.shape
            vehicle_center = w // 2

            results = self.model.predict(frame, conf=0.6, verbose=False)

            left_points = []
            right_points = []

            draw = frame.copy()

            # YOLO hiç sonuç vermezse patlamasın
            if len(results) == 0 or len(results[0].boxes) == 0:
                self.frame_to_show = draw
                return

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Şeridin yere değdiği alt merkez noktası
                bottom_x = (x1 + x2) // 2
                bottom_y = y2

                # Çizimler (model ne görüyor)
                cv2.rectangle(draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(draw, (bottom_x, bottom_y), 5, (255, 0, 0), -1)

                # Sol / Sağ ayır
                if bottom_x < vehicle_center:
                    left_points.append(bottom_x)
                else:
                    right_points.append(bottom_x)

            # Eğer iki taraf da varsa gerçek yol ortası hesaplanır
            if len(left_points) > 0 and len(right_points) > 0:
                left_x = int(np.mean(left_points))
                right_x = int(np.mean(right_points))

                road_center = (left_x + right_x) // 2
                vehicle_center = w // 2
                self.vehicle_center_pub = rospy.Publisher('/vehicle_center', Float32,    queue_size=10)
                self.road_center_pub = rospy.Publisher('/road_center', Float32, queue_size=10)


                offset = road_center - vehicle_center
                angle = offset * self.Kp
                self.offset_pub.publish(float(offset))

                angle = max(min(angle, 30), -30)

                # Publish
                self.pub.publish(float(angle))

                # Log
                self.log.write(f"{datetime.now()} | Offset: {offset} | Angle: {angle}\n")

                # Çizimler
                cv2.line(draw, (vehicle_center, h), (vehicle_center, h-150), (0,0,255), 3)
                cv2.line(draw, (left_x, h), (left_x, h-150), (255,255,0), 3)
                cv2.line(draw, (right_x, h), (right_x, h-150), (255,255,0), 3)
                cv2.line(draw, (road_center, h), (road_center, h-150), (255,0,0), 4)

                cv2.putText(draw, f"Offset: {offset}", (20,40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                cv2.putText(draw, f"Angle: {angle:.2f}", (20,80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            self.frame_to_show = draw

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.frame_to_show is not None:
                cv2.imshow("Lane Follow", self.frame_to_show)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()

        self.log.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = LaneFollower()
    node.run()

