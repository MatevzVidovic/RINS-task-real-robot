#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import math
import time

from ultralytics import YOLO

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

    def __init__(self):
        super().__init__('detect_faces')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', ''),
        ])

        # camera info (simulation)
        """ self.f_x = 277
        self.f_y = 277
        self.c_x = 160
        self.c_y = 120
        self.hfov = 2 * math.atan(self.c_x / self.f_x)
        self.vfov = 2 * math.atan(self.c_y / self.f_y) """

        # camera info (real robot)
        self.f_x = 201.04
        self.f_y = 201.04
        self.c_x = 124.62
        self.c_y = 125.16
        self.hfov = 2 * math.atan(self.c_x / self.f_x)
        self.vfov = 2 * math.atan(self.c_y / self.f_y)

        self.get_logger().info(f"hfov: {self.hfov}, vfov: {self.vfov}")

        self.angle_topic = "/detected_faces"

        self.detection_color = (0,0,255)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.scan = None
        
        # For publishing the markers
        self.angle_pub = self.create_publisher(PointStamped, self.angle_topic, QoSReliabilityPolicy.BEST_EFFORT)

        # For face detection
        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.model = YOLO("yolov8n.pt")

        # all detected faces by YOLO
        self.faces = []

        self.buffer_size = 10
        self.face_buffer = []

        # for unique faces
        self.point_id = 0
        self.min_distance_between_faces = 1.
        self.detected_faces = []

        self.timeout = 10

        self.get_logger().info(f"Node has been initialized! Will publish face markers to {self.angle_topic}.")

    def get_angle(self, x, y):

        angle_x = ((x - self.c_x ) / self.c_x) / (self.hfov)
        angle_y = ((y - self.c_y ) / self.c_y) / (self.vfov)

        return angle_x, angle_y

    def rgb_callback(self, data):

        self.faces = []

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #self.get_logger().info(f"Running inference on image...")

            # run inference
            res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

            # iterate over results
            for x in res:

                bbox = x.boxes.xyxy
                if bbox.nelement() == 0: # skip if empty
                    continue

                #self.get_logger().info(f"Person has been detected {bbox}!")

                bbox = bbox[0]

                cx = int((bbox[0]+bbox[2])/2)
                cy = int((bbox[1]+bbox[3])/2)

                self.faces.append((cx,cy))

                angle_x, angle_y = self.get_angle(cx, cy)

                if angle_x > 0.55 or angle_x < -0.55:
                    continue

                # To do on the real robot for filtering out real people
                if angle_y < 0:
                   continue

                # draw rectangle
                cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

                # draw the center of bounding box
                cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

                self.get_logger().info(f"x: {angle_x}, y: {angle_y}")

                self.publish_face(angle_x, angle_y)
                
                #self.get_logger().info(f"Sleeping...")
                #self.get_logger().info(f"Woke up")

            cv2.imshow("image", cv_image)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()
            
        except CvBridgeError as e:
            print(e)
            
    def distance(self, marker1, marker2):
        # calculate distance between 2 markers in a 3D space
        x1 = marker1.pose.position.x
        x2 = marker2.pose.position.x
        y1 = marker1.pose.position.y
        y2 = marker2.pose.position.y
        z1 = marker1.pose.position.z
        z2 = marker2.pose.position.z

        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

        return dist

    def new(self, marker):
        # check if marker is far enough from the already detected faces
        for face in self.detected_faces:
            if self.distance(marker, face) < self.min_distance_between_faces:
                return False
        
        return True
    
    def notFalsePositive(self, marker):
        # check if enough markers have been detected to prevent a false positive

        self.face_buffer.append(marker)
        if len(self.face_buffer) < self.buffer_size + 1:
            return False

        face_counter = 0
        for face in self.face_buffer:
            if self.distance(marker, face) < self.min_distance_between_faces / 2:
                face_counter += 1

        self.face_buffer.pop(0)

        if face_counter < self.buffer_size // 3:
            return False
        return True
    
    def publish_face(self, angle_x, angle_y):

        # if it's new, append it to detected faces
        self.detected_faces.append((angle_x, angle_y))


        point = PointStamped()
        point.point.x = angle_x
        point.point.y = angle_y

        # Publish the marker
        self.angle_pub.publish(point)
        #self.get_logger().info(f"The point ({angle_x}, {angle_y}) has been published to {self.angle_topic}. You are able to visualize it in Rviz")
    
            
def main():
    print('Face detection node starting.')

    rclpy.init(args=None)
    node = detect_faces()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()