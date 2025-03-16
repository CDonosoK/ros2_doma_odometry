#!/usr/bin/env python3

import os
import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, Imu, MagneticField, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from cv_bridge import CvBridge

class VisualOdometry:

class DomaVodom(Node):
    def __init__(self):
        super().__init__('doma_vodom')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/image_raw'),
                ('imu_topic', '/doma_odometry/imu'),
                ('magnetometer_topic', '/doma_odometry/magnetometer'),
                ('gps_topic', '/doma_odometry/gps'),
                ('rate', 10.0),
                ('orb_features', 3000),
            ]
        )

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.magnetometer_topic = self.get_parameter('magnetometer_topic').get_parameter_value().string_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value

        self.orb_features = cv2.ORB_create(self.get_parameter('orb_features').get_parameter_value().integer_value)
        FLAN_INDEX_LSH = 6
        index_params = dict(algorithm=FLAN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, self.magnetometer_topic, self.mag_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        
        self.bridge = CvBridge()

        self.main_callback = self.create_timer(1.0 / self.get_parameter('rate').get_parameter_value().double_value, self.main_loop)


    def image_callback(self, msg):
        pass

    def imu_callback(self, msg):
        pass

    def mag_callback(self, msg):
        pass

    def gps_callback(self, msg):
        pass

    def main_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DomaVodom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()