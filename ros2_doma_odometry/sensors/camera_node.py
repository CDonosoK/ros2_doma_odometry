import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraFish(Node):
    def __init__(self):
        super().__init__('camera_fish')
        
        self.imu_pub = self.create_publisher(Image, '/doma_odometry/camera/image_raw', 10)
        
        try:
            self.camera = cv2.VideoCapture(2)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            self.get_logger().info('Camera fish node started')
        except Exception as e:
            self.get_logger().error(f"Failed to connect to camera, check connection: {e}")
            rclpy.shutdown()

        self.rate = 30  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.camera_callback)
        self.bridge = CvBridge()

    def get_id_cameras(self):
        for id in range(10):
            camera = cv2.VideoCapture(id)
            if camera.isOpened():
                self.get_logger().info(f'Camera ID: {id} is available')
            else:
                self.get_logger().info(f'Camera ID: {id} is not available')
            camera.release()

    def camera_callback(self):
        ret, frame = self.camera.read()
        if ret:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_fish'
            self.imu_pub.publish(image_msg)
        else:
            self.get_logger().error('Failed to read frame from camera')

def main(args=None):
    rclpy.init(args=args)
    camera_fish = CameraFish()
    rclpy.spin(camera_fish)
    camera_fish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()