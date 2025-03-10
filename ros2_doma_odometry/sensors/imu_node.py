import serial
import struct
import rclpy
import math
from sensor_msgs.msg import Imu, MagneticField
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class YahboomImu(Node):
    def __init__(self):
        super().__init__('yahboom_imu')

        self.imu_publisher = self.create_publisher(Imu, '/doma_odometry/imu', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/doma_odometry/magnetometer', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_', '/dev/ttyUSB0'),
                ('baudrate_', 9600),
                ('timeout_', 0.5),
                ('rate_', 10.0),
            ]
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.buff = {}
        self.key = 0
        self.angular_velocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]

        self.port_ = self.get_parameter('port_').get_parameter_value().string_value
        self.baudrate_ = self.get_parameter('baudrate_').get_parameter_value().integer_value
        self.timeout_ = self.get_parameter('timeout_').get_parameter_value().double_value
        self.rate = self.get_parameter('rate_').get_parameter_value().double_value

        try:
            self.sensor = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=self.timeout_)
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            rclpy.shutdown()

        self.create_timer(1.0 / self.rate, self.main_loop)

    def main_loop(self):
        try:
            buff_count = self.sensor.inWaiting()
        except Exception as e:
            self.get_logger().error(f'IMU disconnected: {e}')
            rclpy.shutdown()
        else:
            if buff_count > 0:
                buff_data = self.sensor.read(buff_count)
                for i in range(buff_count):
                    self.handle_serial_data(buff_data[i])

    def check_sum(self, list_data, check_data):
        return sum(list_data) & 0xff == check_data
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]
    
    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))
    
    def publish_tf(self, quaternion):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "yahboom_imu"
        t.child_frame_id = "yahboom_imu"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(t)

    def handle_serial_data(self, raw_data):
        self.buff[self.key] = raw_data
        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            self.buff = {}
            return
        
        if self.key < 11:
            return
        else:
            data_buff = list(self.buff.values())
            if self.buff[1] == 0x51:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [float(self.hex_to_short(data_buff[2:10])[i]) / 32768.0 * 16 * 9.8 for i in range(3)]
                else:
                    self.get_logger().error('0x51 Check failure')

            elif self.buff[1] == 0x52:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angular_velocity = [float(self.hex_to_short(data_buff[2:10])[i]) / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
                else:
                    self.get_logger().error('0x52 Check failure')

            elif self.buff[1] == 0x53:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [float(self.hex_to_short(data_buff[2:10])[i]) / 32768.0 * 180 for i in range(3)]
                else:
                    self.get_logger().error('0x53 Check failure')

            elif self.buff[1] == 0x54:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = [float(val) for val in self.hex_to_short(data_buff[2:10])]
                else:
                    self.get_logger().error('0x54 Check failure')
            else:
                self.get_logger().error("Unknown header detected")

            self.buff = {}
            self.key = 0

            imu_msg = Imu()
            mag_msg = MagneticField()

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "yahboom_imu"

            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = "yahboom_imu"

            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            angle_radian[2] += math.pi / 2
            qua = self.quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            imu_msg.orientation.x = float(qua[0])
            imu_msg.orientation.y = float(qua[1])
            imu_msg.orientation.z = float(qua[2])
            imu_msg.orientation.w = float(qua[3])

            imu_msg.angular_velocity.x = float(self.angular_velocity[0])
            imu_msg.angular_velocity.y = float(self.angular_velocity[1])
            imu_msg.angular_velocity.z = float(self.angular_velocity[2])

            imu_msg.linear_acceleration.x = float(self.acceleration[0])
            imu_msg.linear_acceleration.y = float(self.acceleration[1])
            imu_msg.linear_acceleration.z = float(self.acceleration[2])

            mag_msg.magnetic_field.x = float(self.magnetometer[0])
            mag_msg.magnetic_field.y = float(self.magnetometer[1])
            mag_msg.magnetic_field.z = float(self.magnetometer[2])

            self.imu_publisher.publish(imu_msg)
            self.mag_publisher.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YahboomImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()