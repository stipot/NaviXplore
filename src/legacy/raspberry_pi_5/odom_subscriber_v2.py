import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import time
import math

"""
Скрипт для получения координат ORB-SLAM3 из потока внутри Rasbperry Pi 5 с настроенным ROS 2
и передачи NMEA-сообщений по Bluetooth на мобильное устройство с Android.
"""

EARTH_RADIUS = 6371e3


def calculate_checksum(sentence):
    checksum = 0

    for char in sentence:
        checksum ^= ord(char)

    return checksum


def generate_gprmc(latitude, lat_dir, longitude, lon_dir, speed, course):
    now = time.gmtime()
    sentence = (
        f"GPRMC,{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.000,A,"
        f"{int(latitude):02d}{(latitude - int(latitude)) * 60:.4f},{lat_dir},"
        f"{int(longitude):03d}{(longitude - int(longitude)) * 60:.4f},{lon_dir},"
        f"{speed:.1f},{course:.1f},{now.tm_mday:02d}{now.tm_mon:02d}{now.tm_year % 100:02d},,"
    )
    checksum = calculate_checksum(sentence)

    return f"${sentence}*{checksum:02X}\r\n"


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__("odom_subscriber")
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )
        self.serial_port = serial.Serial("/dev/rfcomm0", 9600, timeout=1)
        self.serial_port.flush()

    def listener_callback(self, msg):
        latitude0 = 55.752593
        longitude0 = 37.626626

        latitude = (
            msg.pose.pose.position.y / (EARTH_RADIUS * (math.pi / 180)) + latitude0
        )
        longitude = (
            msg.pose.pose.position.x
            / (EARTH_RADIUS * (math.pi / 180))
            / (math.pi / 180 * latitude0)
            + longitude0
        )
        lat_dir = "N" if latitude >= 0 else "S"
        lon_dir = "E" if longitude >= 0 else "W"
        nmea_sentence = generate_gprmc(
            abs(latitude), lat_dir, abs(longitude), lon_dir, 0.0, 0.0
        )

        try:
            self.serial_port.write(nmea_sentence.encode("utf-8"))
            self.get_logger().info(f"Sent: {nmea_sentence.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
