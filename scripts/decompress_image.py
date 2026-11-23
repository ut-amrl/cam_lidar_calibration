#!/usr/bin/env python3
"""Decode recorded JPEG images while retaining their original header."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class Decoder(Node):
    def __init__(self):
        super().__init__("calibration_image_decoder")
        source = self.declare_parameter("compressed_topic", "/camera/rgb/image_raw/compressed").value
        target = self.declare_parameter("image_topic", "/calibration/image_raw").value
        self.publisher = self.create_publisher(Image, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(CompressedImage, source, self.decode, qos_profile_sensor_data)

    def decode(self, message):
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            self.get_logger().error("Could not decode compressed calibration image")
            return
        output = Image()
        output.header = message.header
        output.height, output.width = decoded.shape[:2]
        output.encoding, output.step = "bgr8", decoded.shape[1] * 3
        output.data = decoded.tobytes()
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = Decoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
