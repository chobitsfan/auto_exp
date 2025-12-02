import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
import cv2
import serial
import struct

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('auto_exp')
        self.subscription = self.create_subscription(Image, '/mono_left', self.callback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.cnt = 0
        self.shutter_spd_us = 10000
        self.ser = serial.Serial('/dev/ttyAMA2', 921600)
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
    def __del__(self):
        self.close()
    def callback(self, msg):
        self.cnt = self.cnt + 1
        if self.cnt > 4:
            self.cnt = 0;
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            avg_bright = np.mean(img)
            if avg_bright <= 120 or avg_bright >= 130:
                adj = 1 + (128 / avg_bright - 1) * 0.1
                self.shutter_spd_us = int(self.shutter_spd_us * adj)
                if self.shutter_spd_us > 45000:
                    self.shutter_spd_us = 45000
                elif self.shutter_spd_us < 500:
                    self.shutter_spd_us = 500
                #print(self.shutter_spd_us)
                buf = struct.pack('<H', self.shutter_spd_us)
                self.ser.write(buf)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
