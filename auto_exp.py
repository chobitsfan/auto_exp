import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('auto_exp')
        self.subscription = self.create_subscription(Image, '/mono_left', self.callback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.cnt = 0
        self.shutter_spd_us = 10000
    def callback(self, msg):
        self.cnt = self.cnt + 1
        if self.cnt > 4:
            self.cnt = 0;
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            avg_bright = np.mean(img)
            if avg_bright <= 120 or avg_bright >= 130:
                adj = 1 + (128 / avg_bright - 1) * 0.1
                self.shutter_spd_us = int(self.shutter_spd_us * adj)
                #print(self.shutter_spd_us)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
