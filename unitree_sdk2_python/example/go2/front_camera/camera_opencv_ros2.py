import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()
        self.client = VideoClient()  
        self.client.SetTimeout(3.0)
        self.client.Init()

        # 设置定时器，频率为 10 Hz（每 0.1 秒调用一次）
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        code, data = self.client.GetImageSample()
        if code != 0:
            self.get_logger().error("Get image sample error. code: %d" % code)
            return

        # 将图像数据转换为 OpenCV 格式
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        # 发布图像
        if image is not None:
            self.publish_image(image)

    def publish_image(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)  
    except KeyboardInterrupt:
        pass  

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

