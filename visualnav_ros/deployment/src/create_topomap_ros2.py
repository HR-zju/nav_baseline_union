import argparse
import os
import time
from utils import msg_to_pil

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy

# IMAGE_TOPIC = "/usb_cam/image_raw"
IMAGE_TOPIC = "/camera/image"
TOPOMAP_IMAGES_DIR = "../topomaps/images"


def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e))


class CreateTopomapNode(Node):
    def __init__(self, args):
        super().__init__("create_topomap")
        self.args = args
        self.obs_img = None
        self.start_time = float("inf")
        self.i = 0

        # 创建订阅者和发布者
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.callback_obs, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.callback_joy, 10
        )

        # 创建定时器
        self.timer = self.create_timer(args.dt, self.timer_callback)

        # 创建拓扑图目录
        self.topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
        if not os.path.isdir(self.topomap_name_dir):
            os.makedirs(self.topomap_name_dir)
        else:
            self.get_logger().info(f"{self.topomap_name_dir} already exists. Removing previous images...")
            remove_files_in_dir(self.topomap_name_dir)

    def callback_obs(self, msg: Image):
        self.obs_img = msg_to_pil(msg)

    def callback_joy(self, msg: Joy):
        if msg.buttons[0]:
            self.get_logger().info("Shutdown requested via Joy message.")
            rclpy.shutdown()

    def timer_callback(self):
        if self.obs_img is not None:
            self.obs_img.save(os.path.join(self.topomap_name_dir, f"{self.i}.png"))
            self.get_logger().info(f"Published image {self.i}")
            self.i += 1
            self.start_time = time.time()
            self.obs_img = None

        if time.time() - self.start_time > 2 * self.args.dt:
            self.get_logger().info(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            rclpy.shutdown()


def main(args):
    rclpy.init()
    node = CreateTopomapNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topological map images in ../topomaps/images directory (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=1.0,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 1.0)",
    )
    args = parser.parse_args()

    main(args)