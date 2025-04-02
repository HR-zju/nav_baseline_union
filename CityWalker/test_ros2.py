# test.py

import pytorch_lightning as pl
import argparse
import yaml
import os
from pl_modules.citywalk_datamodule import CityWalkDataModule
from pl_modules.teleop_datamodule import TeleopDataModule
from pl_modules.citywalker_module import CityWalkerModule
from pl_modules.citywalker_feat_module import CityWalkerFeatModule
import torch
import glob

# ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import threading
import queue
from collections import deque
import copy
import cv2
from cv_bridge import CvBridge
import torchvision.transforms.functional as TF
from std_msgs.msg import Bool, Float32MultiArray

from PIL import Image as PILImage
import numpy as np

torch.set_float32_matmul_precision('medium')
pl.seed_everything(42, workers=True)


def show_tensor(img_tensor):
    # 将张量转换为 numpy 数组，并调整通道顺序为 HWC
    img_np = img_tensor.permute(1, 2, 0).numpy()
    
    # 如果张量是 float 类型且值范围在 [0, 1]，转换为 [0, 255]
    if img_np.dtype == np.float32 or img_np.dtype == np.float64:
        img_np = (img_np * 255).astype(np.uint8)
    
    # 如果是 BGR 转 RGB（根据张量来源决定是否需要）
    # img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    
    # 显示图像
    cv2.imshow('Image', img_np)
    cv2.waitKey(1)
    # cv2.destroyAllWindows()

def msg_to_pil(msg: Image) -> PILImage.Image:
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    pil_image = PILImage.fromarray(img)
    return pil_image

class ImagePoseSyncNode(Node):
    def __init__(self):
        super().__init__('image_pose_sync_node')
        self.bridge = CvBridge()  

        # 初始化队列和缓冲区
        self.image_queue = [] # queue.Queue()
        self.image_batch = deque(maxlen=5)  # []
        self.image_batch_old = deque(maxlen=5)  # []
        self.waypoint_pub = self.create_publisher(Float32MultiArray, 'waypoint', 1)
        self.goal_pub = self.create_publisher(Bool, "/topoplan/reached_goal", 1)


        self.pose_buffer = deque(maxlen=10)  # 保留最近的20个位姿用于查找
        self.lock = threading.Lock()
        self.last_image_time = 0.
        
        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            5  # 适当调整队列大小
        )
        
        # 订阅里程计话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            100  # 适当调整队列大小
        )
        
        # 启动处理线程
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        self.get_logger().info("Image and Pose sync node initialized")
    
    def image_callback(self, msg):
        """图像回调函数，将图像和时间戳放入队列"""
        current_time = (msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9 ) * 1e-9
        time_since_last = (current_time - self.last_image_time)  # 转换为秒
        if time_since_last >= 1.0 / 6.0: # control the image frequence is about 6 hz
            # 
            try:
                # self.image_queue.put((msg.header.stamp, msg))
                pose = self.find_closest_pose(msg.header.stamp)
                if not pose:
                    self.get_logger().warning(f"no pose yet")
                    return 0
                # reshape images
                # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                pil_img = msg_to_pil(msg)

            
                # 2. 目标分辨率
                desired_height = 360
                desired_width = 640
                # 3. 先计算缩放比例，用 cv2.resize 进行缩放
                W, H = pil_img.size
                # breakpoint()
                scale = min(desired_height / H, desired_width / W)
                new_H, new_W = int(H * scale), int(W * scale)
                # breakpoint()
                if (new_H, new_W) != (H, W):
                    pil_img = pil_img.resize((new_W, new_H), PILImage.BILINEAR)
                img_np = np.array(pil_img)  # PIL Image → numpy
                # breakpoint()
                img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float() / 255.0
                # 5. 如果缩放后仍不满足尺寸，进行 padding 或 center_crop
                _, H_resized, W_resized = img_tensor.shape
                pad_height = desired_height - H_resized
                pad_width = desired_width - W_resized
                
                if pad_height > 0 or pad_width > 0:  # 需要 padding
                    pad_top = pad_height // 2
                    pad_bottom = pad_height - pad_top
                    pad_left = pad_width // 2
                    pad_right = pad_width - pad_left
                    
                    img_tensor = TF.pad(
                        img_tensor,
                        (pad_left, pad_top, pad_right, pad_bottom),
                    )
                
                elif pad_height < 0 or pad_width < 0:  # 需要裁剪
                    img_tensor = TF.center_crop(img_tensor, (desired_height, desired_width))
                
                # 6. 检查最终尺寸是否正确
                assert img_tensor.shape[1] == desired_height and img_tensor.shape[2] == desired_width, \
                    f"Final image shape {img_tensor.shape} != ({desired_height}, {desired_width})"
               
                ## show image
                # breakpoint()
                show_tensor(img_tensor)

                with self.lock:
                    self.image_batch.append((img_tensor, pose))
                self.last_image_time = current_time

            except Exception as e:
                self.get_logger().error(f"Error in image callback: {str(e)}")
    
    def odom_callback(self, msg):
        """里程计回调函数，存储位姿和时间戳"""
        try:
            with self.lock:
                # 存储时间戳和位姿
                self.pose_buffer.append((msg.header.stamp, msg.pose))
        except Exception as e:
            self.get_logger().error(f"Error in odom callback: {str(e)}")
    
    def find_closest_pose(self, image_stamp):
        """查找与图像时间戳最接近的位姿"""
        with self.lock:
            if not self.pose_buffer:
                return None
            
            image_nsec = image_stamp.nanosec + image_stamp.sec * 1e9

            # 简单的线性搜索找到时间最接近的位姿
            closest_pose = None
            # min_diff = float('inf')
            
            for pose_stamp, pose in self.pose_buffer:
                # breakpoint()
                pose_nsec = pose_stamp.nanosec + pose_stamp.sec * 1e9
                time_diff = pose_nsec - image_nsec

                if abs(time_diff) < 1e8 or time_diff > 0:
                    closest_pose = pose
                    break
                elif not self.pose_buffer:
                    # 如果时间差过大，删除旧数据（避免未来重复计算）
                    self.pose_buffer.popleft()
                    
            # return closest_pose
            return closest_pose if closest_pose is not None else self.pose_buffer[-1][1]

    
    def process_frames(self):
        """处理线程，从队列中取出图像并查找对应位姿"""
        
        while rclpy.ok():
            # 5 frames
            process_flag = False
            if len(self.image_batch) == 5:
                # breakpoint()
                    # self.image_batch has updated
                with self.lock:
                    if self.image_batch_old != self.image_batch[-1][-1]:
                        self.image_batch_old = copy.deepcopy(self.image_batch[-1][-1])
                        process_flag = True
            if process_flag:
                # breakpoint()
                self.process_batch(self.image_batch)
                        
                
            
    def process_batch(self, batch):
        """处理收集到的5帧图像和位姿"""
        self.get_logger().info(f"Processing batch of {len(batch)} frames")
        # 在这里添加你的处理逻辑
        # breakpoint()
        with self.lock:

            image_list = []
            pose_list = []

            for image, pose in self.image_batch:
                image_list.append(image)
                pose_list.append([pose.pose.position.x, pose.pose.position.y])
        pose_list.append(target_xy)
        # breakpoint()
        video_frames = torch.stack(image_list, dim=0).unsqueeze(0).to(device)
        input_positions = torch.tensor(pose_list).unsqueeze(0).to(device)
        # breakpoint()
        # video_frames = torch.rand([1, 5, 3, 360, 640])  # 或 torch.randn
        # input_positions = torch.zeros([1, 6, 2])       # 可能需要调整数量
        # input_positions[0, 5, 0] = 4
        # input_positions[0, 5, 1] = 0

        video_frames = video_frames.to(device)
        input_positions = input_positions.to(device)
        wp_pred, arrive_pred ,_, _  = model(video_frames, input_positions, future_obs=None) # this is we want
        # print('arrive_pred = {}'.format(arrive_pred))

        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = wp_pred[0][2].tolist()
        self.waypoint_pub.publish(waypoint_msg)
        reached_goal = False #(distances <1 ) #closest_node == goal_node
        reached_goal_msg = Bool()
        reached_goal_msg.data = bool(reached_goal)
        self.goal_pub.publish(reached_goal_msg)
        # 3rd point to ctronl
        
        # with self.lock:
        #     for i, (img, pose) in enumerate(batch):
        #         # breakpoint()
        #         self.get_logger().info(
        #             f"Frame {i+1}: Image stamp: {img.shape}, "
        #             f"Pose position: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}"
        #         )


class DictNamespace(argparse.Namespace):
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if isinstance(value, dict):
                setattr(self, key, DictNamespace(**value))
            else:
                setattr(self, key, value)


def parse_args():
    parser = argparse.ArgumentParser(description='Test UrbanNav model')
    parser.add_argument('--config', type=str, default='config/default.yaml', help='Path to config file')
    parser.add_argument('--checkpoint', type=str, default=None, help='Path to model checkpoint. If not provided, the latest checkpoint will be used.')
    parser.add_argument('--save_predictions', action='store_true', help='Whether to save predictions')
    args = parser.parse_args()
    return args


def load_config(config_path):
    with open(config_path, 'r') as f:
        cfg_dict = yaml.safe_load(f)
    cfg = DictNamespace(**cfg_dict)
    return cfg


def find_latest_checkpoint(checkpoint_dir):
    """
    Finds the latest checkpoint in the given directory based on modification time.
    
    Args:
        checkpoint_dir (str): Path to the directory containing checkpoints.
    
    Returns:
        str: Path to the latest checkpoint file.
    
    Raises:
        FileNotFoundError: If no checkpoint files are found in the directory.
    """
    checkpoint_pattern = os.path.join(checkpoint_dir, '*.ckpt')
    checkpoint_files = glob.glob(checkpoint_pattern)
    if not checkpoint_files:
        raise FileNotFoundError(f"No checkpoint files found in directory: {checkpoint_dir}")
    
    # Sort checkpoints by modification time (latest first)
    checkpoint_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
    latest_checkpoint = checkpoint_files[0]
    return latest_checkpoint


def main():
    args = parse_args()
    cfg = load_config(args.config)

    rclpy.init(args=None)
    node = ImagePoseSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # # Create a directory for test results
    # test_dir = os.path.join(cfg.project.result_dir, cfg.project.run_name, 'test')
    # os.makedirs(test_dir, exist_ok=True)

   
    # # Determine the checkpoint path
    # if args.checkpoint:
    #     checkpoint_path = args.checkpoint
    #     if not os.path.isfile(checkpoint_path):
    #         raise FileNotFoundError(f"Checkpoint not found at {checkpoint_path}")
    # else:
    #     # Automatically find the latest checkpoint
    #     checkpoint_dir = os.path.join(cfg.project.result_dir, cfg.project.run_name, 'checkpoints')
    #     if not os.path.isdir(checkpoint_dir):
    #         raise FileNotFoundError(f"Checkpoint directory does not exist: {checkpoint_dir}")
    #     checkpoint_path = find_latest_checkpoint(checkpoint_dir)
    #     print(f"No checkpoint specified. Using the latest checkpoint: {checkpoint_path}")

    # # Load the model from the checkpoint
    # if cfg.model.type == 'citywalker':
    #     model = CityWalkerModule.load_from_checkpoint(checkpoint_path, cfg=cfg)
    # elif cfg.model.type == 'citywalker_feat':
    #     model = CityWalkerFeatModule.load_from_checkpoint(checkpoint_path, cfg=cfg)
    # else:
    #     raise ValueError(f"Invalid model: {cfg.model.type}")
    

    # model.result_dir = test_dir
    # print(f"Loaded model from checkpoint: {checkpoint_path}")

    # device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    # model = model.to(device)
    # model.eval()

    # video_frames = torch.rand([1, 5, 3, 360, 640])  # 或 torch.randn
    # input_positions = torch.rand([1, 6, 2])        # 可能需要调整数量
    # video_frames = video_frames.to(device)
    # input_positions = input_positions.to(device)
    # wp_pred, arrive_pred ,_, _  = model(video_frames, input_positions, future_obs=None) # this is we want
    # breakpoint()


if __name__ == '__main__':
    target_xy = [3, 3]
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

    args = parse_args()
    cfg = load_config(args.config)

        # Create a directory for test results
    test_dir = os.path.join(cfg.project.result_dir, cfg.project.run_name, 'test')
    os.makedirs(test_dir, exist_ok=True)

   
    # Determine the checkpoint path
    if args.checkpoint:
        checkpoint_path = args.checkpoint
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(f"Checkpoint not found at {checkpoint_path}")
    else:
        # Automatically find the latest checkpoint
        checkpoint_dir = os.path.join(cfg.project.result_dir, cfg.project.run_name, 'checkpoints')
        if not os.path.isdir(checkpoint_dir):
            raise FileNotFoundError(f"Checkpoint directory does not exist: {checkpoint_dir}")
        checkpoint_path = find_latest_checkpoint(checkpoint_dir)
        print(f"No checkpoint specified. Using the latest checkpoint: {checkpoint_path}")

    # Load the model from the checkpoint
    if cfg.model.type == 'citywalker':
        model = CityWalkerModule.load_from_checkpoint(checkpoint_path, cfg=cfg)
    elif cfg.model.type == 'citywalker_feat':
        model = CityWalkerFeatModule.load_from_checkpoint(checkpoint_path, cfg=cfg)
    else:
        raise ValueError(f"Invalid model: {cfg.model.type}")
    

    model.result_dir = test_dir
    print(f"Loaded model from checkpoint: {checkpoint_path}")

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    model = model.to(device)
    model.eval()

    # ros
    rclpy.init(args=None)
    node = ImagePoseSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # main()
