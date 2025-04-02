import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class DogController(Node):
    def __init__(self, network_interface):
        super().__init__('dog_controller')  
        self.sport_client = SportClient()  
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.sport_client.RecoveryStand()

        self.subscription = self.create_subscription(
            Twist,  
            'go2_high_level',  
            self.twist_callback, 
            10  
        )
        self.subscription  # 防止未使用的变量警告

    def twist_callback(self, msg):
        """
        Twist 消息的回调函数，根据消息内容控制机器狗移动。
        """
        linear_x = msg.linear.x  
        angular_z = msg.angular.z  

        # 连续控制
        self.sport_client.Move(linear_x, 0, angular_z)


def main(args=None):
    rclpy.init(args=args) 

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    network_interface = sys.argv[1]  
    ChannelFactoryInitialize(0, network_interface)

    dog_controller = DogController(network_interface)

    try:
        rclpy.spin(dog_controller)  
    except KeyboardInterrupt:
        pass 

    dog_controller.destroy_node()  
    rclpy.shutdown()  

if __name__ == "__main__":
    main()