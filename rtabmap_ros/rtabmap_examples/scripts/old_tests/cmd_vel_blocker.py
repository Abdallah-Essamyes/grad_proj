import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Blocker(Node):
    def __init__(self):
        super().__init__('cmd_vel_blocker')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
    def cb(self,msg):
        # drop
        pass

def main():
    rclpy.init()
    n=Blocker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
