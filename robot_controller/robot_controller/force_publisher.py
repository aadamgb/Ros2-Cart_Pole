import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench, Entity
from geometry_msgs.msg import Wrench

class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')

        # Publishing to the correct topic that works in CLI
        self.publisher = self.create_publisher(EntityWrench, '/world/empty/wrench/persistent', 10)

        # Setup periodic publishing
        self.timer = self.create_timer(0.1, self.publish_force)

    def publish_force(self):
        msg = EntityWrench()
        msg.entity.name = 'simple_box'
        msg.entity.type = Entity.MODEL
        msg.wrench.force.x = 20.0  # Same value as the CLI test

        self.publisher.publish(msg)
        self.get_logger().info('Published force to simple_box')

def main(args=None):
    rclpy.init(args=args)
    node = ForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
