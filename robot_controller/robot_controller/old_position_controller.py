import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Point, Quaternion
from gazebo_msgs.srv import ApplyBodyWrench
from builtin_interfaces.msg import Duration

class CartController(Node):

    def __init__(self):
        super().__init__('cart_controller')
        self.declare_parameter('target_x', 5.0)
        self.target_x = self.get_parameter('target_x').value
        self.current_x = 0.0

        self.body_name = 'cart_link'  # Change if different in URDF

        # Subscribe to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create service client for applying wrench
        self.cli = self.create_client(ApplyBodyWrench, '/world/empty/wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/apply_body_wrench service...')

        # Call control loop periodically
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x

    def control_loop(self):
        error = self.target_x - self.current_x
        force_x = 5.0 * error  # Proportional controller (tune gain as needed)

        wrench = Wrench()
        wrench.force.x = max(min(force_x, 10.0), -10.0)  # Clamp force to avoid instability

        request = ApplyBodyWrench.Request()
        request.body_name = self.body_name
        request.reference_frame = 'world'
        request.reference_point = Point(x=0.0, y=0.0, z=0.0)
        request.wrench = wrench
        request.start_time = self.get_clock().now().to_msg()
        request.duration = Duration(sec=0, nanosec=100_000_000)  # Apply force for 0.1 sec

        self.cli.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = CartController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
