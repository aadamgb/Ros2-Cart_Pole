import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Import JointState for the /joint_states topic
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Wrench
from ros_gz_interfaces.msg import EntityWrench, Entity  # ✅ correct import
import time
from std_msgs.msg import Float32


class CartController(Node):

    def __init__(self):
        super().__init__('cart_controller')
        self.declare_parameter('target_x', 20.0)
        self.target_x = self.get_parameter('target_x').value

        # PID parameters
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 3.5

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.current_x = 0.0

        # Subscribe to odometry
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Create a publisher for the EntityWrench message
        self.publisher_ = self.create_publisher(EntityWrench, '/world/empty/wrench', 10)

        # Plot Juggler Stuff
        self.position_pub = self.create_publisher(Float32, 'cart_position', 10)
        self.error_pub = self.create_publisher(Float32, 'cart_error', 10)
        self.force_pub = self.create_publisher(Float32, 'cart_force', 10)

        # Timer for control loop

        self.timer = self.create_timer(0.05, self.control_loop)

    def joint_state_callback(self, msg):
        self.current_x = msg.position[0]  # Get the position of the first joint
        self.get_logger().info(f"x (from joint state): {self.current_x:.2f}")

    def control_loop(self):
        now = time.time()
        dt = now - self.prev_time
        if dt == 0.0:
            return  # Avoid division by zero
    
        error = self.target_x - self.current_x
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        force_x = self.Kp * error + self.Ki * self.integral + self.Kd * derivative # Proportional control

        # Clamp the force
        force_x = max(min(force_x, 50.0), -50.0)


        # Create and populate the EntityWrench message
        msg = EntityWrench()
        msg.entity.name = "simple_box"  # or your robot model name
        msg.entity.type = Entity.MODEL

        msg.wrench.force.x = force_x
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0

        self.publisher_.publish(msg)
        # Publish values
        self.position_pub.publish(Float32(data=self.current_x))
        self.error_pub.publish(Float32(data=error))
        self.force_pub.publish(Float32(data=force_x))


        self.prev_error = error
        self.prev_time = now
        # self.get_logger().info(f"Applied force: {force_x:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CartController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
