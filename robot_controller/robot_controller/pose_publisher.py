import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        # Create a publisher that will publish to the /pose topic
        self.publisher = self.create_publisher(Pose, '/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # Publish every 0.1 seconds

    def publish_pose(self):
        pose_msg = Pose()

        # Set the position of the pose (x, y, z)
        pose_msg.position.x = 1.0  # Example position
        pose_msg.position.y = 2.0
        pose_msg.position.z = 0.0

        # Set the orientation of the pose (x, y, z, w quaternion)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0  # No rotation (identity quaternion)

        # Publish the pose message
        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Publishing pose: {pose_msg}')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
