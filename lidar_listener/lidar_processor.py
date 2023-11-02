import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

# msgCount = 0

'''
  LidarPreprocessor class definition
  This ROS node class is responsible for converting raw laser scan data
  into a 2D point cloud. The class subscribes to a topic that provides
  LaserScan messages and publishes converted PointCloud messages.
  Each LaserScan message is processed in a callback function where the
  scan data is translated into a series of points in a two-dimensional
  plane, assuming a flat ground and a fixed z-coordinate for all points.

'''

class LidarPreprocessor(Node):

    def __init__(self):
        super().__init__('lidar_preprocessor')

        # Subscription and Publication
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(
            PointCloud, '/lidar_to_point_cloud', 10)
        self.get_logger().info('Lidar Preprocessor Node created')

    def lidar_callback(self, msg):
        # global msgCount

        # list to hold points in the point cloud
        points = []

        for i in range(len(msg.ranges)):
            # Compute the x and y coordinates for each point using trigonometry
            x = msg.ranges[i] * \
                math.cos(msg.angle_min + i * msg.angle_increment)
            y = msg.ranges[i] * \
                math.sin(msg.angle_min + i * msg.angle_increment)

            # Create a point with the computed x, y, and fixed z
            point = Point32(x=x, y=y, z=0.0)

            # Add the computed point to the points list
            points.append(point)

        # Create a PointCloud message
        cloud = PointCloud()
        cloud.header = msg.header
        cloud.points = points
        # self.get_logger().info(
        #     f'Received scan and translated! {msgCount} iterations!')
        # Publish the processed msg
        self.publisher.publish(cloud)
        # msgCount += 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
