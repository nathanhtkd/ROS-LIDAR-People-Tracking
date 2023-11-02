import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from example_interfaces.msg import Int64

'''
  This class reads in the Clustered Points data and publishes an integer
  describing the number of people found in the lidar scan data
'''

class PersonCount(Node):
  def __init__(self):
    super().__init__('person_count')
    self.subscription = self.create_subscription(
      PointCloud, '/clustered_points', self.listener_callback, 10
    )
    self.publisher_ = self.create_publisher(
      Int64, '/person_count', 10
    )
    self.unique_people_count = 0

def main(args=None):
  rclpy.init(args=args)
  node = PersonCount()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
