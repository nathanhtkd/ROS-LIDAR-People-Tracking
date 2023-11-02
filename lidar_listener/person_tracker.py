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
    self.publisher = self.create_publisher(
      Int64, '/person_count', 10
    )
    self.unique_people_count = 0
    self.get_logger().info('Person Count Node created')

  def publish_person_count(self):
    # Create a new Int64 message
    msg = Int64()
    # Set the data to be the number of unique people observed
    msg.data = self.unique_people_count
    # Publish the message
    self.publisher.publish(msg)
    # self.get_logger().info(f'Publishing: {msg.data}')

  def process_sensor_data(self, sensor_data):
    # TODO: Implement person detection and tracking logic here
    # Update self.unique_people_count accordingly
    pass

  # TODO: Define the callback method for the sensor data subscription
  def listener_callback(self, msg):
    self.process_sensor_data(msg)
    self.publish_person_count()

def main(args=None):
  rclpy.init(args=args)
  node = PersonCount()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
