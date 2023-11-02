import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point32

# max_clusters = 0

class PointCluster(Node):
  def __init__(self):
    super().__init__('point_cluster')
    self.subscription = self.create_subscription(
      PointCloud, '/lidar_to_point_cloud', self.listener_callback, 10
    )
    self.publisher = self.create_publisher(
      PointCloud, '/clustered_points', 10
    )
    self.get_logger().info('Point Cluster Node created -> subscribing to /lidar_to_point_cloud -> publishing to /clustered_points')

  def publish_point_cloud(self, points, msg):
      # Create a PointCloud message
      cloud = PointCloud()
      # Set the appropriate frame id
      cloud.header = msg.header

      cloud.points = [Point32(x=point[0], y=point[1], z=point[2]) for point in points]

      # Publish the PointCloud message
      self.publisher.publish(cloud)

  def listener_callback(self, msg):
    # global max_clusters
    points_list = []

    for point in msg.points:
      # ensure points are not NaN or inf
      if not np.isinf(point.x) and not np.isinf(point.y) and not np.isinf(point.z) and \
        not np.isnan(point.x) and not np.isnan(point.y) and not np.isnan(point.z):
        points_list.append([point.x, point.y, point.z])

        # DEBUG: print point clouds
        # self.get_logger().info(f'{point.x}, {point.y}, {point.z}')

    if points_list:
      points_np = np.array(points_list, dtype=np.float32)

      # Apply DBSCAN clustering
      clustering = DBSCAN(eps=0.5, min_samples=5).fit(points_np)
      labels = clustering.labels_
      n_clusters = len(set(labels)) - (1 if -1 in labels else 0)  # '-1' represents noise in DBSCAN
      max_clusters = max(max_clusters, n_clusters)

      # Initialize an array to hold the centroid points
      centroids = []

      for label in set(labels):
        if label != -1:  # Skip noise
          # Extract all points belonging to the current cluster
          cluster_points = points_np[labels == label]
          # Compute the centroid of the cluster
          centroid = np.mean(cluster_points, axis=0)
          centroids.append(centroid)
          # DEBUG: check centroid calcluation points
          self.get_logger().info(f'Cluster {label} centroid: {centroid}')

      # Create a new PointCloud message for the centroids
      centroid_cloud = PointCloud()
      centroid_cloud.header = msg.header  # Use the same header as the input message
      for centroid in centroids:
        # Add each centroid to the new PointCloud message
        centroid_cloud.points.append(Point32(
          x=float(centroid[0]),
          y=float(centroid[1]),
          z=float(centroid[2])
        ))

      # DEBUG: check max clusters
      # self.get_logger().info(f'Max Clusters found: {max_clusters}')

      # Publish the centroid PointCloud
      self.publisher.publish(centroid_cloud)
    else:
        self.get_logger().info('No valid points received.')


def main(args=None):
  rclpy.init(args=args)
  node = PointCluster()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == 'main':
  main()
