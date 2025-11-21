#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

class ImuTimeSync(Node):
    def __init__(self):
        super().__init__('imu_time_sync')
        
        self.latest_lidar_time = None
        
        # QoS for LiDAR - match the publisher's QoS (BEST_EFFORT)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for IMU - RELIABLE
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to lidar to get its timestamp
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/rslidar_points', self.lidar_callback, lidar_qos)
        
        # Subscribe to original IMU
        self.imu_sub = self.create_subscription(
            Imu, '/utlidar/imu', self.imu_callback, imu_qos)
        
        # Publish synced IMU
        self.imu_pub = self.create_publisher(Imu, '/utlidar/imu_synced', imu_qos)
        
        self.get_logger().info('IMU Time Sync started - syncing to LiDAR time')
        
    def lidar_callback(self, msg):
        # Store the latest lidar timestamp
        self.latest_lidar_time = msg.header.stamp
        self.get_logger().info('Received LiDAR data', throttle_duration_sec=5.0)
        
    def imu_callback(self, msg):
        if self.latest_lidar_time is None:
            self.get_logger().warn('Waiting for LiDAR data...', throttle_duration_sec=5.0)
            return
        
        # Use the LiDAR's timestamp (they should be very close in real-time)
        msg.header.stamp = self.latest_lidar_time
        self.imu_pub.publish(msg)

def main():
    rclpy.init()
    node = ImuTimeSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()