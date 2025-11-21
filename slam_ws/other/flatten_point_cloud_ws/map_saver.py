#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map2d',
            self.map_callback,
            10)
        self.get_logger().info('Map saver started. Waiting for /map2d...')
        self.map_count = 0
        
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        
        # Convert to numpy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Convert: -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
        img_data = np.zeros((height, width), dtype=np.uint8)
        img_data[data == -1] = 128  # Unknown -> gray
        img_data[data == 0] = 255   # Free -> white
        img_data[data == 100] = 0   # Occupied -> black
        
        # Flip vertically (ROS uses bottom-left origin)
        img_data = np.flipud(img_data)
        
        # Save as JPEG
        img = Image.fromarray(img_data, mode='L')
        filename = f'map_{self.map_count:04d}.jpg'
        img.save(filename, 'JPEG', quality=95)
        
        self.get_logger().info(f'Saved {filename} ({width}x{height})')
        self.map_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()