import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import json
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .wifi_scanner import WiFiScanner
from .bluetooth_scanner import BluetoothScanner
from .data_handler import DataHandler

# Configuration constants
WIFI_SCAN_METHOD = 'iw'  # Choose between 'nmcli' and 'iw'
WIFI_TOP_N = 10
BLUETOOTH_SCAN_TIMEOUT = 8
SCAN_DISTANCE_THRESHOLD = 5.0  # Scan every 5 meters
PUBLISHING_TIMER = 30.0  # Fallback timer for scanning (seconds)

class WirelessScannerNode(Node):
    """
    Main node for wireless scanning.
    Coordinates WiFi and Bluetooth scanning based on robot movement.
    """
    
    def __init__(self):
        super().__init__('wireless_scanner')
        
        # Initialize publishers
        self.publisher = self.create_publisher(String, 'wireless_data', 10)
        
        # Initialize position and orientation variables
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        
        # Initialize last scan position
        self.last_scan_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Initialize scanners and data handler
        self.wifi_scanner = WiFiScanner(self.get_logger(), wifi_top_n=WIFI_TOP_N)
        self.bluetooth_scanner = BluetoothScanner(self.get_logger(), 
                                                scan_timeout=BLUETOOTH_SCAN_TIMEOUT,
                                                max_devices=WIFI_TOP_N)
        self.data_handler = DataHandler(self.get_logger())
        
        # Try to load last position from XML file
        last_position, last_orientation = self.data_handler.load_last_position('wireless_data.xml')
        if last_position:
            self.last_scan_position = last_position
            self.get_logger().info(f"Loaded last scan position: x={last_position['x']}, y={last_position['y']}, z={last_position['z']}")
        
        # Subscribe to odometry and IMU topics
        self.odom_subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu, 
            '/imu', 
            self.imu_callback, 
            10
        )
        
        # Create a timer as a fallback in case odometry is not available
        self.timer = self.create_timer(PUBLISHING_TIMER, self.timer_callback)
        
        self.get_logger().info("Wireless Scanner Node initialized and subscriptions created.")
        self.get_logger().info(f"Will scan every {SCAN_DISTANCE_THRESHOLD} meters of robot movement")
    
    def odom_callback(self, msg):
        """
        Callback for odometry messages.
        Updates position and triggers scanning if the robot has moved enough.
        
        Args:
            msg: Odometry message
        """
        # Update current position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        self.position['z'] = msg.pose.pose.position.z
        
        # Calculate distance moved since last scan
        distance = self.calculate_distance(self.position, self.last_scan_position)
        
        # Log position at debug level to avoid flooding logs
        self.get_logger().debug(f"Robot Position: x={self.position['x']}, y={self.position['y']}, z={self.position['z']}")
        self.get_logger().debug(f"Distance since last scan: {distance:.2f} meters")
        
        # If moved enough, perform a scan
        if distance >= SCAN_DISTANCE_THRESHOLD:
            self.get_logger().info(f"Robot moved {distance:.2f} meters, triggering scan")
            self.perform_scan()
            # Update last scan position
            self.last_scan_position = self.position.copy()
    
    def imu_callback(self, msg):
        """
        Callback for IMU messages.
        Updates orientation data.
        
        Args:
            msg: IMU message
        """
        self.orientation['x'] = msg.orientation.x
        self.orientation['y'] = msg.orientation.y
        self.orientation['z'] = msg.orientation.z
        self.orientation['w'] = msg.orientation.w
        self.get_logger().debug(f"IMU Orientation: x={self.orientation['x']}, y={self.orientation['y']}, z={self.orientation['z']}, w={self.orientation['w']}")
    
    def timer_callback(self):
        """
        Fallback timer callback.
        Performs a scan periodically in case odometry is not available.
        """
        self.get_logger().info("Timer-triggered scan (fallback)")
        self.perform_scan()
    
    def perform_scan(self):
        """
        Perform WiFi and Bluetooth scanning and save the data.
        """
        self.get_logger().info("Starting wireless scan...")
        
        # Scan WiFi
        wifi_data = self.wifi_scanner.scan_wifi(method=WIFI_SCAN_METHOD)
        
        # Scan Bluetooth
        bluetooth_list = asyncio.run(self.bluetooth_scanner.scan_bluetooth())
        
        # Publish data
        self.publish_data(wifi_data, bluetooth_list)
        
        # Save data to XML
        self.data_handler.save_data_to_xml(
            self.position,
            self.orientation,
            wifi_data,
            bluetooth_list,
            'wireless_data.xml'
        )
        
        self.get_logger().info("Wireless scan completed")
    
    def publish_data(self, wifi_list, bluetooth_list):
        """
        Publish wireless data to ROS topic.
        
        Args:
            wifi_list (list): List of WiFi networks
            bluetooth_list (list): List of Bluetooth devices
        """
        data = {
            'position': self.position,
            'orientation': self.orientation,
            'wifi': wifi_list,
            'bluetooth': bluetooth_list
        }
        message = json.dumps(data)
        self.publisher.publish(String(data=message))
        self.get_logger().info("Published wireless data to topic")
    
    def calculate_distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two positions.
        
        Args:
            pos1 (dict): First position with x, y, z coordinates
            pos2 (dict): Second position with x, y, z coordinates
            
        Returns:
            float: Distance in meters
        """
        return math.sqrt(
            (pos1['x'] - pos2['x'])**2 +
            (pos1['y'] - pos2['y'])**2 +
            (pos1['z'] - pos2['z'])**2
        )

def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    node = WirelessScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
