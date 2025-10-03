import rclpy
import asyncio
import json
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from .wifi_scanner import WiFiScanner
from .bluetooth_scanner import BluetoothScanner
from .data_handler import DataHandler

# Configuration constants
WIFI_SCAN_METHOD = 'iw'  # Choose between 'nmcli' and 'iw'
DEVICE_TOP_N = 15
BLUETOOTH_SCAN_TIMEOUT = 8
SCAN_DISTANCE_THRESHOLD = 1.0  # Scan every 1 meters
PUBLISHING_TIMER = 5.0  # Fallback timer for scanning (seconds)
POSITION_METHOD = "AMCL" # Change it to ODOM for Odometry as position

class WirelessScannerNode(Node):
    """
    Main node for wireless scanning.
    Coordinates WiFi and Bluetooth scanning based on robot movement.
    """
    
    def __init__(self):
        super().__init__('wireless_scanner')
        
        # Initialize publishers
        #self.publisher = self.create_publisher(String, 'wireless_data', 10)
        
        # Initialize position and orientation variables
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        self.transform = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        
        # Initialize last scan position
        self.last_scan_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Initialize scanners and data handler
        self.wifi_scanner = WiFiScanner(self.get_logger(), wifi_top_n=DEVICE_TOP_N)
        self.bluetooth_scanner = BluetoothScanner(self.get_logger(), 
                                                scan_timeout=BLUETOOTH_SCAN_TIMEOUT,
                                                max_devices=DEVICE_TOP_N)
        self.data_handler = DataHandler(self.get_logger())
        
        # Try to load last position from XML file
        """ last_position, last_orientation = self.data_handler.load_last_position('wireless_data.xml')
        if last_position:
            self.last_scan_position = last_position
            self.get_logger().info(f"Loaded last scan position: x={last_position['x']}, y={last_position['y']}, z={last_position['z']}")
         
            """
        
        # Subscribe to odometry or AMCL based on the argument
        if POSITION_METHOD == "ODOM":
            self.odom_subscription = self.create_subscription(
                Odometry, 
                '/odom', 
                self.odom_callback, 
                10
            )
            self.get_logger().info("Using Odometry for position updates.")
        else:
            self.amcl_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_pose_callback,
                10
            )
            self.get_logger().info("Using AMCL for position updates.")
        
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a timer as a fallback
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
        
        self.orientation['x'] = msg.pose.pose.orientation.x
        self.orientation['y'] = msg.pose.pose.orientation.y
        self.orientation['z'] = msg.pose.pose.orientation.z
        self.orientation['w'] = msg.pose.pose.orientation.w
        
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
                    
    def tf_callback(self, msg):
        """
        Callback for TF messages.
        Updates transform data.
        
        Args:
            msg: TF message   
        """
        
        # Assuming the first transform is the one we are interested in
        if msg.transforms:
            transform = msg.transforms[0]
            self.transform['x'] = transform.transform.translation.x
            self.transform['y'] = transform.transform.translation.y
            self.transform['z'] = transform.transform.translation.z
            self.get_logger().debug(f"TF Transform: x={self.transform['x']}, y={self.transform['y']}, z={self.transform['z']}")
    
    def amcl_pose_callback(self, msg):
        """
        Callback for AMCL pose messages.
        Updates position and triggers scanning if the robot has moved enough.
        
        Args:
            msg: PoseWithCovarianceStamped message
        """
        
        # Update current position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        self.position['z'] = msg.pose.pose.position.z
        
        self.orientation['x'] = msg.pose.pose.orientation.x
        self.orientation['y'] = msg.pose.pose.orientation.y
        self.orientation['z'] = msg.pose.pose.orientation.z
        self.orientation['w'] = msg.pose.pose.orientation.w
        
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
        self.get_logger().info(f"Performing SCAN at point --> ({self.position['x']} --  {self.position['y']})")
        self.get_logger().info("Starting wireless scan...")
        
        # Scan WiFi
        wifi_data = self.wifi_scanner.scan_wifi(method=WIFI_SCAN_METHOD)
        
        self.get_logger().info(f"wifi_data: {wifi_data}")
        
        # Remove device connected to avoid false positives
       # wifi_data = [wifi for wifi in wifi_data if wifi['SSID'] != 'RedmiAnto']
        
        # Scan Bluetooth
        bluetooth_list = asyncio.run(self.bluetooth_scanner.scan_bluetooth())
        
        # Publish data
        self.publish_data(wifi_data, bluetooth_list)
        
        # Save data to XML
        self.data_handler.save_data_to_xml(
            self.position,
            self.orientation,
            self.transform,
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
            'transform': self.transform,
            'wifi': wifi_list,
            'bluetooth': bluetooth_list
        }
        message = json.dumps(data)
        #self.publisher.publish(String(data=message))
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