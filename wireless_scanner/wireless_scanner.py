import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
from bleak import BleakScanner
import asyncio
import json

PUBLISHING_TIMER = 10

class WifiBluetoothScannerNode(Node):
    def __init__(self):
        super().__init__('wireless_scanner')
        self.publisher = self.create_publisher(String, 'wireless_data', PUBLISHING_TIMER)
        self.timer = self.create_timer(PUBLISHING_TIMER, self.scan_wifi_bluetooth)  # Scans every 10 seconds

    def scan_wifi_bluetooth(self):
        wifi_data = self.scan_wifi()
        
        # Run the async Bluetooth scan in the event loop
        bluetooth_list = asyncio.run(self.scan_bluetooth(wifi_data))
        
        #Send the data
        self.return_data(wifi_data, bluetooth_list)

    def scan_wifi(self):
        try:
            
            self.get_logger().info("Starting Wi-Fi scan...")
        
            # Run nmcli to scan Wi-Fi
            result = subprocess.run(['nmcli', '-t', '-f', 'SIGNAL,BSSID,SSID', 'dev', 'wifi'], stdout=subprocess.PIPE)
            wifi_info = result.stdout.decode().strip()
            wifi_list = []
            
            for line in wifi_info.split('\n'):
                if line:
                    clean_line = line.replace("\\", "")
                    signal = clean_line[:2]
                    signal = int(signal)
                    bssid = clean_line[3:20]
                    ssid = line[26:]
                    #self.get_logger().info(f" SSID: {ssid},bssid: {bssid}, signal: {signal}")
                    wifi_list.append({'SSID': ssid,
                                      'BSSID': bssid,
                                      'SIGNAL': signal})
            
            
            
            # Sort the Wi-Fi list by signal strength in descending order and keep the top 10
            wifi_list = sorted(wifi_list, key=lambda x: x['SIGNAL'], reverse=True)[:10]
            
            self.get_logger().info("Wi-Fi scan completed")
            
            return wifi_list
        except Exception as e:
            self.get_logger().error(f"Error scanning Wi-Fi: {str(e)}")
            return []

    async def scan_bluetooth(self, wifi_data):
        try:
            self.get_logger().info("Starting BLE scan...")

            # Use BleakScanner to scan for BLE devices
            scanner = BleakScanner()
            devices =  await scanner.discover(timeout=8, return_adv=True)

            bluetooth_list = []
            if not devices:
                self.get_logger().warn("No BLE devices found")
            else:
                # Format the discovered BLE devices into a list of dictionaries
                for device, adv in devices.values():
                    #advertisement_data = device.metadata.get("manufacturer_data", {})
                    bluetooth_list.append({
                        'name': device.name,
                        'address': device.address,
                        #'rssi': device.rssi
                        'rssi' : adv.rssi
                    })
            
            # Sort the Bluetooth list by RSSI in descending order and keep the top 10
            bluetooth_list = sorted(bluetooth_list, key=lambda x: x['rssi'], reverse=True)[:10]

            self.get_logger().info("BLE scan completed")
            
            return bluetooth_list
        except Exception as e:
            self.get_logger().error(f"Error scanning BLE: {str(e)}")

    def return_data(self, wifi_list, bluetooth_list):
        
        
            # Combine both Wi-Fi and Bluetooth data into a dictionary
            data = {
                'wifi': wifi_list,
                'bluetooth': bluetooth_list
            }

            # Convert the dictionary to a JSON string
            message = json.dumps(data)
            self.publisher.publish(String(data=message))
            self.get_logger().info(f"Publishing Wi-Fi and Bluetooth data:\n{message}")






def main(args=None):
    rclpy.init(args=args)
    node = WifiBluetoothScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
