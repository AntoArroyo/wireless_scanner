# Wireless Scanner ROS2 Package

This ROS2 package contains a node that scans for both Wi-Fi and Bluetooth devices and publishes the results to a ROS2 topic.

## Features

- Scans for Wi-Fi networks using `nmcli` or `iw` (configurable).
- Scans for Bluetooth Low Energy (BLE) devices using the `BleakScanner`.
- Publishes the scanned data as a JSON string to the `wireless_data` topic.
- **Distance-based scanning**: Performs a scan every X meters of robot movement.
- Fallback timer-based scanning when odometry data is not available.
- Stores scan data in XML format with position and orientation information.
- Uses BSSID as key for WiFi networks and address as key for Bluetooth devices to prevent duplicates.

## Code Structure

The package has been refactored into modular components:

- `scanner.py`: Main node and coordination logic
- `wifi_scanner.py`: WiFi scanning functionality
- `bluetooth_scanner.py`: Bluetooth scanning functionality
- `data_handler.py`: XML data handling and storage

## Requirements

- ROS2 (Foxy, Galactic, Humble, or Rolling)
- Python 3.7+
- `nmcli` and `iw` for WiFi scanning
- `bleak` Python package for Bluetooth scanning

## Installation

1. Ensure you have ROS2 installed on your system. Follow the instructions on the [ROS2 installation page](https://docs.ros.org/en/foxy/Installation.html) if you haven't installed it yet.

2. Install the `bleak` package:
    ```sh
    pip install bleak
    ```

3. Clone this repository into your ROS2 workspace:
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/AntoArroyo/wireless_scanner.git
    ```

4. Build the package:
    ```sh
    cd ~/ros2_ws/wireless_scanner
    colcon build
    ```

5. Source the setup file:
    ```sh
    source install/setup.bash
    ```

## Usage

To run the wireless scanner node, use the following command:

```sh
ros2 run wireless_scanner wireless_scanner
```

## Configuration

You can modify the following parameters in the `scanner.py` file:

- `SCAN_DISTANCE_THRESHOLD`: Distance threshold in meters for triggering a scan (default: 5.0)
- `WIFI_SCAN_METHOD`: Method for scanning WiFi networks ('nmcli' or 'iw', default: 'iw')
- `WIFI_TOP_N`: Maximum number of WiFi networks to include in results (default: 10)
- `BLUETOOTH_SCAN_TIMEOUT`: Timeout for Bluetooth scanning in seconds (default: 8)
- `PUBLISHING_TIMER`: Fallback timer period in seconds (default: 10.0)
- `POSITION_METHOD` : Selects the odometry or acml for position data (default: 'amcl')

## Data Storage

Scan data is stored in XML format in the `data/wireless_data.xml` file. Each scan includes:
- Position (x, y, z)
- Orientation (quaternion x, y, z, w)
- WiFi networks (SSID, BSSID, signal strength)
- Bluetooth devices (name, address, RSSI)
- Timestamp
