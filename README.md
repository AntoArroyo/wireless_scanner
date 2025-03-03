# Wireless Scanner ROS2 Package

This ROS2 package contains a node that scans for both Wi-Fi and Bluetooth devices and publishes the results to a ROS2 topic.

## Features

- Scans for Wi-Fi networks using `nmcli`.
- Scans for Bluetooth Low Energy (BLE) devices using the `BleakScanner`.
- Publishes the scanned data as a JSON string to the `wireless_data` topic every 10 seconds.

## Requirements

- ROS2 (Foxy, Galactic, Humble, or Rolling)
- Python 3.7+
- `nmcli` (NetworkManager Command Line Interface)
- `bleak` Python package

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
    cd ~/ros2_ws
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
