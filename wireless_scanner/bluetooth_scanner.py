from bleak import BleakScanner

class BluetoothScanner:
    """
    Class responsible for Bluetooth scanning operations.
    Uses BleakScanner for Bluetooth Low Energy (BLE) scanning.
    """
    
    def __init__(self, logger, scan_timeout=8, max_devices=10):
        self.logger = logger
        self.scan_timeout = scan_timeout
        self.max_devices = max_devices
        self.logger.info("Bluetooth Scanner initialized")
    
    async def scan_bluetooth(self):
        """
        Scan for Bluetooth devices.
        
        Returns:
            list: List of Bluetooth devices with address as dictionary keys
        """
        try:
            self.logger.info("Starting BLE scan...")
            scanner = BleakScanner()
            devices = await scanner.discover(timeout=self.scan_timeout, return_adv=True)
            bluetooth_list = self.parse_bluetooth_info(devices)
            self.logger.info("BLE scan completed")
            return bluetooth_list
        except Exception as e:
            self.logger.error(f"Error scanning BLE: {str(e)}")
            return []
    
    def parse_bluetooth_info(self, devices):
        """
        Parse the discovered Bluetooth devices.
        
        Args:
            devices: Discovered Bluetooth devices from BleakScanner
            
        Returns:
            list: List of Bluetooth devices with address as dictionary keys
        """
        bluetooth_dict = {}  # Use a dictionary instead of a list to avoid duplicates
        if not devices:
            self.logger.warn("No BLE devices found")
        else:
            for device, adv in devices.values():
                address = device.address or ''
                if not address:  # Skip devices without an address
                    continue
                    
                # Use address as the key
                bluetooth_dict[address] = {
                    'name': device.name or '',  # Ensure name is not None
                    'address': address,
                    'rssi': adv.rssi or 0  # Ensure rssi is not None
                }
        
        # Convert to list for sorting
        bluetooth_list = list(bluetooth_dict.values())
        return sorted(bluetooth_list, key=lambda x: x['rssi'], reverse=True)[:self.max_devices]
