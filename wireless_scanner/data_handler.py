import xml.etree.ElementTree as ET
import os

class DataHandler:
    """
    Class responsible for handling data storage and retrieval.
    Manages XML file operations for position, orientation, WiFi, and Bluetooth data.
    """
    
    def __init__(self, logger, data_dir='./data'):
        self.logger = logger
        self.data_dir = data_dir
        
        # Ensure data directory exists
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.logger.info(f"Data Handler initialized with directory: {self.data_dir}")
    
    def save_data_to_xml(self, position, orientation, wifi_list, bluetooth_list, filename):
        """
        Save position, orientation, WiFi, and Bluetooth data to XML file.
        
        Args:
            position (dict): Position data with x, y, z coordinates
            orientation (dict): Orientation data with x, y, z, w quaternion
            wifi_list (list): List of WiFi networks
            bluetooth_list (list): List of Bluetooth devices
            filename (str): Name of the XML file to save to
        """
        file_path = os.path.join(self.data_dir, filename)
        
        # Create root element if file doesn't exist, otherwise load existing XML
        if os.path.exists(file_path):
            try:
                tree = ET.parse(file_path)
                root = tree.getroot()
                
                # If root is not a "Positions" element, create a new structure
                if root.tag != "Positions":
                    new_root = ET.Element("Positions")
                    # Add the existing position as a child if it was in the old format
                    if root.tag == "Position":
                        new_root.append(root)
                    root = new_root
                    tree = ET.ElementTree(root)
            except ET.ParseError:
                # If file exists but is not valid XML, create new structure
                self.logger.warn(f"Existing XML file {file_path} is not valid. Creating new file.")
                root = ET.Element("Positions")
                tree = ET.ElementTree(root)
        else:
            # Create new XML structure
            root = ET.Element("Positions")
            tree = ET.ElementTree(root)
            
        # Create a new position entry
        position_entry = ET.SubElement(root, "Position")
        
        # Add position data
        ET.SubElement(position_entry, "X").text = str(position['x'])
        ET.SubElement(position_entry, "Y").text = str(position['y'])
        ET.SubElement(position_entry, "Z").text = str(position['z'])
        
        # Add orientation data
        self.add_orientation_to_entry(position_entry, orientation)
        
        # Add WiFi data
        self.add_wifi_to_entry(position_entry, wifi_list)
        
        # Add Bluetooth data
        self.add_bluetooth_to_entry(position_entry, bluetooth_list)
        
        # Add timestamp
        timestamp = ET.SubElement(position_entry, "Timestamp")
        timestamp.text = str(self._get_current_time())
        
        # Write to file
        tree.write(file_path)
        
        self.logger.info(f"Data appended to XML file: {file_path}")
    
    def add_orientation_to_entry(self, position_entry, orientation):
        """
        Add orientation data to a position entry.
        
        Args:
            position_entry: XML element for position
            orientation (dict): Orientation data with x, y, z, w quaternion
        """
        orientation_element = ET.SubElement(position_entry, "Orientation")
        ET.SubElement(orientation_element, "X").text = str(orientation['x'])
        ET.SubElement(orientation_element, "Y").text = str(orientation['y'])
        ET.SubElement(orientation_element, "Z").text = str(orientation['z'])
        ET.SubElement(orientation_element, "W").text = str(orientation['w'])
    
    def add_wifi_to_entry(self, position_entry, wifi_list):
        """
        Add WiFi data to a position entry.
        
        Args:
            position_entry: XML element for position
            wifi_list (list): List of WiFi networks
        """
        wifi = ET.SubElement(position_entry, "WiFi")
        for network in wifi_list:
            network_element = ET.SubElement(wifi, "Network")
            # Use .get() method to provide default values if keys are missing
            ET.SubElement(network_element, "SSID").text = network.get('SSID', '')
            ET.SubElement(network_element, "BSSID").text = network.get('BSSID', '')
            ET.SubElement(network_element, "SIGNAL").text = str(network.get('SIGNAL', 0))
    
    def add_bluetooth_to_entry(self, position_entry, bluetooth_list):
        """
        Add Bluetooth data to a position entry.
        
        Args:
            position_entry: XML element for position
            bluetooth_list (list): List of Bluetooth devices
        """
        bluetooth = ET.SubElement(position_entry, "Bluetooth")
        for device in bluetooth_list:
            device_element = ET.SubElement(bluetooth, "Device")
            # Use .get() method to provide default values if keys are missing
            ET.SubElement(device_element, "Name").text = device.get('name', '')
            ET.SubElement(device_element, "Address").text = device.get('address', '')
            ET.SubElement(device_element, "RSSI").text = str(device.get('rssi', 0))
    
    def _get_current_time(self):
        """
        Get current time in seconds.
        This is a placeholder - in the actual implementation, 
        this would use ROS2's clock.
        
        Returns:
            int: Current time in seconds
        """
        import time
        return int(time.time())
    
    def load_last_position(self, filename):
        """
        Load the last recorded position from the XML file.
        
        Args:
            filename (str): Name of the XML file to load from
            
        Returns:
            tuple: (position, orientation) dictionaries or (None, None) if not found
        """
        file_path = os.path.join(self.data_dir, filename)
        
        if not os.path.exists(file_path):
            self.logger.warn(f"File {file_path} does not exist, cannot load last position")
            return None, None
        
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Find the last position entry
            if root.tag == "Positions":
                positions = root.findall("Position")
                if positions:
                    last_position = positions[-1]
                else:
                    self.logger.warn("No Position elements found in XML file")
                    return None, None
            elif root.tag == "Position":
                last_position = root
            else:
                self.logger.warn(f"Unexpected root tag: {root.tag}")
                return None, None
            
            # Extract position data
            position = {
                'x': float(last_position.find("X").text) if last_position.find("X") is not None else 0.0,
                'y': float(last_position.find("Y").text) if last_position.find("Y") is not None else 0.0,
                'z': float(last_position.find("Z").text) if last_position.find("Z") is not None else 0.0
            }
            
            # Extract orientation data
            orientation_elem = last_position.find("Orientation")
            if orientation_elem is not None:
                orientation = {
                    'x': float(orientation_elem.find("X").text) if orientation_elem.find("X") is not None else 0.0,
                    'y': float(orientation_elem.find("Y").text) if orientation_elem.find("Y") is not None else 0.0,
                    'z': float(orientation_elem.find("Z").text) if orientation_elem.find("Z") is not None else 0.0,
                    'w': float(orientation_elem.find("W").text) if orientation_elem.find("W") is not None else 0.0
                }
            else:
                orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
            
            return position, orientation
            
        except ET.ParseError as e:
            self.logger.error(f"Error parsing XML file {file_path}: {str(e)}")
            return None, None
        except Exception as e:
            self.logger.error(f"Error loading last position from {file_path}: {str(e)}")
            return None, None
