import requests
import json

class KlipperController:
    def __init__(self, ip_address, port=7125):
        """
        Initialize the connection to Moonraker.
        :param ip_address: The IP address of your Klipper/Moonraker host (e.g., '192.168.1.100')
        :param port: The port Moonraker is running on (default is 7125)
        """
        self.base_url = f"http://{ip_address}:{port}"

    def check_connection(self):
        """Check if Moonraker is online and responding."""
        try:
            response = requests.get(f"{self.base_url}/printer/info", timeout=5)
            if response.status_code == 200:
                print("Successfully connected to Klipper/Moonraker!")
                return True
            else:
                print(f"Failed to connect. Status code: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"Error connecting to printer: {e}")
            return False

    def send_gcode(self, gcode_cmd):
        """
        Send a G-code command to Klipper.
        :param gcode_cmd: The G-code string (e.g., 'G28' or 'M114')
        """
        url = f"{self.base_url}/printer/gcode/script"
        payload = {"script": gcode_cmd}
        
        try:
            print(f"Sending command: {gcode_cmd}")
            response = requests.post(url, json=payload, timeout=10)
            
            if response.status_code == 200:
                print("Command executed successfully.")
                # Moonraker returns JSON, we can parse it to see what Klipper said
                result = response.json()
                return result
            else:
                print(f"Error executing command. Status code: {response.status_code}")
                print("Response:", response.text)
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"Error sending G-code: {e}")
            return None

# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    # REPLACE WITH YOUR PRINTER'S IP ADDRESS OR HOSTNAME (e.g., 'mainsailos.local')
    PRINTER_IP = "127.0.0.1" 
    
    klipper = KlipperController(PRINTER_IP)
    
    if klipper.check_connection():
        # Example 1: Get current position
        # M114 asks the printer for its current coordinates
        klipper.send_gcode("M114")
        
        # Example 2: Home all axes 
        # (Uncomment the line below to actually home the printer)
        klipper.send_gcode("G28")
        
        # Example 3: Send a terminal message
        klipper.send_gcode("M117 Hello from Python!")   