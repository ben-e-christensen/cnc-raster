import serial
import csv
import time
from datetime import datetime

# --- CONFIGURATION ---
# Windows: 'COM3', 'COM4', etc. | Mac/Linux: '/dev/ttyUSB0'
COM_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200 
CSV_FILENAME = 'charge_map_data.csv'

def main():
    try:
        # Open the connection to the USSVM2
        print(f"Connecting to AlphaLab meter on {COM_PORT}...")
        meter = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Give the port a moment to initialize
        
        # Prepare the CSV file and write the header row
        with open(CSV_FILENAME, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Here is where you add columns for your "other junk"
            writer.writerow(["Timestamp", "X_Coord", "Y_Coord", "Voltage_V", "Temperature", "Notes"])
            
            print("Listening for data... (Press Ctrl+C to stop)")
            
            # Dummy variables for your future CNC integration
            current_x = 120.0
            current_y = 80.0
            temp_junk = 22.5
            
            while True:
                # Read a line of data from the meter
                if meter.in_waiting > 0:
                    # Decode the bytes to a string and strip whitespace/newlines
                    raw_data = meter.readline().decode('utf-8', errors='ignore').strip()
                    
                    if raw_data:
                        # Assuming the meter just spits out numbers like "+0.15" or "150.2"
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        
                        # Write the row to your CSV
                        writer.writerow([timestamp, current_x, current_y, raw_data, temp_junk, "Test Run"])
                        
                        # Force the file to save immediately so data isn't lost if it crashes
                        file.flush() 
                        
                        print(f"[{timestamp}] X: {current_x} | Y: {current_y} | Volts: {raw_data}")
                        
                        # Simulate the CNC moving for the sake of the test
                        current_x += 1.0

    except serial.SerialException as e:
        print(f"\n[!] SERIAL ERROR: Could not connect to {COM_PORT}. Check your cable and port name.")
        print(e)
    except KeyboardInterrupt:
        print("\nLogging stopped by user. CSV file saved.")
    finally:
        if 'meter' in locals() and meter.is_open:
            meter.close()

if __name__ == "__main__":
    main()