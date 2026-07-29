import serial
import time
import csv
from datetime import datetime

# --- CONFIGURATION ---
PORT = 'COM3'         # Change to '/dev/ttyUSB0' if running on your Pi
BAUD_RATE = 115200
CSV_FILE_NAME = "cnc_raster_data.csv"
# ---------------------

def main():
    print(f"Connecting to AlphaLab USSVM2 on {PORT}...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Let the connection stabilize
        
        # Open CSV file for writing
        with open(CSV_FILE_NAME, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            # Write our data headers
            writer.writerow(["Timestamp", "Raw_Data"])
            print(f"Created CSV log: {CSV_FILE_NAME}")
            
            print("\nInitializing real-time communication sequence...")
            
            # 1. Send the standard 6-byte query packet to initiate data stream
            # 0x01 tells the device to begin sending its structured information blocks
            init_packet = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
            ser.reset_input_buffer()
            ser.write(init_packet)
            
            print("Streaming active. Press Ctrl+C to stop logging.\n")
            
            while True:
                if ser.in_waiting > 0:
                    # Read the incoming chunk
                    raw_bytes = ser.readline()
                    
                    # Clean and decode the transmission text
                    decoded_line = raw_bytes.decode('utf-8', errors='ignore').strip()
                    
                    if decoded_line:
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        print(f"[{timestamp}] -> {decoded_line}")
                        
                        # Write immediately to the CSV file
                        writer.writerow([timestamp, decoded_line])
                        file.flush()  # Force write to disk so data isn't lost if interrupted
                        
                    # 2. AlphaLab Protocol Safeguard:
                    # Send a 1-byte Acknowledge back to the meter to keep the stream flowing
                    # and prevent its internal state machine from crashing/freezing.
                    ser.write(bytes([0x06])) 
                    
                time.sleep(0.05)  # Quick cycle pause to match the meter's clock rate

    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    except Exception as e:
        print(f"\n[!] Serial Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port cleanly closed. No hardware lockup.")
        print("Data logging complete.")

if __name__ == "__main__":
    main()