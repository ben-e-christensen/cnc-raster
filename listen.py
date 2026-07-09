import serial
import csv
from datetime import datetime

PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
CSV_FILE_NAME = "ussvm2_fast_raster.csv"

def make_packet(command_byte):
    return bytes([command_byte] * 6)

def main():
    print(f"Connecting to AlphaLab USSVM2 on {PORT}...")
    try:
        # We use a short 50ms timeout. 
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.05)
        
        with open(CSV_FILE_NAME, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Internal_Timer", "Raw_Reading"])
            print("Blasting at maximum speed... Press Ctrl+C to stop.")
            
            while True:
                # 1. Instantly request data
                ser.write(make_packet(0x03))
                
                # 2. Block and wait for exactly 13 bytes (no artificial sleep delays!)
                packet = ser.read(13)
                
                if len(packet) == 13 and packet[-1] == 0x08:
                    internal_timer = int.from_bytes(packet[4:6], byteorder='big')
                    raw_reading = int.from_bytes(packet[8:12], byteorder='big', signed=True)
                    
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    print(f"[{timestamp}] Timer: {internal_timer:<5} | Reading: {raw_reading}")
                    
                    writer.writerow([timestamp, internal_timer, raw_reading])
                    # Note: We removed file.flush() here because disk writing slows down the loop. 
                    # The OS will bulk-write the CSV automatically.

    except KeyboardInterrupt:
        print("\nLogging stopped.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()