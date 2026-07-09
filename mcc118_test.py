from daqhats import mcc118, OptionFlags, HatError

# --- Configuration ---
CHANNEL = 1
CHANNEL_MASK = 1 << CHANNEL   # Bitmask for Channel 0
SAMPLE_RATE = 100000.0        # 100k Samples per second
SAMPLES_PER_READ = 1000       # 1000 samples / 100kHz = 10ms (100Hz output)
VOLTAGE_MULTIPLIER = 10000 
# ---------------------

def main():
    print("Initializing MCC 118 for Hardware-Paced Oversampling...")
    try:
        hat = mcc118(0)
        
        options = OptionFlags.CONTINUOUS
        
        # Ask the DAQ what its true hardware clock rate will be for 1 channel
        actual_rate = hat.a_in_scan_actual_rate(1, SAMPLE_RATE)
        
        # Start the continuous hardware scan in the background
        hat.a_in_scan_start(CHANNEL_MASK, SAMPLES_PER_READ, SAMPLE_RATE, options)
        
        print(f"Hardware scan locked at {actual_rate:.0f} Hz.")
        print(f"Averaging {SAMPLES_PER_READ} samples per output (Effective Update Rate: {actual_rate/SAMPLES_PER_READ:.0f} Hz)")
        print("Press Ctrl+C to stop.\n")
        
        while True:
            # Scoop 1000 samples from the buffer
            # This function automatically blocks/waits until exactly 1000 samples are ready,
            # giving us perfectly spaced 100Hz timing.
            read_result = hat.a_in_scan_read(SAMPLES_PER_READ, timeout=0.1)
            
            # Check if the Pi CPU fell behind the DAQ hardware
            if read_result.hardware_overrun or read_result.buffer_overrun:
                print("\n[!] Buffer Overrun! The Pi can't keep up with the data flow.")
                break
            
            samples = read_result.data
            
            if len(samples) > 0:
                # Average the 1000 noisy steps to get our high-resolution sub-step
                avg_voltage = sum(samples) / len(samples)
                actual_reading = avg_voltage * VOLTAGE_MULTIPLIER
                
                # Printing with an extra decimal place to show off the new resolution!
                print(f"Avg Raw: {avg_voltage:+.5f} V  |  Smoothed Charge: {actual_reading:+.1f} V")

    except HatError as e:
        print(f"\n[!] Hardware Error: {e}")
    except KeyboardInterrupt:
        print("\nOversampling stopped safely.")
    finally:
        # We must gracefully stop the hardware scan, or the buffer will overflow in the background
        if 'hat' in locals():
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()

if __name__ == "__main__":
    main()