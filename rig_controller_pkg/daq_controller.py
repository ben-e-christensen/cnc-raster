"""
DAQController - wraps the MCC118 DAQ HAT for reading the electrostatic
probe's charge signal.

CONTINUOUS POLLING MODE: start_continuous() runs the scan on its OWN
dedicated background thread, reading as fast as the hardware paces itself
(~100Hz with the default config) and appending (timestamp, charge) samples
to an internal thread-safe buffer. This is deliberately decoupled from
anything CNC/network-related - the whole point is that a slow or stalled
Moonraker HTTP call can NEVER block or slow down DAQ sampling, because
they're on entirely separate threads. Something else (RasterRecorder)
drains the buffer to tag samples with position; this class doesn't know or
care about rasters, gcode, or position at all.

`daqhats` is imported LAZILY (inside _ensure_hat_open(), not at module load
time) - that library only exists on the Pi with the HAT's drivers
installed, so importing this module elsewhere won't blow up just from
importing it.
"""

import threading
import collections
import time


class DAQController:
    def __init__(self, channel=1, sample_rate=100000.0, samples_per_read=1000,
                 voltage_multiplier=10000.0, address=0, buffer_maxlen=200000):
        self.channel = channel
        self.sample_rate = sample_rate
        self.samples_per_read = samples_per_read
        self.voltage_multiplier = voltage_multiplier
        self.address = address

        self.hat = None
        self.actual_rate = None
        self.scanning = False
        self._OptionFlags = None
        self._HatError = Exception  # replaced with the real HatError once loaded

        self._continuous_thread = None
        self._continuous_running = False
        self._buffer = collections.deque(maxlen=buffer_maxlen)
        self._buffer_lock = threading.Lock()

    def _ensure_hat_open(self):
        if self.hat is not None:
            return
        from daqhats import mcc118, OptionFlags, HatError
        self._OptionFlags = OptionFlags
        self._HatError = HatError
        self.hat = mcc118(self.address)

    def start_scan(self):
        """Opens the HAT the first time only; starts the hardware scan if
        not already running. Returns the actual locked hardware sample rate.

        NOTE: the requested sample_rate (100kHz by default) is not always
        what the hardware actually locks to. This prints the REAL achieved
        rate every time, so a lower-than-expected sample count can be
        traced back to "the hardware only gave us X Hz" rather than assumed
        to be a bug in the sampling loop.

        SELF-HEALING: the MCC118 remembers 'a scan is active' at the
        HARDWARE level, independent of which Python process asks - so if a
        previous run of this script was killed abruptly without calling
        a_in_scan_stop(), the hardware is still marked as scanning. If that
        happens, this clears the stale state and retries once."""
        self._ensure_hat_open()
        if self.scanning:
            return self.actual_rate
        channel_mask = 1 << self.channel
        self.actual_rate = self.hat.a_in_scan_actual_rate(1, self.sample_rate)
        if self.actual_rate < 0.9 * self.sample_rate:
            print(f"[daq] NOTE: requested {self.sample_rate:.0f}Hz, hardware "
                  f"locked to {self.actual_rate:.0f}Hz instead. Effective "
                  f"output rate is only {self.actual_rate/self.samples_per_read:.1f}Hz.")
        else:
            print(f"[daq] Hardware locked at {self.actual_rate:.0f}Hz "
                  f"(effective output rate {self.actual_rate/self.samples_per_read:.1f}Hz).")
        try:
            self.hat.a_in_scan_start(channel_mask, self.samples_per_read,
                                      self.sample_rate, self._OptionFlags.CONTINUOUS)
        except self._HatError as e:
            if "already active" in str(e).lower():
                print("[daq] Hardware reports a scan already active - likely "
                      "left over from a previous run that didn't shut down "
                      "cleanly. Clearing it and retrying...")
                try:
                    self.hat.a_in_scan_stop()
                    self.hat.a_in_scan_cleanup()
                except Exception as cleanup_err:
                    print(f"[daq] Cleanup during recovery raised (usually harmless): {cleanup_err}")
                self.hat.a_in_scan_start(channel_mask, self.samples_per_read,
                                          self.sample_rate, self._OptionFlags.CONTINUOUS)
            else:
                raise
        self.scanning = True
        return self.actual_rate

    def stop_scan(self):
        """Stops the hardware scan but keeps the HAT handle open."""
        if self.hat is not None and self.scanning:
            try:
                self.hat.a_in_scan_stop()
                self.hat.a_in_scan_cleanup()
            except Exception as e:
                print(f"[daq] Error stopping scan (usually harmless): {e}")
        self.scanning = False

    def read_once(self, timeout=0.1):
        """Blocks for roughly samples_per_read / actual_rate seconds (about
        10ms with the default config) and returns ONE averaged, scaled
        charge reading, or None if the scan isn't running / on overrun or
        timeout."""
        if not self.scanning or self.hat is None:
            return None
        try:
            result = self.hat.a_in_scan_read(self.samples_per_read, timeout=timeout)
        except self._HatError as e:
            print(f"[daq] Hardware error: {e}")
            self.scanning = False
            return None

        if result.hardware_overrun or result.buffer_overrun:
            print("[daq] Buffer overrun - the Pi fell behind the DAQ hardware.")
            self.scanning = False
            return None

        samples = result.data
        if len(samples) == 0:
            return None

        avg_voltage = sum(samples) / len(samples)
        return avg_voltage * self.voltage_multiplier

    # ---------------- continuous polling mode ----------------

    def start_continuous(self):
        """Starts the scan AND a dedicated background thread that reads
        continuously, appending (timestamp, charge) to an internal buffer.
        Runs independent of rasters, CNC state, or anything network-related
        - call this ONCE (e.g. at app startup), not per-raster."""
        self.start_scan()
        if self._continuous_running:
            return
        self._continuous_running = True
        self._continuous_thread = threading.Thread(target=self._continuous_loop, daemon=True)
        self._continuous_thread.start()

    def _continuous_loop(self):
        while self._continuous_running:
            charge = self.read_once()
            if charge is not None:
                with self._buffer_lock:
                    self._buffer.append((time.time(), charge))
            else:
                if not self.scanning:
                    print("[daq] Scan stopped unexpectedly - continuous polling halting.")
                    break
                time.sleep(0.01)

    def pop_new_samples(self):
        """Returns (and removes) everything currently buffered, in order.
        Simple single-consumer pattern - call this repeatedly (e.g. every
        few ms from RasterRecorder) to drain what's accumulated since the
        last call."""
        with self._buffer_lock:
            items = list(self._buffer)
            self._buffer.clear()
        return items

    def stop_continuous(self):
        self._continuous_running = False
        if self._continuous_thread is not None:
            self._continuous_thread.join(timeout=2)
            self._continuous_thread = None
        self.stop_scan()

    def close(self):
        """Fully releases the HAT handle - call this once at app shutdown."""
        self.stop_continuous()
        self.hat = None


if __name__ == "__main__":
    # Standalone smoke test - matches your original script's behavior.
    daq = DAQController()
    try:
        rate = daq.start_scan()
        print(f"Hardware scan locked at {rate:.0f} Hz.")
        print(f"Effective update rate: {rate / daq.samples_per_read:.0f} Hz")
        print("Press Ctrl+C to stop.\n")
        while True:
            reading = daq.read_once()
            if reading is not None:
                print(f"Charge: {reading:+.1f} V")
            if not daq.scanning:
                break
    except KeyboardInterrupt:
        print("\nStopped safely.")
    finally:
        daq.close()
