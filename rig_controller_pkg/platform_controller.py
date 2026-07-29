"""
PlatformController - talks to the Stewart platform's Octopus board over
plain USB serial. Unchanged from the merged script, just moved into its own
module.
"""

import threading
import time
import serial
import serial.tools.list_ports


class PlatformController:
    def __init__(self):
        self.ser = None
        self.connected = False
        # Guards every send/read exchange on self.ser. Needed now that the
        # GUI polls the accelerometer ('A') on its own background thread
        # while other threads (cycle controller, button handlers) can send
        # Z/H/W/S/Q at the same time - without this, two threads calling
        # readline() concurrently on the same port can each get half of the
        # other's response line.
        self._lock = threading.RLock()

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    @staticmethod
    def find_platform_port(match_substrings=("F446", "STMicroelectronics"),
                            exclude_substrings=("Klipper", "rp2040")):
        """Scans available serial ports for one matching the Octopus's USB
        descriptor (e.g. 'STMicroelectronics GENERIC F446ZETX...'), while
        explicitly excluding anything that looks like the CNC's Klipper/Pico
        MCU - so this never accidentally grabs the wrong board even if some
        other substring happened to overlap. Returns the port device path,
        or None if nothing matched."""
        for p in serial.tools.list_ports.comports():
            haystack = f"{p.description} {p.hwid}".lower()
            if any(ex.lower() in haystack for ex in exclude_substrings):
                continue
            if any(m.lower() in haystack for m in match_substrings):
                return p.device
        return None

    def auto_connect(self, baud=115200, fallback_paths=("/dev/ttyACM0",)):
        """Finds and connects to the Octopus automatically. Tries the
        descriptor-based scan first (robust to whatever /dev/ttyACMx number
        it happens to enumerate as); if that finds nothing, tries each path
        in fallback_paths directly (known-good paths you've observed it
        land on). Returns the port connected to, or None if nothing worked
        (fall back to the manual port dropdown + Connect)."""
        port = self.find_platform_port()
        if port is not None:
            return port if self.connect(port, baud) else None

        for candidate in fallback_paths:
            if self.connect(candidate, baud):
                return candidate
        return None

    def connect(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # let the board settle after opening the port
            self.ser.reset_input_buffer()
            self.connected = True
            return True
        except serial.SerialException as e:
            print(f"[!] PLATFORM CONNECT ERROR: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def _send(self, cmd):
        if not self.connected:
            return
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    def _read_line(self):
        try:
            return self.ser.readline().decode(errors="ignore").strip()
        except Exception:
            return ""

    def _send_and_wait(self, cmd, done_markers, error_markers=None, timeout=60):
        """Send a command the firmware handles blocking (Z, H), and read
        lines until a completion or error marker. Returns (success, last_line)."""
        error_markers = error_markers or []
        self._send(cmd)
        start = time.time()
        last_line = ""
        while time.time() - start < timeout:
            line = self._read_line()
            if line:
                print(f"[platform] {line}")
                last_line = line
                if any(m in line for m in error_markers):
                    return False, last_line
                if any(m in line for m in done_markers):
                    return True, last_line
        return False, "TIMEOUT"

    def home(self, timeout=60):
        """Z - assumes you've set the horns vertical by hand."""
        with self._lock:
            return self._send_and_wait("Z", ["Ready."], ["ERROR"], timeout)

    def hall_home(self, timeout=120):
        """H - auto-finds vertical via the hall sensors, then homes."""
        with self._lock:
            return self._send_and_wait("H", ["Ready."], ["ERROR", "ABORTED"], timeout)

    def wiggle_start(self, tilt_deg, period_s):
        """W - starts the continuous orbit. Non-blocking; call level() to stop."""
        with self._lock:
            self._send(f"W {tilt_deg} {period_s}")

    def level(self):
        """S - stop and return to the level neutral pose. Non-blocking by
        itself; follow with wait_until_settled() once that's wired to a
        real confirmation source."""
        with self._lock:
            self._send("S")

    def wait_until_settled(self, poll_interval=0.2, timeout=30):
        """Polls Q until the firmware reports SETTLED (all legs at target).
        This is the hook point for the accelerometer: once that's wired in,
        this is where you'd ALSO read it and confirm actual level, not just
        'motors think they're done'."""
        if not self.connected:
            print("[platform] wait_until_settled called with no connection - failing fast.")
            return False
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                self._send("Q")
                line = self._read_line()
            if "SETTLED" in line:
                return True
            time.sleep(poll_interval)
        return False

    def read_accel(self, timeout=2.0):
        """A - reads the MMA8451 wired to the Octopus's onboard I2C header.
        Returns an (x, y, z) tuple in g's, or None if not connected, the
        firmware reports the sensor never initialized (ACCEL ERR), or
        nothing usable comes back within timeout."""
        if not self.connected:
            return None
        with self._lock:
            self._send("A")
            start = time.time()
            while time.time() - start < timeout:
                line = self._read_line()
                if not line:
                    continue
                if line.startswith("ACCEL ERR"):
                    return None
                if line.startswith("ACCEL "):
                    parts = line.split()
                    if len(parts) == 4:
                        try:
                            return tuple(float(p) for p in parts[1:])
                        except ValueError:
                            return None
                    return None
            return None

    def stop_now(self):
        """Emergency-style stop: just sends S. (The firmware has no separate
        immediate-halt command yet - S is the closest thing.)"""
        with self._lock:
            self._send("S")
