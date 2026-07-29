import tkinter as tk
from tkinter import ttk
import requests
import math
import time
import threading
import serial
import serial.tools.list_ports


# ============================================================================
# KLIPPER / CNC SIDE - unchanged from your original script, with one addition
# (get_homed_axes) so the auto-cycle can confirm a real re-home rather than
# just guessing from position.
# ============================================================================

class KlipperController:
    def __init__(self, ip_address="127.0.0.1", port=7125):
        self.base_url = f"http://{ip_address}:{port}"

    def send_gcode(self, gcode_cmd):
        url = f"{self.base_url}/printer/gcode/script"
        try:
            response = requests.post(url, json={"script": gcode_cmd}, timeout=5)
            if response.status_code == 200:
                return True
            else:
                print(f"\n[!] KLIPPER ERROR: {response.text}\n")
                return False
        except requests.exceptions.RequestException:
            return False

    def get_position(self):
        url = f"{self.base_url}/printer/objects/query?motion_report=live_position"
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                pos = data['result']['status']['motion_report']['live_position']
                return pos[0], pos[1]
        except requests.exceptions.RequestException:
            pass
        return None, None

    # NEW: used by the auto-cycle to confirm a real homing completion,
    # rather than inferring it from position (endstop trigger offsets mean
    # "homed" doesn't always land exactly at your nominal 0,0).
    def get_homed_axes(self):
        url = f"{self.base_url}/printer/objects/query?toolhead=homed_axes"
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                return data['result']['status']['toolhead']['homed_axes']
        except requests.exceptions.RequestException:
            pass
        return ""

    def is_fully_homed(self, axes="xy"):
        homed = self.get_homed_axes()
        return all(a in homed for a in axes)


# ============================================================================
# STEWART PLATFORM SIDE - new. Talks to the Octopus over plain USB serial
# using the same text protocol you've been sending by hand: Z, H, P, W, J,
# S, and the new Q status query.
# ============================================================================

class PlatformController:
    def __init__(self):
        self.ser = None
        self.connected = False

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

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
            line = self.ser.readline().decode(errors="ignore").strip()
            return line
        except Exception:
            return ""

    def _send_and_wait(self, cmd, done_markers, error_markers=None, timeout=60):
        """Send a command that the firmware handles blocking (Z, H), and
        read lines until we see a completion or error marker. Returns
        (success, last_line)."""
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
        return self._send_and_wait("Z", ["Ready."], ["ERROR"], timeout)

    def hall_home(self, timeout=120):
        """H - auto-finds vertical via the hall sensors, then homes."""
        return self._send_and_wait("H", ["Ready."], ["ERROR", "ABORTED"], timeout)

    def wiggle_start(self, tilt_deg, period_s):
        """W - starts the continuous orbit. Non-blocking; call level() to stop."""
        self._send(f"W {tilt_deg} {period_s}")

    def level(self):
        """S - stop and return to the level neutral pose. Non-blocking by
        itself; follow with wait_until_settled()."""
        self._send("S")

    def wait_until_settled(self, poll_interval=0.2, timeout=30):
        """Polls Q until the firmware reports SETTLED (all legs at target).
        This is the hook point for the accelerometer later: once that's
        wired in, this is where you'd ALSO read it and confirm actual level,
        not just 'motors think they're done'."""
        if not self.connected:
            print("[platform] wait_until_settled called with no connection - failing fast.")
            return False
        start = time.time()
        while time.time() - start < timeout:
            self._send("Q")
            line = self._read_line()
            if "SETTLED" in line:
                return True
            time.sleep(poll_interval)
        return False

    def stop_now(self):
        """Emergency-style stop: just sends S. (The firmware has no separate
        immediate-halt command yet - S is the closest thing.)"""
        self._send("S")


# ============================================================================
# RASTER G-CODE BUILDER - pulled out of the original raster_sensor() method
# so both the manual button AND the auto-cycle can share the exact same
# math without duplicating it.
# ============================================================================

def build_raster_gcode(start_x, start_y, circle_radius, raster_stepover, fast_speed):
    cx = start_x
    cy = start_y + circle_radius

    setup_gcode = f"G90\nG1 X{start_x:.3f} Y{start_y:.3f} F{fast_speed}\n"

    raster_gcode = ""
    y_offsets = []
    current_y = -circle_radius
    while current_y <= circle_radius + 0.1:
        y_offsets.append(current_y)
        current_y += raster_stepover

    direction = 1
    last_x, last_y = start_x, start_y

    for y_off in y_offsets:
        val_under_root = circle_radius**2 - y_off**2
        if val_under_root < 0:
            val_under_root = 0
        x_val = math.sqrt(val_under_root)
        x_start = cx - (x_val * direction)
        x_end = cx + (x_val * direction)
        y_actual = cy + y_off

        raster_gcode += f"G1 X{x_start:.3f} Y{y_actual:.3f} F{fast_speed}\n"
        raster_gcode += f"G1 X{x_end:.3f} Y{y_actual:.3f} F{fast_speed}\n"

        last_x = x_end
        last_y = y_actual
        direction *= -1

    end_gcode = "G1 X0 Y0 F{}\n".format(fast_speed)
    return setup_gcode, raster_gcode, end_gcode, last_x, last_y


# ============================================================================
# CYCLE ORCHESTRATION - the actual state machine you described:
#   wiggle -> level (confirm) -> raster (confirm) -> re-home CNC (confirm)
#   -> wiggle -> ... repeat, for N cycles or indefinitely.
# Runs in its own background thread so the GUI stays responsive.
# ============================================================================

class CycleController:
    def __init__(self, klipper: KlipperController, platform: PlatformController,
                 status_callback, cnc_wait_for_pos_fn):
        self.klipper = klipper
        self.platform = platform
        self.status_cb = status_callback  # called with a short status string
        self.cnc_wait_for_pos = cnc_wait_for_pos_fn
        self._stop_flag = threading.Event()
        self._thread = None
        self.running = False

    def start(self, wiggle_tilt_deg, wiggle_period_s, wiggle_rotations,
              raster_params, num_cycles=None, rehome_every_n=1,
              level_settle_delay_s=3.0):
        """num_cycles=None means run indefinitely until stop() is called.
        rehome_every_n: re-home the CNC every N cycles instead of every
        single one (sensorless homing physically stalls against a hard
        stop each time - re-homing less often means less mechanical wear,
        at the cost of not catching missed-step drift as quickly).
        level_settle_delay_s: TEMPORARY fixed delay after sending S, used
        in place of a real settle/level confirmation until the
        accelerometer is wired in (see _run for the swap-back point)."""
        if self.running:
            return
        self._stop_flag.clear()
        self.running = True
        self._thread = threading.Thread(
            target=self._run,
            args=(wiggle_tilt_deg, wiggle_period_s, wiggle_rotations,
                  raster_params, num_cycles, rehome_every_n, level_settle_delay_s),
            daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_flag.set()

    def _run(self, tilt_deg, period_s, rotations, raster_params, num_cycles,
              rehome_every_n, level_settle_delay_s):
        cycle_count = 0
        try:
            while not self._stop_flag.is_set():
                if num_cycles is not None and cycle_count >= num_cycles:
                    break
                cycle_count += 1
                self.status_cb(f"Cycle {cycle_count}: wiggling...")

                # --- WIGGLE ---
                self.platform.wiggle_start(tilt_deg, period_s)
                wiggle_duration = period_s * rotations
                elapsed = 0.0
                step = 0.2
                while elapsed < wiggle_duration:
                    if self._stop_flag.is_set():
                        self.platform.level()
                        return
                    time.sleep(step)
                    elapsed += step

                # --- LEVEL ---
                self.status_cb(f"Cycle {cycle_count}: leveling...")
                self.platform.level()
                # TEMPORARY: no accelerometer yet, so there's no real signal
                # to confirm actual level - just wait a fixed, generous delay
                # instead of polling for a "SETTLED" motor-status confirmation
                # (which requires a live serial connection; if that's not
                # connected this used to silently burn its whole timeout).
                # Swap this back to the real check once the accelerometer is
                # wired in:
                #   settled = self.platform.wait_until_settled(timeout=30)
                #   if not settled:
                #       self.status_cb(f"Cycle {cycle_count}: ERROR - platform did not settle. Stopping.")
                #       return
                # ...and add a real roll/pitch-vs-zero check right here too.
                time.sleep(level_settle_delay_s)

                if self._stop_flag.is_set():
                    return

                # --- RASTER (and confirm via position, like the original script) ---
                self.status_cb(f"Cycle {cycle_count}: rastering...")
                start_x, start_y, circle_radius, stepover, fast_speed = raster_params
                setup_g, raster_g, end_g, last_x, last_y = build_raster_gcode(
                    start_x, start_y, circle_radius, stepover, fast_speed
                )
                self.klipper.send_gcode(setup_g)
                self.cnc_wait_for_pos(start_x, start_y)
                self.klipper.send_gcode(raster_g)
                self.cnc_wait_for_pos(last_x, last_y)
                self.klipper.send_gcode(end_g)
                self.cnc_wait_for_pos(0, 0)

                if self._stop_flag.is_set():
                    return

                # --- RE-HOME CNC (every rehome_every_n cycles, not every
                # single one - sensorless homing deliberately stalls the
                # motor against a hard mechanical stop each time, so doing
                # it less often means less cumulative wear, at the cost of
                # slower detection of any missed-step drift). ---
                if cycle_count % rehome_every_n == 0:
                    self.status_cb(f"Cycle {cycle_count}: re-homing CNC...")
                    self.klipper.send_gcode("G28")
                    homed_ok = self._wait_for_homed(timeout=60)
                    if not homed_ok:
                        self.status_cb(f"Cycle {cycle_count}: ERROR - CNC did not confirm home. Stopping.")
                        return
                else:
                    self.status_cb(f"Cycle {cycle_count}: skipping re-home ({cycle_count % rehome_every_n} of {rehome_every_n}).")

                self.status_cb(f"Cycle {cycle_count}: complete.")
        finally:
            self.running = False
            if num_cycles is not None and cycle_count >= num_cycles and not self._stop_flag.is_set():
                self.status_cb(f"Done - completed {cycle_count} cycles.")
            else:
                self.status_cb("Stopped.")

    def _wait_for_homed(self, timeout=60):
        start = time.time()
        while time.time() - start < timeout:
            if self.klipper.is_fully_homed("xy"):
                return True
            time.sleep(0.3)
        return False


# ============================================================================
# GUI
# ============================================================================

class JogGUI:
    def __init__(self, root, controller):
        self.klipper = controller
        self.root = root
        self.root.title("CNC + Stewart Platform Rig Controller")

        # ==========================================
        # --- CENTRAL CONTROL PANEL (VARIABLES) ---
        # ==========================================

        self.jog_distance = 5
        self.jog_speed = 3000

        self.start_x = 117.5
        self.start_y = 15
        self.circle_radius = 90
        self.fast_speed = 18000

        self.sensor_radius = 12.50
        self.raster_stepover = 25.0

        # ==========================================

        self.platform = PlatformController()
        self.cycle = CycleController(
            self.klipper, self.platform,
            status_callback=self._on_cycle_status,
            cnc_wait_for_pos_fn=self._wait_for_pos
        )

        self.setup_ui()
        self.update_coordinates()

        print("Initializing program and homing machine...")
        self.home_machine()

    # ---------------- original CNC UI (unchanged) ----------------

    def setup_ui(self):
        outer = tk.Frame(self.root)
        outer.pack(padx=20, pady=20)

        cnc_frame = tk.LabelFrame(outer, text="CNC (Klipper)", padx=10, pady=10)
        cnc_frame.grid(row=0, column=0, sticky="n", padx=(0, 15))
        self._setup_cnc_ui(cnc_frame)

        right_col = tk.Frame(outer)
        right_col.grid(row=0, column=1, sticky="n")

        platform_frame = tk.LabelFrame(right_col, text="Stewart Platform", padx=10, pady=10)
        platform_frame.pack(fill="x", pady=(0, 15))
        self._setup_platform_ui(platform_frame)

        cycle_frame = tk.LabelFrame(right_col, text="Auto Cycle", padx=10, pady=10)
        cycle_frame.pack(fill="x")
        self._setup_cycle_ui(cycle_frame)

    def _setup_cnc_ui(self, frame):
        self.coord_label = tk.Label(frame, text="X: 0.00  |  Y: 0.00", font=('Helvetica', 16, 'bold'), fg="blue")
        self.coord_label.grid(row=0, column=0, columnspan=3, pady=(0, 15))

        tk.Button(frame, text="▲ Y+", command=lambda: self.jog("Y", self.jog_distance), width=10, height=2).grid(row=1, column=1, pady=5)
        tk.Button(frame, text="◀ X-", command=lambda: self.jog("X", -self.jog_distance), width=10, height=2).grid(row=2, column=0, padx=5)
        tk.Button(frame, text="▶ X+", command=lambda: self.jog("X", self.jog_distance), width=10, height=2).grid(row=2, column=2, padx=5)
        tk.Button(frame, text="▼ Y-", command=lambda: self.jog("Y", -self.jog_distance), width=10, height=2).grid(row=3, column=1, pady=5)

        tk.Button(frame, text="Home All (G28)", bg="yellow", command=self.home_machine).grid(row=2, column=1)
        tk.Button(frame, text="EMERGENCY STOP", bg="red", fg="white", font=('Helvetica', 12, 'bold'), command=self.e_stop).grid(row=4, column=0, columnspan=3, pady=(15, 5))
        tk.Button(frame, text="Firmware Restart", bg="orange", command=self.firmware_restart).grid(row=5, column=0, columnspan=3, pady=5)

        tk.Button(frame, text=f"Draw {self.circle_radius*2}mm Circle", bg="lightblue", command=self.draw_circle).grid(row=6, column=0, columnspan=3, pady=(10, 5))
        tk.Button(frame, text="Raster Area", bg="lightgreen", command=self.raster_sensor).grid(row=7, column=0, columnspan=3, pady=5)

        self.timer_label = tk.Label(frame, text="Last Raster Time: --.-- s", font=('Helvetica', 12, 'bold'), fg="green")
        self.timer_label.grid(row=8, column=0, columnspan=3, pady=(10, 0))

    # ---------------- new: platform connection + manual controls ----------------

    def _setup_platform_ui(self, frame):
        tk.Label(frame, text="Port:").grid(row=0, column=0, sticky="e")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=0, column=1, padx=5)
        self._refresh_ports()
        tk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2)

        self.platform_status_label = tk.Label(frame, text="Not connected", fg="red")
        self.platform_status_label.grid(row=1, column=0, columnspan=3, pady=(5, 5))

        tk.Button(frame, text="Connect", bg="lightyellow", command=self._connect_platform).grid(row=2, column=0, columnspan=3, sticky="we", pady=(0, 10))

        tk.Button(frame, text="Home (Z - manual vertical)", command=self._platform_home).grid(row=3, column=0, columnspan=3, sticky="we")
        tk.Button(frame, text="Auto-Home (H - hall sensors)", command=self._platform_hall_home).grid(row=4, column=0, columnspan=3, sticky="we", pady=(5, 0))
        tk.Button(frame, text="Level (S)", command=lambda: self.platform.level()).grid(row=5, column=0, columnspan=3, sticky="we", pady=(5, 0))

    def _refresh_ports(self):
        ports = PlatformController.list_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _connect_platform(self):
        port = self.port_var.get()
        if not port:
            self.platform_status_label.config(text="No port selected", fg="red")
            return
        ok = self.platform.connect(port)
        if ok:
            self.platform_status_label.config(text=f"Connected: {port}", fg="green")
        else:
            self.platform_status_label.config(text=f"Failed to connect: {port}", fg="red")

    def _platform_home(self):
        threading.Thread(target=self.platform.home, daemon=True).start()

    def _platform_hall_home(self):
        threading.Thread(target=self.platform.hall_home, daemon=True).start()

    # ---------------- new: auto-cycle controls ----------------

    def _setup_cycle_ui(self, frame):
        tk.Label(frame, text="Wiggle tilt (deg):").grid(row=0, column=0, sticky="e")
        self.wiggle_tilt_var = tk.StringVar(value="10")
        tk.Entry(frame, textvariable=self.wiggle_tilt_var, width=8).grid(row=0, column=1)

        tk.Label(frame, text="Wiggle period (s):").grid(row=1, column=0, sticky="e")
        self.wiggle_period_var = tk.StringVar(value="4")
        tk.Entry(frame, textvariable=self.wiggle_period_var, width=8).grid(row=1, column=1)

        tk.Label(frame, text="Rotations per wiggle:").grid(row=2, column=0, sticky="e")
        self.wiggle_rotations_var = tk.StringVar(value="3")
        tk.Entry(frame, textvariable=self.wiggle_rotations_var, width=8).grid(row=2, column=1)

        tk.Label(frame, text="Level settle delay (s):").grid(row=3, column=0, sticky="e")
        self.settle_delay_var = tk.StringVar(value="3")
        tk.Entry(frame, textvariable=self.settle_delay_var, width=8).grid(row=3, column=1)
        # ^ TEMPORARY fixed wait in place of a real level confirmation, until
        # the accelerometer is wired in.

        tk.Label(frame, text="Re-home CNC every N cycles:").grid(row=4, column=0, sticky="e")
        self.rehome_every_n_var = tk.StringVar(value="1")
        tk.Entry(frame, textvariable=self.rehome_every_n_var, width=8).grid(row=4, column=1)
        # ^ sensorless homing stalls the motor against a hard stop every
        # time - set higher than 1 to re-home less often (less wear, slower
        # detection of any missed-step drift).

        self.indefinite_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame, text="Run indefinitely", variable=self.indefinite_var,
                       command=self._toggle_cycle_count_entry).grid(row=5, column=0, columnspan=2, sticky="w")

        tk.Label(frame, text="Num cycles:").grid(row=6, column=0, sticky="e")
        self.num_cycles_var = tk.StringVar(value="10")
        self.num_cycles_entry = tk.Entry(frame, textvariable=self.num_cycles_var, width=8, state="disabled")
        self.num_cycles_entry.grid(row=6, column=1)

        tk.Button(frame, text="Start Auto Cycle", bg="lightgreen", command=self._start_cycle).grid(row=7, column=0, columnspan=2, sticky="we", pady=(10, 2))
        tk.Button(frame, text="Stop Auto Cycle", bg="salmon", command=self._stop_cycle).grid(row=8, column=0, columnspan=2, sticky="we")

        self.cycle_status_label = tk.Label(frame, text="Idle", font=('Helvetica', 11, 'bold'), fg="gray")
        self.cycle_status_label.grid(row=9, column=0, columnspan=2, pady=(10, 0))

    def _toggle_cycle_count_entry(self):
        self.num_cycles_entry.config(state="disabled" if self.indefinite_var.get() else "normal")

    def _start_cycle(self):
        try:
            tilt = float(self.wiggle_tilt_var.get())
            period = float(self.wiggle_period_var.get())
            rotations = float(self.wiggle_rotations_var.get())
            settle_delay = float(self.settle_delay_var.get())
            rehome_every_n = int(self.rehome_every_n_var.get())
            if rehome_every_n < 1:
                raise ValueError("rehome_every_n must be >= 1")
            num_cycles = None if self.indefinite_var.get() else int(self.num_cycles_var.get())
        except ValueError:
            self.cycle_status_label.config(text="Invalid cycle settings", fg="red")
            return

        raster_params = (self.start_x, self.start_y, self.circle_radius,
                          self.raster_stepover, self.fast_speed)
        self.cycle.start(tilt, period, rotations, raster_params, num_cycles,
                          rehome_every_n=rehome_every_n,
                          level_settle_delay_s=settle_delay)

    def _stop_cycle(self):
        self.cycle.stop()

    def _on_cycle_status(self, text):
        # called from the cycle's background thread - hop back to the GUI thread
        self.root.after(0, lambda: self.cycle_status_label.config(text=text, fg="orange"))

    # ---------------- original CNC methods (unchanged) ----------------

    def update_coordinates(self):
        x, y = self.klipper.get_position()
        if x is None or y is None:
            self.coord_label.config(text="X: --.--  |  Y: --.--", fg="red")
        else:
            self.coord_label.config(text=f"X: {x:.2f}  |  Y: {y:.2f}", fg="blue")

        self.root.after(500, self.update_coordinates)

    def jog(self, axis, distance):
        gcode = f"G91\nG1 {axis}{distance} F{self.jog_speed}\nG90"
        print(f"Jogging: {axis} by {distance}mm")
        self.klipper.send_gcode(gcode)

    def draw_circle(self):
        gcode = f"G90\nG1 X{self.start_x} Y{self.start_y} F{self.fast_speed}\nG3 I0 J{self.circle_radius} F{self.fast_speed}\nG1 X0 Y0 F{self.fast_speed}"
        print(f"Snapping to X{self.start_x} Y{self.start_y}, drawing {self.circle_radius*2}mm Circle, returning to 0,0...")
        self.klipper.send_gcode(gcode)

    def raster_sensor(self):
        setup_gcode, raster_gcode, end_gcode, last_x, last_y = build_raster_gcode(
            self.start_x, self.start_y, self.circle_radius, self.raster_stepover, self.fast_speed
        )
        self.timer_label.config(text="Rastering... Timer Active", fg="orange")
        threading.Thread(
            target=self._run_timed_sequence,
            args=(setup_gcode, raster_gcode, end_gcode, self.start_x, self.start_y, last_x, last_y),
            daemon=True
        ).start()

    def _run_timed_sequence(self, setup_gcode, raster_gcode, end_gcode, start_x, start_y, end_x, end_y):
        print(f"Moving to start coordinates (X{start_x}, Y{start_y})...")
        self.klipper.send_gcode(setup_gcode)
        self._wait_for_pos(start_x, start_y)

        print("At start position. Starting raster and timer!")
        start_time = time.time()
        self.klipper.send_gcode(raster_gcode)

        self._wait_for_pos(end_x, end_y)
        elapsed_time = time.time() - start_time

        print(f"Raster complete! Total physical time: {elapsed_time:.2f} seconds.")

        self.root.after(0, lambda: self.timer_label.config(text=f"Last Raster Time: {elapsed_time:.2f} s", fg="green"))

        print("Returning to 0,0...")
        self.klipper.send_gcode(end_gcode)

    def _wait_for_pos(self, target_x, target_y, tolerance=1.5, timeout=300):
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            x, y = self.klipper.get_position()
            if x is not None and y is not None:
                if abs(x - target_x) <= tolerance and abs(y - target_y) <= tolerance:
                    return True
            time.sleep(0.1)
        print("Warning: Coordinate polling timed out (hit E-Stop or machine stalled).")
        return False

    def home_machine(self):
        print("Homing machine...")
        self.klipper.send_gcode("G28")

    def e_stop(self):
        print("EMERGENCY STOP ACTIVATED")
        self.klipper.send_gcode("M112")
        self.cycle.stop()
        self.platform.stop_now()
        self.timer_label.config(text="E-STOP TRIGGERED", fg="red")

    def firmware_restart(self):
        print("Restarting firmware to clear shutdown state...")
        self.klipper.send_gcode("FIRMWARE_RESTART")


if __name__ == "__main__":
    root = tk.Tk()
    printer = KlipperController("127.0.0.1")
    app = JogGUI(root, printer)
    root.mainloop()