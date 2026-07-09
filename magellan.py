import tkinter as tk
import requests
import math
import time
import threading

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
        # Changed from ?toolhead=position to ?motion_report=live_position
        url = f"{self.base_url}/printer/objects/query?motion_report=live_position"
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                # Grab the live physical coordinates
                pos = data['result']['status']['motion_report']['live_position']
                return pos[0], pos[1]
        except requests.exceptions.RequestException:
            pass
        return None, None

class JogGUI:
    def __init__(self, root, controller):
        self.klipper = controller
        self.root = root
        self.root.title("CNC Raster Mapper")
        
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

        self.setup_ui()
        self.update_coordinates()
        
        print("Initializing program and homing machine...")
        self.home_machine()

    def setup_ui(self):
        frame = tk.Frame(self.root)
        frame.pack(padx=20, pady=20)

        # Coordinate Display
        self.coord_label = tk.Label(frame, text="X: 0.00  |  Y: 0.00", font=('Helvetica', 16, 'bold'), fg="blue")
        self.coord_label.grid(row=0, column=0, columnspan=3, pady=(0, 15))

        # Directional Buttons
        tk.Button(frame, text="▲ Y+", command=lambda: self.jog("Y", self.jog_distance), width=10, height=2).grid(row=1, column=1, pady=5)
        tk.Button(frame, text="◀ X-", command=lambda: self.jog("X", -self.jog_distance), width=10, height=2).grid(row=2, column=0, padx=5)
        tk.Button(frame, text="▶ X+", command=lambda: self.jog("X", self.jog_distance), width=10, height=2).grid(row=2, column=2, padx=5)
        tk.Button(frame, text="▼ Y-", command=lambda: self.jog("Y", -self.jog_distance), width=10, height=2).grid(row=3, column=1, pady=5)

        # Action Buttons
        tk.Button(frame, text="Home All (G28)", bg="yellow", command=self.home_machine).grid(row=2, column=1)
        tk.Button(frame, text="EMERGENCY STOP", bg="red", fg="white", font=('Helvetica', 12, 'bold'), command=self.e_stop).grid(row=4, column=0, columnspan=3, pady=(15, 5))
        tk.Button(frame, text="Firmware Restart", bg="orange", command=self.firmware_restart).grid(row=5, column=0, columnspan=3, pady=5)
        
        # Test Paths 
        tk.Button(frame, text=f"Draw {self.circle_radius*2}mm Circle", bg="lightblue", command=self.draw_circle).grid(row=6, column=0, columnspan=3, pady=(10, 5))
        tk.Button(frame, text="Raster Area", bg="lightgreen", command=self.raster_sensor).grid(row=7, column=0, columnspan=3, pady=5)
        
        # NEW: Timer Display
        self.timer_label = tk.Label(frame, text="Last Raster Time: --.-- s", font=('Helvetica', 12, 'bold'), fg="green")
        self.timer_label.grid(row=8, column=0, columnspan=3, pady=(10, 0))

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
        # 1. Prepare coordinates
        cx = self.start_x
        cy = self.start_y + self.circle_radius
        stepover = self.raster_stepover 
        
        # 2. Slice 1: Setup G-code
        setup_gcode = f"G90\nG1 X{self.start_x:.3f} Y{self.start_y:.3f} F{self.fast_speed}\n"
        
        # 3. Slice 2: Raster G-code
        raster_gcode = ""
        y_offsets = []
        current_y = -self.circle_radius
        while current_y <= self.circle_radius + 0.1:
            y_offsets.append(current_y)
            current_y += stepover

        direction = 1 
        last_x, last_y = 0, 0
        
        for y_off in y_offsets:
            val_under_root = self.circle_radius**2 - y_off**2
            if val_under_root < 0:
                val_under_root = 0
                
            x_val = math.sqrt(val_under_root)
            x_start = cx - (x_val * direction)
            x_end = cx + (x_val * direction)
            y_actual = cy + y_off
            
            raster_gcode += f"G1 X{x_start:.3f} Y{y_actual:.3f} F{self.fast_speed}\n"
            raster_gcode += f"G1 X{x_end:.3f} Y{y_actual:.3f} F{self.fast_speed}\n"
            
            # Store the very last coordinate it visits to know when to stop the timer
            last_x = x_end
            last_y = y_actual
            direction *= -1
            
        # 4. Slice 3: Return Home G-code
        end_gcode = f"G1 X0 Y0 F{self.fast_speed}\n"
        
        # 5. Start the background monitoring thread
        self.timer_label.config(text="Rastering... Timer Active", fg="orange")
        threading.Thread(
            target=self._run_timed_sequence, 
            args=(setup_gcode, raster_gcode, end_gcode, self.start_x, self.start_y, last_x, last_y),
            daemon=True
        ).start()

    def _run_timed_sequence(self, setup_gcode, raster_gcode, end_gcode, start_x, start_y, end_x, end_y):
        """Runs in the background so Tkinter doesn't freeze while we poll coordinates."""
        print(f"Moving to start coordinates (X{start_x}, Y{start_y})...")
        self.klipper.send_gcode(setup_gcode)
        self._wait_for_pos(start_x, start_y)
        
        print("At start position. Starting raster and timer!")
        start_time = time.time()
        self.klipper.send_gcode(raster_gcode)
        
        # Wait until it hits that final raster line coordinate
        self._wait_for_pos(end_x, end_y)
        elapsed_time = time.time() - start_time
        
        print(f"Raster complete! Total physical time: {elapsed_time:.2f} seconds.")
        
        # Send GUI update back to the main Tkinter thread safely
        self.root.after(0, lambda: self.timer_label.config(text=f"Last Raster Time: {elapsed_time:.2f} s", fg="green"))
        
        print("Returning to 0,0...")
        self.klipper.send_gcode(end_gcode)

    def _wait_for_pos(self, target_x, target_y, tolerance=1.5, timeout=300):
        """Continuously polls Klipper until the toolhead physically reaches the target."""
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            x, y = self.klipper.get_position()
            if x is not None and y is not None:
                # If we are within the physical tolerance of the target, break the loop
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
        self.timer_label.config(text="E-STOP TRIGGERED", fg="red")

    def firmware_restart(self):
        print("Restarting firmware to clear shutdown state...")
        self.klipper.send_gcode("FIRMWARE_RESTART")

if __name__ == "__main__":
    root = tk.Tk()
    printer = KlipperController("127.0.0.1") 
    app = JogGUI(root, printer)
    root.mainloop()