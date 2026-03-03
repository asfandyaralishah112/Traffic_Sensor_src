import json
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import queue
import paho.mqtt.client as mqtt
import tkinter as tk
from tkinter import ttk, messagebox

# =========================
# CONFIG & PERSISTENCE
# =========================
CONFIG_FILE = "plot_config.json"

DEFAULT_CONFIG = {
    "broker": "www.cavlineglobal.com",
    "port": 1883,
    "user": "Traffic_Sensor",
    "password": "randompass",
    "uid": "CVL-TS1-26-000001"
}

# Tracking Constants
MIN_ACTIVE_PIXELS = 1
BASELINE_ALPHA = 0.001
NOISE_ALPHA = 0.01
NOISE_MULTIPLIER = 2.0
DOOR_LINE = 3.5
MAX_CONSECUTIVE_MISSES = 3

class SensorPlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Cavline Global - Traffic Sensor Visualizer")
        self.root.geometry("1000x700")
        
        self.data_queue = queue.Queue()
        self.mqtt_client = None
        self.is_connected = False
        
        # Tracking Variables
        self.baseline = None
        self.noise = None
        self.trajectory = []
        self.tracking_active = False
        self.total_in = 0
        self.total_out = 0
        self.last_centroid_y = None
        self.last_velocity = 0
        self.consecutive_misses = 0
        
        self.setup_ui()
        self.load_settings()
        self.setup_plot()
        
    def setup_ui(self):
        # Main Layout
        self.main_pane = ttk.PanedWindow(self.root, orient="horizontal")
        self.main_pane.pack(fill="both", expand=True)
        
        # --- Sidebar (Controls) ---
        self.sidebar = ttk.Frame(self.main_pane, padding=10)
        self.main_pane.add(self.sidebar, weight=1)
        
        ttk.Label(self.sidebar, text="MQTT SETTINGS", font=("Arial", 10, "bold")).pack(pady=(0, 10))
        
        fields = [
            ("Broker", "broker"),
            ("Port", "port"),
            ("User", "user"),
            ("Password", "password"),
            ("Device UID", "uid")
        ]
        
        self.entries = {}
        for label_text, key in fields:
            ttk.Label(self.sidebar, text=label_text).pack(anchor="w")
            entry = ttk.Entry(self.sidebar)
            if key == "password":
                entry.config(show="*")
            entry.pack(fill="x", pady=(0, 5))
            self.entries[key] = entry
            
        self.conn_btn = ttk.Button(self.sidebar, text="Connect", command=self.toggle_connection)
        self.conn_btn.pack(fill="x", pady=10)
        
        ttk.Separator(self.sidebar, orient="horizontal").pack(fill="x", pady=10)
        
        ttk.Label(self.sidebar, text="DEVICE CONTROL", font=("Arial", 10, "bold")).pack(pady=(0, 10))
        
        self.telemetry_btn = ttk.Button(self.sidebar, text="Enable Telemetry", command=self.toggle_telemetry, state="disabled")
        self.telemetry_btn.pack(fill="x", pady=5)
        self.telemetry_state = False
        
        ttk.Button(self.sidebar, text="Reset Baseline", command=self.reset_baseline).pack(fill="x", pady=5)
        ttk.Button(self.sidebar, text="Clear Counts", command=self.clear_counts).pack(fill="x", pady=5)
        
        ttk.Separator(self.sidebar, orient="horizontal").pack(fill="x", pady=20)
        
        self.in_label = ttk.Label(self.sidebar, text="IN: 0", font=("Arial", 20, "bold"), foreground="#2f9e44")
        self.in_label.pack(pady=5)
        
        self.out_label = ttk.Label(self.sidebar, text="OUT: 0", font=("Arial", 20, "bold"), foreground="#e03131")
        self.out_label.pack(pady=5)
        
        self.info_label = ttk.Label(self.sidebar, text="Status: Disconnected", foreground="red")
        self.info_label.pack(side="bottom", pady=10)

        # --- Plot Area ---
        self.plot_container = ttk.Frame(self.main_pane, padding=10)
        self.main_pane.add(self.plot_container, weight=4)
        
    def setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.grid_img = self.ax.imshow(np.zeros((8, 8)), vmin=0, vmax=2500, cmap='viridis')
        self.fig.colorbar(self.grid_img, ax=self.ax)
        self.ax.set_title("VL53L5CX Telemetry Grid")
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Start background update loop
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=False)

    def load_settings(self):
        config = DEFAULT_CONFIG.copy()
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    config.update(json.load(f))
            except:
                pass
        
        for key, entry in self.entries.items():
            entry.delete(0, tk.END)
            entry.insert(0, str(config.get(key, "")))
            
    def save_settings(self):
        config = {key: entry.get() for key, entry in self.entries.items()}
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)

    def toggle_connection(self):
        if not self.is_connected:
            self.connect_mqtt()
        else:
            self.disconnect_mqtt()

    def connect_mqtt(self):
        self.save_settings()
        broker = self.entries["broker"].get()
        try:
            port = int(self.entries["port"].get())
        except:
            messagebox.showerror("Error", "Invalid Port")
            return
        user = self.entries["user"].get()
        password = self.entries["password"].get()
        
        self.mqtt_client = mqtt.Client()
        if port == 8883:
            self.mqtt_client.tls_set()
            
        if user:
            self.mqtt_client.username_pw_set(user, password)
            
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(broker, port, 60)
            self.mqtt_client.loop_start()
            self.is_connected = True
            self.conn_btn.config(text="Disconnect")
            self.info_label.config(text="Status: Connected", foreground="green")
            self.telemetry_btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect_mqtt(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        self.is_connected = False
        self.conn_btn.config(text="Connect")
        self.info_label.config(text="Status: Disconnected", foreground="red")
        self.telemetry_btn.config(state="disabled")

    def on_connect(self, client, userdata, flags, rc):
        uid = self.entries["uid"].get()
        topic = f"cavline/traffic_sensor/{uid}/telemetry"
        client.subscribe(topic)
        print(f"Subscribed to {topic}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if "zones" in payload:
                if self.data_queue.qsize() > 5:
                    self.data_queue.get_nowait()
                self.data_queue.put(payload)
        except Exception as e:
            print(f"MQTT msg error: {e}")

    def toggle_telemetry(self):
        uid = self.entries["uid"].get()
        topic = f"cavline/traffic_sensor/{uid}/command"
        self.telemetry_state = not self.telemetry_state
        state_str = "ON" if self.telemetry_state else "OFF"
        
        payload = json.dumps({"command": "telemetry", "state": state_str})
        self.mqtt_client.publish(topic, payload)
        
        self.telemetry_btn.config(text=f"Telemetry: {state_str}")
        print(f"Sent {state_str} to {topic}")

    def reset_baseline(self):
        self.baseline = None
        print("Baseline reset requested")

    def clear_counts(self):
        self.total_in = 0
        self.total_out = 0

    def update_plot(self, frame):
        updated = False
        while not self.data_queue.empty():
            data = self.data_queue.get()
            updated = True
            
            zones = np.array(data["zones"])
            if len(zones) != 64: continue
            
            grid = zones.reshape((8, 8))
            self.grid_img.set_data(grid)
            
            # Processing Logic (Ported from Plot1.py)
            if self.baseline is None:
                self.baseline = grid.astype(float)
                self.noise = np.ones((8, 8)) * 50
                continue
                
            diff = self.baseline - grid
            abs_err = np.abs(diff)
            threshold = self.noise * NOISE_MULTIPLIER
            occupied = diff > threshold
            occupied[0:2, :] = False
            
            # Dilation logic
            dilated = occupied.copy()
            for y in range(8):
                for x in range(8):
                    if occupied[y, x]:
                        for dy in (-1, 0, 1):
                            for dx in (-1, 0, 1):
                                ny, nx = y + dy, x + dx
                                if 0 <= ny < 8 and 0 <= nx < 8:
                                    dilated[ny, nx] = True
            
            active_pixels = np.sum(dilated)
            stable_mask = ~dilated
            
            self.baseline[stable_mask] = (1 - BASELINE_ALPHA) * self.baseline[stable_mask] + BASELINE_ALPHA * grid[stable_mask]
            self.noise[stable_mask] = (1 - NOISE_ALPHA) * self.noise[stable_mask] + NOISE_ALPHA * abs_err[stable_mask]
            
            if active_pixels >= MIN_ACTIVE_PIXELS:
                ys, _ = np.where(dilated)
                centroid_y = np.mean(ys)
                if self.last_centroid_y is not None:
                    centroid_y = 0.7 * self.last_centroid_y + 0.3 * centroid_y
                    self.last_velocity = centroid_y - self.last_centroid_y
                self.last_centroid_y = centroid_y
                self.consecutive_misses = 0
                if not self.tracking_active:
                    self.trajectory = []
                    self.tracking_active = True
                self.trajectory.append(centroid_y)
            else:
                self.consecutive_misses += 1
                if self.tracking_active and self.consecutive_misses <= MAX_CONSECUTIVE_MISSES:
                    predicted = self.last_centroid_y + self.last_velocity
                    self.trajectory.append(predicted)
                    self.last_centroid_y = predicted
                else:
                    if self.tracking_active and len(self.trajectory) > 5:
                        start, end = self.trajectory[0], self.trajectory[-1]
                        if start > DOOR_LINE and end < DOOR_LINE:
                            self.total_in += 1
                        elif start < DOOR_LINE and end > DOOR_LINE:
                            self.total_out += 1
                        
                        self.in_label.config(text=f"IN: {self.total_in}")
                        self.out_label.config(text=f"OUT: {self.total_out}")

                    self.tracking_active = False
                    self.trajectory = []
                    self.last_centroid_y = None
                    self.last_velocity = 0
                    self.consecutive_misses = 0
            
            self.ax.set_title(f"IN: {self.total_in} | OUT: {self.total_out} | Track: {self.tracking_active}")

        if updated:
            return [self.grid_img]
        return []

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorPlotterApp(root)
    root.mainloop()
