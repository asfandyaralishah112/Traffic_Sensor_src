import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import threading
import queue
import time

# --- Configuration ---
UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 5005
DEVICE_ID = "ESP32C6_COUNTER_001"

# --- Shared Data ---
data_queue = queue.Queue()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"📡 UDP Listener started on {UDP_IP}:{UDP_PORT}")
    
    count = 0
    while True:
        try:
            data, addr = sock.recvfrom(2048) # buffer size is 2048 bytes
            payload = data.decode()
            json_data = json.loads(payload)
            
            count += 1
            if count % 20 == 0:
                print(f"📥 Received UDP packet #{count} from {addr}, State: {json_data.get('state')}")

            # Put data in queue. If queue is too full, drop old data to keep it real-time.
            if data_queue.qsize() > 5:
                try:
                    data_queue.get_nowait()
                except queue.Empty:
                    pass
            data_queue.put(json_data)
        except Exception as e:
            print(f"⚠️ UDP Error: {e}")
            time.sleep(1)

# Start UDP Listener in background thread
listener_thread = threading.Thread(target=udp_listener, daemon=True)
listener_thread.start()

# --- Visualization Setup ---
fig, (ax_grid, ax_state) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [3, 1]})

# Grid visualization
img = ax_grid.imshow(np.zeros((8, 8)), vmin=0, vmax=2500, cmap='viridis', interpolation='nearest')
plt.colorbar(img, ax=ax_grid, label='Distance (mm)')
ax_grid.set_title("ToF Sensor 8x8 Grid (UDP)")

# State visualization
state_text = ax_state.text(0.5, 0.5, "IDLE", ha='center', va='center', fontsize=24, fontweight='bold')
ax_state.set_title("System State")
ax_state.axis('off')

# Mapping for firmware FlowState enum
STATE_MAP = {
    0: "IDLE",
    1: "FLOW_A",
    2: "FLOW_AM",
    3: "FLOW_MB",
    4: "B_AFTER_IN",
    5: "FLOW_B",
    6: "FLOW_BM",
    7: "FLOW_MA",
    8: "A_AFTER_OUT",
    9: "CLEARING"
}

def update(frame):
    updated = False
    while not data_queue.empty():
        data = data_queue.get()
        updated = True
        
        # 1. Update Grid
        if "zones" in data:
            zones = np.array(data["zones"])
            if len(zones) == 64:
                grid = zones.reshape((8, 8))
                img.set_data(grid)
        
        # 2. Update State
        if "state" in data:
            s_val = data["state"]
            s_str = STATE_MAP.get(s_val, f"UNKNOWN({s_val})")
            state_text.set_text(s_str)
            
            # Change background color based on state
            if "IN" in s_str or s_str == "B_AFTER_IN":
                ax_state.set_facecolor('#ccffcc') # Light green
            elif "OUT" in s_str or s_str == "A_AFTER_OUT":
                ax_state.set_facecolor('#ffcccc') # Light red
            else:
                ax_state.set_facecolor('white')
    
    if updated:
        return [img, state_text]
    return []

# Using FuncAnimation for smooth updates
ani = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False) # 30 FPS update

plt.tight_layout()
plt.show()

print("Closing application...")
