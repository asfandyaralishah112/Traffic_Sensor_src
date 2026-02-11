import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt
import queue
import time

# --- Configuration ---
BROKER = "192.168.3.10"
TOPIC = "door/counter/telemetry/ESP32C6_COUNTER_001"
MQTT_USER = "Traffic_Sensor"
MQTT_PASS = "admin"

# --- Shared Data ---
# We use a queue for thread-safe communication between MQTT and Matplotlib
data_queue = queue.Queue()

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"✅ Connected to MQTT Broker ({BROKER})")
        client.subscribe(TOPIC)
        print(f"📡 Subscribed to: {TOPIC}")
    else:
        # rc codes for MQTT v5: 135 is Not Authorized
        print(f"❌ Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        
        # Debug: Print once every few messages to not flood console
        if not hasattr(on_message, "count"): on_message.count = 0
        on_message.count += 1
        if on_message.count % 20 == 0:
            print(f"📥 Received data #{on_message.count}, State: {data.get('state')}")

        # Put data in queue. If queue is too full, drop old data to keep it real-time.
        if data_queue.qsize() > 5:
            try:
                data_queue.get_nowait()
            except queue.Empty:
                pass
        data_queue.put(data)
    except Exception as e:
        print(f"⚠️ Error parsing message: {e}")

# --- MQTT Setup ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# Set credentials
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

print(f"🔄 Connecting to broker {BROKER}...")
try:
    client.connect(BROKER, 1883, 60)
    client.loop_start()  # Run MQTT in a background thread
except Exception as e:
    print(f"❌ MQTT Connection Error: {e}")

# --- Visualization Setup ---
fig, (ax_grid, ax_state) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [3, 1]})

# Grid visualization
# Using interpolation='nearest' to keep the 8x8 squares sharp
img = ax_grid.imshow(np.zeros((8, 8)), vmin=0, vmax=2500, cmap='viridis', interpolation='nearest')
plt.colorbar(img, ax=ax_grid, label='Distance (mm)')
ax_grid.set_title("ToF Sensor 8x8 Grid")

# State visualization
state_text = ax_state.text(0.5, 0.5, "IDLE", ha='center', va='center', fontsize=24, fontweight='bold')
ax_state.set_title("System State")
ax_state.axis('off')

# Mapping for firmware FlowState enum (from firmware.ino)
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
# Increase interval slightly to give more CPU time to MQTT thread if needed
ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()

# Clean up
print("Closing application...")
client.loop_stop()
client.disconnect()
