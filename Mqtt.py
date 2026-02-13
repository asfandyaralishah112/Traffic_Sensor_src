import tkinter as tk
from tkinter import ttk
import json
import time
import threading
import paho.mqtt.client as mqtt
from datetime import datetime

# ================= MQTT CONFIG =================
MQTT_BROKER = "192.168.3.10"
MQTT_PORT = 1883
MQTT_USER = "Traffic_Sensor"
MQTT_PASS = "admin"

DEVICE_UID = "ESP32C6_COUNTER_001"

TOPIC_EVENTS = "door/counter/events"
TOPIC_STATUS = f"door/counter/status/{DEVICE_UID}"
TOPIC_COMMAND = f"door/counter/commands/{DEVICE_UID}"

# ================= GUI =================
root = tk.Tk()
root.title("ESP32 Smart Counter Control")
root.geometry("900x600")

main_frame = ttk.Frame(root, padding=10)
main_frame.pack(fill="both", expand=True)

# -------- Buttons --------
button_frame = ttk.Frame(main_frame)
button_frame.pack(fill="x", pady=5)

# -------- Event List --------
event_label = ttk.Label(main_frame, text="Events (IN / OUT)")
event_label.pack(anchor="w")

event_list = tk.Listbox(main_frame, height=12)
event_list.pack(fill="both", expand=True, pady=5)

# -------- Status List --------
status_label = ttk.Label(main_frame, text="Device Status")
status_label.pack(anchor="w")

status_list = tk.Listbox(main_frame, height=10)
status_list.pack(fill="both", expand=True)

# ================= MQTT =================
client = mqtt.Client()

def timestamp():
    return datetime.now().strftime("%H:%M:%S")

def add_event(text):
    event_list.insert(0, text)
    event_list.yview(0)

def add_status(text):
    status_list.insert(0, text)
    status_list.yview(0)

def on_connect(client, userdata, flags, rc):
    add_status(f"{timestamp()} MQTT Connected")
    client.subscribe(TOPIC_EVENTS)
    client.subscribe(TOPIC_STATUS)

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())

        if msg.topic == TOPIC_EVENTS:
            direction = payload.get("direction", "?")
            ts = timestamp()
            add_event(f"{ts}  {direction}")

        elif msg.topic == TOPIC_STATUS:
            status = payload.get("status", "unknown")
            ts = timestamp()
            add_status(f"{ts}  {status}")

    except Exception as e:
        add_status(f"{timestamp()} JSON error: {e}")

client.username_pw_set(MQTT_USER, MQTT_PASS)
client.on_connect = on_connect
client.on_message = on_message

def mqtt_loop():
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

threading.Thread(target=mqtt_loop, daemon=True).start()

# ================= COMMANDS =================
def send_calibrate():
    payload = json.dumps({"command": "calibrate"})
    client.publish(TOPIC_COMMAND, payload)
    add_status(f"{timestamp()} Calibration requested")

def send_ota():
    payload = json.dumps({"command": "update"})
    client.publish(TOPIC_COMMAND, payload)
    add_status(f"{timestamp()} OTA check requested")

cal_btn = ttk.Button(button_frame, text="Calibrate", command=send_calibrate)
cal_btn.pack(side="left", padx=5)

ota_btn = ttk.Button(button_frame, text="OTA Check", command=send_ota)
ota_btn.pack(side="left", padx=5)

# ================= START GUI =================
root.mainloop()
